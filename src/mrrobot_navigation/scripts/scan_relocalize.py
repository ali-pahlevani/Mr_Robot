#!/usr/bin/env python3
"""Scan-to-map relocalization on demand: /relocalize (std_srvs/Trigger).

AMCL, once its cloud has collapsed on a pose 10 cm off, has no particles near
the truth and nothing pulls it back while the robot stands still (measured
at the honey station: a +10 cm y bias that the jam station never showed).
This node does what a person would: take the current scan, slide it over the
map around AMCL's estimate, and keep the pose where it fits.

The fit is a brute-force search over dx, dy (+-0.20 m, 2 cm) -- and dyaw
(+-6 deg, 1 deg) unless the heading is taken from the odometry (yaw_from_odom,
see there) -- of the mean truncated distance from each scan point to the
nearest SURFACE cell's centre: only occupied cells that border the free
region the robot stands in count (a flood fill from its pose), because that
is what the lidar sees, the map's walls are 2-3 cells thick, and the map has
stray free cells behind them from rays that leaked through gaps while
mapping -- with every free-bordering cell counted, the fit had a second
minimum exactly one wall thickness away (measured, 10 cm)
(a distance transform on a 4x upsampled map with those centres marked,
so each lookup is O(1) and the score is not flat across a cell or a wall's
thickness -- with plain cell distances it was both, and the fit wandered
10 cm along the flat bottom), refined once at 5 mm / 0.25 deg. The winner is published on
/initialpose with a tight covariance, which re-seeds AMCL there -- if it fits
well (max_fit), or fits passably and is pinned down (max_fit_pinned,
min_rise: see there). The response message carries the correction, and how
well pinned the fit was.
"""

import math

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

import cv2


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScanRelocalize(Node):
    def __init__(self):
        super().__init__("scan_relocalize")
        self.declare_parameter("search_xy", 0.20)
        self.declare_parameter("search_yaw_deg", 6.0)
        self.declare_parameter("truncate", 0.15)
        self.declare_parameter("max_fit", 0.03)     # metres; a worse best fit is not applied
        # ...unless it is pinned down: moved 3 cm any way, it scores at least
        # min_rise worse. By the dining table the lidar sees chair legs the
        # map has not got and the best fit scored 4.2-4.6 cm, refused, while
        # it was within a centimetre of the truth, from starts up to 14 cm
        # off; the base then dead-reckoned the dock 3 cm out instead
        # (replayed against the simulator's ground truth). Pinned fits in the
        # kitchen rise 0.2-1.0 cm; a fit that can slide -- facing the counter
        # at 30 deg, with the north counter behind the lidar -- rises 0.1-0.2
        # and was 4-7 cm off.
        self.declare_parameter("max_fit_pinned", 0.05)
        self.declare_parameter("min_rise", 0.0035)
        # The heading is not searched but taken from the odometry: the EKF's
        # heading is the IMU's, which is the world's (0.01 deg off the
        # simulator's truth over a whole errand), and the map is the world's
        # frame, so map->odom never turns. Searched, the heading came out
        # +0.5 deg by the dining table and AMCL's scattered +-0.6 deg, a
        # centimetre at the hand 0.8 m out (replayed against the truth).
        self.declare_parameter("yaw_from_odom", True)
        # the map it matches against: the costmaps' /map, or one of its own
        # (localization.launch.py gives it /map_reloc)
        self.declare_parameter("map_topic", "/map")
        self.dist = None
        self.scan = None
        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value,
                                 self._on_map, map_qos)
        self.create_subscription(LaserScan, "/scan_filtered", self._on_scan, qos_profile_sensor_data)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_service(Trigger, "/relocalize", self._relocalize)
        # the start-up heading watch (_watch_heading), for its first minute
        self._watching = 60
        self._out = 0
        self.create_timer(1.0, self._watch_heading)
        self.get_logger().info("scan relocalizer ready; /relocalize")

    def _watch_heading(self):
        """map->odom must not turn: the map is the world's frame and the
        odometry's heading the IMU's, which is the world's. Now and then
        AMCL's first estimate came out a quarter turn out anyway (a start-up
        race: its initial pose goes in seconds before the simulator steps;
        1 bring-up in 4 to 6, measured), and a bare launch -- the launch
        test, anyone driving by hand -- then navigated 90 deg wrong. For the
        first minute, a turn over 5 deg two checks running (a race stays; a
        turn's passing wobble does not) re-seeds AMCL where it thinks the
        base is, with the odometry's heading. (The mission checks again.)"""
        if self._watching <= 0:
            return
        self._watching -= 1
        try:
            turn = yaw_of(self.tf.lookup_transform("map", "odom", rclpy.time.Time()).transform.rotation)
            est = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time()).transform
            heading = yaw_of(self.tf.lookup_transform("odom", "base_footprint",
                                                      rclpy.time.Time()).transform.rotation)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        if abs(math.remainder(turn, 2 * math.pi)) < math.radians(5.0):
            self._out = 0
            return
        self._out += 1
        if self._out < 2:
            return
        self._out = 0
        init = PoseWithCovarianceStamped()
        init.header.frame_id = "map"
        init.header.stamp = self.get_clock().now().to_msg()
        init.pose.pose.position.x, init.pose.pose.position.y = est.translation.x, est.translation.y
        init.pose.pose.orientation.z = math.sin(heading / 2)
        init.pose.pose.orientation.w = math.cos(heading / 2)
        init.pose.covariance[0] = init.pose.covariance[7] = 0.02 ** 2
        init.pose.covariance[35] = math.radians(2.0) ** 2
        self.pub.publish(init)
        self.get_logger().warn(
            f"AMCL is {math.degrees(turn):+.0f} deg out of line with the odometry; re-seeded at "
            f"({est.translation.x:.2f}, {est.translation.y:.2f}, {math.degrees(heading):+.0f} deg)")

    UP = 4      # upsampling of the map for the distance field

    def _on_map(self, msg):
        h, w = msg.info.height, msg.info.width
        self.grid = np.array(msg.data, dtype=np.int16).reshape(h, w)
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_res = msg.info.resolution
        self.res = msg.info.resolution / self.UP
        self.free = (self.grid >= 0) & (self.grid < 50)
        self.dist = None
        self.get_logger().info(f"map {w}x{h} @ {msg.info.resolution} m")

    def _field_from(self, x, y):
        """Distance field to the surfaces seen from the free region containing (x, y)."""
        h, w = self.grid.shape
        cx, cy = int((x - self.origin[0]) / self.map_res), int((y - self.origin[1]) / self.map_res)
        mask = np.zeros((h + 2, w + 2), np.uint8)
        region = self.free.astype(np.uint8).copy()
        if not (0 <= cx < w and 0 <= cy < h and region[cy, cx]):
            return False
        cv2.floodFill(region, mask, (cx, cy), 2)
        reachable = region == 2
        near = np.zeros_like(reachable)
        near[1:, :] |= reachable[:-1, :]
        near[:-1, :] |= reachable[1:, :]
        near[:, 1:] |= reachable[:, :-1]
        near[:, :-1] |= reachable[:, 1:]
        occ = np.argwhere((self.grid >= 50) & near)
        centres = np.ones((h * self.UP, w * self.UP), np.uint8)
        centres[occ[:, 0] * self.UP + self.UP // 2, occ[:, 1] * self.UP + self.UP // 2] = 0
        self.dist = cv2.distanceTransform(centres, cv2.DIST_L2, 5) * self.res
        return True

    def _on_scan(self, msg):
        self.scan = msg

    def _lookup(self, xs, ys):
        cx = ((xs - self.origin[0]) / self.res).astype(int)
        cy = ((ys - self.origin[1]) / self.res).astype(int)
        h, w = self.dist.shape
        inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
        out = np.full(xs.shape, self.get_parameter("truncate").value, dtype=float)
        out[inside] = np.minimum(self.dist[cy[inside], cx[inside]],
                                 self.get_parameter("truncate").value)
        return out

    # points of the chassis footprint (0.84 x 0.57 m, slightly shrunk) that must
    # lie in free map cells: a fit that puts the body inside a counter is not
    # a fit, however well the beams line up (measured: docked 8 cm from the
    # counter, the unconstrained search slid the robot 8 cm into it)
    FOOTPRINT = np.array([(x, y) for x in np.linspace(-0.3825, 0.3825, 7)
                          for y in np.linspace(-0.255, 0.255, 5)])

    def _score(self, px, py, angles, ranges, x, y, yaw):
        """Mean truncated distance of the scan placed at (x, y, yaw) -- lower is better.
        (px, py) is the lidar in base_footprint."""
        c, s = np.cos(yaw), np.sin(yaw)
        fx = x + c * self.FOOTPRINT[:, 0] - s * self.FOOTPRINT[:, 1]
        fy = y + s * self.FOOTPRINT[:, 0] + c * self.FOOTPRINT[:, 1]
        if not self._free(fx, fy).all():
            return 10.0
        lx, ly = x + c * px - s * py, y + s * px + c * py
        a = yaw + angles
        return self._lookup(lx + ranges * np.cos(a), ly + ranges * np.sin(a)).mean()

    def _rise(self, px, py, angles, ranges, x, y, yaw, sc, d=0.03):
        """How much worse the fit gets moved `d` either way, along the
        direction (of four) where that is least: how well it is pinned."""
        least = float("inf")
        for a in (0.0, 45.0, 90.0, 135.0):
            ux, uy = d * math.cos(math.radians(a)), d * math.sin(math.radians(a))
            up = self._score(px, py, angles, ranges, x + ux, y + uy, yaw)
            down = self._score(px, py, angles, ranges, x - ux, y - uy, yaw)
            least = min(least, (up + down) / 2 - sc)
        return least

    def _free(self, xs, ys):
        cx = ((xs - self.origin[0]) / self.map_res).astype(int)
        cy = ((ys - self.origin[1]) / self.map_res).astype(int)
        h, w = self.free.shape
        inside = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
        out = np.zeros(xs.shape, bool)
        out[inside] = self.free[cy[inside], cx[inside]]
        return out

    def _relocalize(self, request, response):
        if not hasattr(self, "grid") or self.scan is None:
            response.success = False
            response.message = "no map or scan yet"
            return response
        # One scan, and the estimate AT that scan's time: the answer is where
        # the base was when the scan was taken, and it is published with that
        # stamp. Stamped with now() instead, it was newer than anything in
        # AMCL's TF buffer, AMCL's lookup of the motion since then failed
        # ("extrapolation into the future", every call) and the pose was
        # applied as if it were the present -- harmless at rest, but a
        # correction handled while the base moves would snap the estimate
        # back by however far it had gone.
        s = self.scan
        stamp = rclpy.time.Time.from_msg(s.header.stamp)
        try:
            try:
                est = self.tf.lookup_transform("map", "base_footprint", stamp,
                                               timeout=rclpy.duration.Duration(seconds=0.2))
            except tf2_ros.ExtrapolationException:
                est = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            lid = self.tf.lookup_transform("base_footprint", s.header.frame_id,
                                           rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            response.success = False
            response.message = f"tf: {e}"
            return response
        x0, y0, yaw0 = est.transform.translation.x, est.transform.translation.y, yaw_of(est.transform.rotation)
        est_yaw = yaw0            # AMCL's, which the correction is reported against
        fixed = False
        if self.get_parameter("yaw_from_odom").value:
            try:
                try:
                    od = self.tf.lookup_transform("odom", "base_footprint", stamp,
                                                  timeout=rclpy.duration.Duration(seconds=0.2))
                except tf2_ros.ExtrapolationException:
                    od = self.tf.lookup_transform("odom", "base_footprint", rclpy.time.Time())
                yaw0, fixed = yaw_of(od.transform.rotation), True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass
        if not self._field_from(x0, y0):
            response.success = False
            response.message = "the estimate is not in free space; cannot tell which surfaces are visible"
            return response
        px, py = lid.transform.translation.x, lid.transform.translation.y
        r = np.array(s.ranges, dtype=float)
        ok = np.isfinite(r) & (r >= s.range_min) & (r <= s.range_max)
        angles = (s.angle_min + np.arange(len(r)) * s.angle_increment)[ok] + yaw_of(lid.transform.rotation)
        ranges = r[ok]
        if ranges.size < 30:
            response.success = False
            response.message = "too few scan returns"
            return response

        def search(cx, cy, cyaw, span, step, yspan, ystep):
            best = (self._score(px, py, angles, ranges, cx, cy, cyaw), cx, cy, cyaw)
            for dyaw in np.arange(-yspan, yspan + 1e-9, ystep):
                for dx in np.arange(-span, span + 1e-9, step):
                    for dy in np.arange(-span, span + 1e-9, step):
                        sc = self._score(px, py, angles, ranges, cx + dx, cy + dy, cyaw + dyaw)
                        if sc < best[0]:
                            best = (sc, cx + dx, cy + dy, cyaw + dyaw)
            return best

        before = self._score(px, py, angles, ranges, x0, y0, yaw0)
        span = self.get_parameter("search_xy").value
        yspan = 0.0 if fixed else math.radians(self.get_parameter("search_yaw_deg").value)
        sc, x, y, yaw = search(x0, y0, yaw0, span, 0.02, yspan, math.radians(1.0))
        # A poor best fit pinned against the edge of the window is the answer
        # lying OUTSIDE it: once a short hop ran 20 cm on odometry while the
        # base went nowhere, and every call after it came back with the same
        # (-0.230, +0.070) -- the window's edge plus the refinement -- at a
        # 10 cm fit, which the gate below then rightly refused. Look three
        # times as far, coarser, before refining. A poor fit in the MIDDLE of
        # the window (the dining table) is the map, not the window, and is
        # left to the gate as before.
        if (sc > self.get_parameter("max_fit").value
                and max(abs(x - x0), abs(y - y0)) >= span - 0.021):
            sc, x, y, yaw = search(x0, y0, yaw0, 3 * span, 0.04, 2 * yspan, math.radians(2.0))
            self.get_logger().info(
                f"relocalize: best fit at the window's edge; widened to "
                f"{3 * span:.2f} m, now ({x - x0:+.3f}, {y - y0:+.3f}) at {sc * 100:.1f} cm")
        sc, x, y, yaw = search(x, y, yaw, 0.03, 0.005, 0.0 if fixed else math.radians(1.5),
                               math.radians(0.25))
        shift = math.hypot(x - x0, y - y0)
        rise = self._rise(px, py, angles, ranges, x, y, yaw, sc)
        msg = (f"fit {before * 100:.1f} -> {sc * 100:.1f} cm; moved ({x - x0:+.3f}, {y - y0:+.3f}, "
               f"{math.degrees(yaw - est_yaw):+.1f} deg); pinned {rise * 100:.2f} cm")
        # A fit that is still poor is no fit: by the dining table, against a
        # map with the table painted solid, the best score was 5 cm and
        # applying it walked the estimate 13 cm sideways three calls running
        # (measured). Leave AMCL alone then -- unless it is pinned down.
        pinned = (sc <= self.get_parameter("max_fit_pinned").value
                  and rise >= self.get_parameter("min_rise").value)
        if sc > self.get_parameter("max_fit").value and not pinned:
            msg += "; poor fit, estimate left alone"
            self.get_logger().warn("relocalize: " + msg)
            response.success = False
            response.message = msg
            return response
        if shift > 0.005 or abs(math.remainder(yaw - est_yaw, 2 * math.pi)) > math.radians(0.3):
            init = PoseWithCovarianceStamped()
            init.header.frame_id = "map"
            init.header.stamp = s.header.stamp
            init.pose.pose.position.x, init.pose.pose.position.y = x, y
            init.pose.pose.orientation.z, init.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            init.pose.covariance[0] = init.pose.covariance[7] = 0.0004
            init.pose.covariance[35] = 0.0006
            self.pub.publish(init)
        self.get_logger().info("relocalize: " + msg)
        response.success = True
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelocalize()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
