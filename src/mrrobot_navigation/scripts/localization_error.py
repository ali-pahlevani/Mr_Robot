#!/usr/bin/env python3
"""Localization error against Webots ground truth.

Compares the pose the rest of the stack believes -- the map->base_link
transform, i.e. AMCL or slam_toolbox on top of the EKF -- with the true pose
from the supervisor plugin (/ground_truth/robot), falling back to /gps for
position if that topic is absent. Publishes the position error on
/localization_error (std_msgs/Float32), logs running statistics, and prints
a summary on shutdown. Optionally writes every sample to a CSV.

    ros2 run mrrobot_navigation localization_error.py
    ros2 run mrrobot_navigation localization_error.py --ros-args -p csv:=/tmp/loc.csv

The numbers this produces are what REACH_MARGIN and the creep limit in the
mission are based on; see mrrobot_navigation/maps/README.md.
"""

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LocalizationError(Node):
    def __init__(self):
        super().__init__("localization_error")
        self.declare_parameter("csv", "")
        self.declare_parameter("period", 0.25)
        self.declare_parameter("log_every", 2.0)
        self._truth = None
        self._gps = None
        self._samples = []
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        self.create_subscription(PoseStamped, "/ground_truth/robot",
                                 self._on_truth, 10)
        self.create_subscription(NavSatFix, "/gps", self._on_gps, 10)
        self._pub = self.create_publisher(Float32, "/localization_error", 10)
        csv = self.get_parameter("csv").value
        self._csv = open(csv, "w") if csv else None
        if self._csv:
            self._csv.write("t,est_x,est_y,est_yaw,true_x,true_y,true_yaw,"
                            "err_pos,err_yaw\n")
        self._last_log = 0.0
        self.create_timer(self.get_parameter("period").value, self._tick)

    def _on_truth(self, msg):
        self._truth = msg

    def _on_gps(self, msg):
        self._gps = msg

    def _tick(self):
        try:
            tf = self._buffer.lookup_transform("map", "base_link",
                                               rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        est = (tf.transform.translation.x, tf.transform.translation.y,
               yaw_of(tf.transform.rotation))
        if self._truth is not None:
            p, q = self._truth.pose.position, self._truth.pose.orientation
            true = (p.x, p.y, yaw_of(q))
        elif self._gps is not None:
            # webots_ros2 puts local x/y in latitude/longitude when the GPS is
            # in "local" mode; yaw is unavailable this way.
            true = (self._gps.latitude, self._gps.longitude, None)
        else:
            return
        err_pos = math.hypot(est[0] - true[0], est[1] - true[1])
        err_yaw = (None if true[2] is None else
                   (est[2] - true[2] + math.pi) % (2 * math.pi) - math.pi)
        t = self.get_clock().now().nanoseconds * 1e-9
        self._samples.append((err_pos, err_yaw))
        self._pub.publish(Float32(data=float(err_pos)))
        if self._csv:
            self._csv.write(f"{t:.3f},{est[0]:.4f},{est[1]:.4f},{est[2]:.4f},"
                            f"{true[0]:.4f},{true[1]:.4f},"
                            f"{'' if true[2] is None else f'{true[2]:.4f}'},"
                            f"{err_pos:.4f},"
                            f"{'' if err_yaw is None else f'{err_yaw:.4f}'}\n")
        if t - self._last_log >= self.get_parameter("log_every").value:
            self._last_log = t
            self.get_logger().info(self.summary(err_pos, err_yaw))

    def summary(self, now_pos=None, now_yaw=None):
        pos = [s[0] for s in self._samples]
        yaws = [abs(s[1]) for s in self._samples if s[1] is not None]
        if not pos:
            return "no samples"
        rms = math.sqrt(sum(p * p for p in pos) / len(pos))
        out = (f"pos err {now_pos:.3f} m" if now_pos is not None else "pos err")
        out += (f" (mean {sum(pos)/len(pos):.3f}, rms {rms:.3f}, "
                f"max {max(pos):.3f}, n={len(pos)})")
        if yaws:
            out += (f"; yaw err "
                    + (f"{math.degrees(now_yaw):+.1f} deg" if now_yaw is not None else "")
                    + f" (mean {math.degrees(sum(yaws)/len(yaws)):.1f}, "
                    f"max {math.degrees(max(yaws)):.1f} deg)")
        return out


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("summary: " + node.summary())
        if node._csv:
            node._csv.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
