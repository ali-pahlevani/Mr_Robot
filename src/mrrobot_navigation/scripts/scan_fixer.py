#!/usr/bin/env python3
"""/scan -> /scan_filtered: the lidar scan the way the mapping stack expects it.

Three things are wrong with the raw scan for slam_toolbox, AMCL and the costmaps:

  1. It runs CLOCKWISE. Webots publishes angle_min = +fov/2 with a NEGATIVE
     angle_increment. AMCL copes; Karto (slam_toolbox) does not, and maps the
     room mirrored -- every scan match then fights the last one and the map
     comes out as rotated copies of the kitchen stacked on each other. So the
     ranges are reversed here into a conventional counter-clockwise scan.

  2. It sees the robot. The lidar hangs under the top plate 1.5 cm ahead of
     the upper chassis box, and its rear half (151 of 360 rays, measured)
     returns the chassis at 0.09-0.29 m. Left in, SLAM draws a phantom arc
     behind the robot everywhere it goes and the costmaps put the robot inside
     an obstacle. Any return that lands inside the chassis box is dropped.

  3. Its angles are a fraction off. The driver reports the rays as spanning
     the whole field of view, increment fov / (n - 1); Webots spaces them
     fov / n from the +fov/2 edge. Matched against the map with the
     driver's angles, every scan relocalization came out rotated +0.47 deg
     (replayed against the simulator's truth, 38 matches); with these,
     +0.10 deg. At the table spot's 0.8 m that is a centimetre.

Pure Python at 360 points and 8.6 Hz is well under a millisecond a scan.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# Lidar pose in base_link, from the URDF; the chassis box, with a little margin.
LIDAR_X, LIDAR_Y = 0.3485, 0.0
BOX_X = (-0.4675, 0.425)   # bumper at 0.4199: the counter stays in view to the last cm
BOX_Y = (-0.34, 0.34)


class ScanFixer(Node):
    def __init__(self):
        super().__init__("scan_fixer")
        self._pub = self.create_publisher(LaserScan, "/scan_filtered",
                                          qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self._on_scan,
                                 qos_profile_sensor_data)
        self._angles = None
        self._n = 0

    def _on_scan(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.range_min, out.range_max = msg.range_min, msg.range_max
        out.scan_time, out.time_increment = msg.scan_time, 0.0
        n = len(msg.ranges)
        if msg.angle_increment < 0:
            ranges = list(reversed(msg.ranges))
            # (3) ray j is at +fov/2 - j fov/n, so reversed, ray i is at
            # -fov/2 + (i + 1) fov/n
            fov = -msg.angle_increment * (n - 1)
            out.angle_increment = fov / n
            out.angle_min = -fov / 2 + out.angle_increment
            out.angle_max = out.angle_min + (n - 1) * out.angle_increment
        else:
            ranges = list(msg.ranges)
            out.angle_min, out.angle_max = msg.angle_min, msg.angle_max
            out.angle_increment = msg.angle_increment
        if self._angles is None or self._n != n:
            self._n = n
            self._angles = [(math.cos(out.angle_min + i * out.angle_increment),
                             math.sin(out.angle_min + i * out.angle_increment))
                            for i in range(n)]
        for i, r in enumerate(ranges):
            if not (msg.range_min <= r <= msg.range_max):
                ranges[i] = math.inf
                continue
            c, s = self._angles[i]
            x, y = LIDAR_X + r * c, LIDAR_Y + r * s
            if BOX_X[0] <= x <= BOX_X[1] and BOX_Y[0] <= y <= BOX_Y[1]:
                ranges[i] = math.nan
        out.ranges = ranges
        out.intensities = []
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFixer()
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
