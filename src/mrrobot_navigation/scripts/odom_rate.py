#!/usr/bin/env python3
"""/base_controller/odom -> /wheel_odom: the wheel odometry with its twist
taken from its own pose over a short window, for the EKF.

The diff-drive controller's POSE is exact (a 0.4 m leg: 37.4 cm true, 37.4 cm
odometry), but its TWIST came out 6-7 % fast, and the EKF, which integrates
the twist, drove every straight move that much too far (measured: 39.5 cm for
the same 37.4, and 3 cm of every 40 cm back-out). The controller's clock is
the /clock topic as the driver last received it, so the time between two of
its updates jumps between 0, 16, 32 and 48 ms while the wheels move a steady
16 ms of travel per physics step; it averages distance / that time over its
rolling window, and the mean of a ratio with a jittery denominator is larger
than the ratio of the means.

Here the velocity is the pose's change over the last WINDOW seconds divided
by the time between those two stamps: the jitter then only enters through
the two ends, a few per cent of the window instead of the whole of every
step. It lags by half the window, which the EKF sees as a slightly late
velocity and still integrates to the right distance.
"""

import math
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

WINDOW = 0.12      # s


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomRate(Node):
    def __init__(self):
        super().__init__("odom_rate")
        self._hist = deque()
        self._pub = self.create_publisher(Odometry, "/wheel_odom", 20)
        self.create_subscription(Odometry, "/base_controller/odom", self._on_odom, 50)

    def _on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose
        yaw = yaw_of(p.orientation)
        if self._hist and t <= self._hist[-1][0]:
            return                  # a repeated stamp: nothing new to difference
        self._hist.append((t, p.position.x, p.position.y, yaw))
        while len(self._hist) > 2 and t - self._hist[1][0] >= WINDOW:
            self._hist.popleft()
        t0, x0, y0, yaw0 = self._hist[0]
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist.covariance = msg.twist.covariance
        dt = t - t0
        if dt > 1e-3:
            dx, dy = p.position.x - x0, p.position.y - y0
            dyaw = math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))
            mid = yaw0 + dyaw / 2.0
            out.twist.twist.linear.x = (math.cos(mid) * dx + math.sin(mid) * dy) / dt
            out.twist.twist.linear.y = (-math.sin(mid) * dx + math.cos(mid) * dy) / dt
            out.twist.twist.angular.z = dyaw / dt
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRate()
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
