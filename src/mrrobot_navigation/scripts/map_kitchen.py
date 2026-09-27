#!/usr/bin/env python3
"""Drive a fixed tour of the kitchen so slam_toolbox can map it.

Odometry-closed-loop moves on /odometry/filtered, in the odom frame, which
starts at the spawn point with world-aligned axes (the EKF takes its heading
from the IMU). The tour is: a full spin where it stands, out to the lane,
down the lane to the fridge end and back, then west past the dining chairs to
the far wall, and home. That puts the lidar within 2 m of every surface the
mission will localize against.

    ros2 launch mrrobot_bringup mrRobot.launch.py localization:=slam
    ros2 run mrrobot_navigation map_kitchen.py
    ros2 run nav2_map_server map_saver_cli -f src/mrrobot_navigation/maps/kitchen
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node

SPAWN = (0.15, 0.20)     # world position of odom's origin
# World-frame waypoints, (x, y); "spin" means one slow full turn.
TOUR = [
    "spin",
    (0.76, 0.20),
    (0.76, -1.55),
    "spin",
    (0.76, 0.50),
    (0.15, 0.50),
    (-1.20, 0.30),
    "spin",
    (0.15, 0.20),
]
SPEED = 0.25
TURN = 0.5


class MapKitchen(Node):
    def __init__(self):
        super().__init__("map_kitchen")
        self._cmd = self.create_publisher(
            Twist, "/base_controller/cmd_vel_unstamped", 10)
        self._odom = None
        self.create_subscription(Odometry, "/odometry/filtered",
                                 self._on_odom, 10)

    def _on_odom(self, msg):
        self._odom = msg

    def here(self):
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return p.x + SPAWN[0], p.y + SPAWN[1], yaw

    def drive(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self._cmd.publish(msg)

    def spin_for(self, seconds):
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def turn_to(self, heading, tol=0.04, timeout=30.0):
        end = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.get_clock().now() < end:
            err = (heading - self.here()[2] + math.pi) % (2 * math.pi) - math.pi
            if abs(err) < tol:
                self.drive(0, 0)
                self.spin_for(0.3)
                return True
            rate = max(0.25, min(TURN, 2.0 * abs(err)))
            self.drive(0.0, rate if err > 0 else -rate)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.drive(0, 0)
        return False

    def goto(self, x, y, tol=0.08, timeout=60.0):
        cx, cy, _ = self.here()
        self.turn_to(math.atan2(y - cy, x - cx))
        end = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.get_clock().now() < end:
            cx, cy, ch = self.here()
            dist = math.hypot(x - cx, y - cy)
            if dist < tol:
                break
            want = math.atan2(y - cy, x - cx)
            err = (want - ch + math.pi) % (2 * math.pi) - math.pi
            if abs(err) > 0.8:
                self.drive(0.0, max(-TURN, min(TURN, 2.0 * err)))
            else:
                speed = max(0.08, min(SPEED, 1.5 * dist)) * math.cos(err)
                self.drive(speed, max(-1.0, min(1.0, 1.5 * err)))
            rclpy.spin_once(self, timeout_sec=0.02)
        self.drive(0, 0)
        self.spin_for(0.5)

    def full_spin(self):
        start = self.here()[2]
        # Three legs of 120 degrees so the wrap-around cannot fool turn_to.
        for k in (1, 2, 3):
            self.turn_to(start + k * 2 * math.pi / 3)
        self.spin_for(0.5)

    def run(self):
        while rclpy.ok() and self.here() is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.spin_for(2.0)
        for leg in TOUR:
            if leg == "spin":
                self.get_logger().info("spinning")
                self.full_spin()
            else:
                self.get_logger().info(f"-> {leg}")
                self.goto(*leg)
            x, y, h = self.here()
            self.get_logger().info(
                f"   at ({x:.2f}, {y:.2f}) heading {math.degrees(h):.0f} deg")
        self.get_logger().info("tour done; save the map now")


def main(args=None):
    rclpy.init(args=args)
    node = MapKitchen()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.drive(0, 0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
