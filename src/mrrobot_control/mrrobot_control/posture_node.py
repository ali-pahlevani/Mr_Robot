"""Turns named postures into joint trajectories.

Publishes to the JointTrajectoryControllers rather than using their action
servers: a posture is a single "go here" with no need to monitor progress, and
the topic interface keeps this node free of action bookkeeping.

    ros2 topic pub --once /mrRobot/posture std_msgs/String "data: ready"
    ros2 topic pub --once /mrRobot/gripper std_msgs/String "data: open"
"""

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mrrobot_control import arm_kinematics as k

ARM_TOPICS = {"left": "/left_arm_controller/joint_trajectory",
              "right": "/right_arm_controller/joint_trajectory"}
GRIPPER_TOPICS = {"left": "/left_gripper_controller/joint_trajectory",
                  "right": "/right_gripper_controller/joint_trajectory"}
HEAD_TOPIC = "/head_controller/joint_trajectory"
LIFT_TOPIC = "/lift_controller/joint_trajectory"

# Head pitch that goes with each posture, so the robot looks where it works.
# OpenFleX's head pitch is about x: negative nods DOWN (limits -1.086..0.403).
HEAD_TILT = {"stow": 0.0, "carry": -0.10, "ready": -0.15}
# Lift height per posture, metres about mid travel (-0.75..0.40).
LIFT = {"stow": 0.0, "carry": 0.0, "ready": 0.0}


def arm_joint_names(side):
    return [f"openarmx_{side}_{j}" for j in k.ARM_JOINTS]


def finger_joint_names(side):
    return [f"openarmx_{side}_finger_joint1", f"openarmx_{side}_finger_joint2"]


class PostureNode(Node):
    def __init__(self):
        super().__init__("mrRobot_posture")
        self.declare_parameter("move_time", 2.5)

        self._arms = {s: self.create_publisher(JointTrajectory, t, 10)
                      for s, t in ARM_TOPICS.items()}
        self._grippers = {s: self.create_publisher(JointTrajectory, t, 10)
                          for s, t in GRIPPER_TOPICS.items()}
        self._head = self.create_publisher(JointTrajectory, HEAD_TOPIC, 10)
        self._lift = self.create_publisher(JointTrajectory, LIFT_TOPIC, 10)

        self.create_subscription(String, "/mrRobot/posture", self._on_posture, 10)
        self.create_subscription(String, "/mrRobot/gripper", self._on_gripper, 10)
        self.get_logger().info(
            f"ready; postures: {', '.join(sorted(k.POSTURES))}")

    # ------------------------------------------------------------------
    def _send(self, pub, names, positions, seconds=None):
        seconds = seconds or self.get_parameter("move_time").value
        msg = JointTrajectory()
        msg.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=int(seconds),
                                         nanosec=int((seconds % 1) * 1e9))
        msg.points = [point]
        pub.publish(msg)

    def set_posture(self, name):
        if name not in k.POSTURES:
            self.get_logger().warn(
                f"unknown posture '{name}'; have {sorted(k.POSTURES)}")
            return False
        for side, angles in k.POSTURES[name].items():
            self._send(self._arms[side], arm_joint_names(side),
                       [angles[j] for j in k.ARM_JOINTS])
        self._send(self._head, ["openarmx_head_pitch_joint", "openarmx_head_yaw_joint"],
                   [HEAD_TILT.get(name, 0.0), 0.0])
        self._send(self._lift, ["lift_joint"], [LIFT.get(name, 0.0)], seconds=5.0)
        self.get_logger().info(f"posture -> {name}")
        return True

    def set_gripper(self, opening, side=None):
        for s in (GRIPPER_TOPICS if side is None else (side,)):
            self._send(self._grippers[s], finger_joint_names(s),
                       [opening, opening], seconds=1.5)

    # ------------------------------------------------------------------
    def _on_posture(self, msg):
        self.set_posture(msg.data.strip())

    def _on_gripper(self, msg):
        want = msg.data.strip().lower()
        if want in ("open", "o"):
            self.set_gripper(k.FINGER_OPEN)
        elif want in ("close", "closed", "c"):
            self.set_gripper(k.FINGER_CLOSED)
        else:
            self.get_logger().warn(f"gripper: expected open/close, got '{want}'")


def main(args=None):
    rclpy.init(args=args)
    node = PostureNode()
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
