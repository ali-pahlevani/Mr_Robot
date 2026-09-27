"""Integration test: bring the whole stack up headless and check it
works end to end -- the clock ticks, every controller is active, Nav2 takes
the base to the honey station's staging pose, and the colour detector
reports the honey jar from there.

    colcon test --packages-select mrrobot_bringup
    launch_test src/mrrobot_bringup/test/test_bringup.py

Runs Webots with --no-rendering in fast mode (sensors still render off
screen), so it needs no display and finishes in a few minutes.
"""

import math
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.action import ActionClient
from rclpy.node import Node

STARTUP_S = 120.0
NAV_S = 120.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("mrrobot_bringup"), "launch", "mrRobot.launch.py")),
        launch_arguments={"headless": "true", "rviz": "false", "localization": "amcl",
                          "nav": "true", "moveit": "false", "perception": "true",
                          "mission": "false"}.items())
    return launch.LaunchDescription([bringup, launch_testing.actions.ReadyToTest()]), {}


class TestBringup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_bringup", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def wait_for(self, predicate, seconds, what):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail(f"timed out waiting for {what}")

    def test_1_clock_ticks(self):
        from rosgraph_msgs.msg import Clock
        got = []
        sub = self.node.create_subscription(Clock, "/clock", lambda m: got.append(m), 10)
        self.wait_for(lambda: len(got) > 20, STARTUP_S, "/clock")
        self.node.destroy_subscription(sub)
        self.assertGreater(got[-1].clock.sec + got[-1].clock.nanosec * 1e-9,
                           got[0].clock.sec + got[0].clock.nanosec * 1e-9)

    def test_2_controllers_active(self):
        from controller_manager_msgs.srv import ListControllers
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.assertTrue(client.wait_for_service(timeout_sec=STARTUP_S))
        # Discovery has to finish both ways before the first request: the
        # driver answering a client whose reply reader it had not matched yet
        # threw ("failed to send response") and took the whole driver down.
        self.spin(2.0)
        expected = {"joint_state_broadcaster", "base_controller", "lift_controller",
                    "head_controller", "left_arm_controller", "right_arm_controller",
                    "left_gripper_controller", "right_gripper_controller"}

        def active():
            fut = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
            res = fut.result()
            if res is not None and {c.name for c in res.controller
                                    if c.state == "active"} >= expected:
                return True
            self.spin(1.0)
            return False
        self.wait_for(active, STARTUP_S, "all eight controllers active")

    def test_3_navigate_to_staging(self):
        from nav2_msgs.action import NavigateToPose
        from action_msgs.msg import GoalStatus
        from lifecycle_msgs.srv import GetState
        client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")
        self.assertTrue(client.wait_for_server(timeout_sec=STARTUP_S))
        # The action server exists from the moment bt_navigator is configured
        # but rejects goals until the lifecycle manager has activated it,
        # which can be seconds later.
        state = self.node.create_client(GetState, "/bt_navigator/get_state")
        self.assertTrue(state.wait_for_service(timeout_sec=STARTUP_S))

        def nav2_active():
            fut = state.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
            res = fut.result()
            if res is not None and res.current_state.label == "active":
                return True
            self.spin(1.0)
            return False
        self.wait_for(nav2_active, STARTUP_S, "Nav2 active")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = 0.76, 0.36
        goal.pose.pose.orientation.w = 1.0
        self.spin(5.0)                       # let AMCL and the costmaps settle
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=10.0)
        handle = send.result()
        self.assertIsNotNone(handle)
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=NAV_S)
        self.assertTrue(result.done(), "NavigateToPose did not finish")
        self.assertEqual(result.result().status, GoalStatus.STATUS_SUCCEEDED)

    def test_4_detector_sees_honey(self):
        from vision_msgs.msg import Detection3DArray
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        import tf2_ros
        honey = (1.71, 0.70)        # where the jar stands in the world file
        # Nav2 stops anywhere within its goal tolerance (up to 20 cm and some
        # degrees from the staging pose), so the head is aimed at the jar
        # from wherever the base actually is.
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf, self.node)
        pose = {}

        def located():
            try:
                t = buf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                return False
            q = t.rotation
            pose["x"], pose["y"] = t.translation.x, t.translation.y
            pose["yaw"] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return True
        self.wait_for(located, 30.0, "the robot's pose in the map")
        bearing = math.atan2(honey[1] - pose["y"], honey[0] - pose["x"]) - pose["yaw"]
        head = ActionClient(self.node, FollowJointTrajectory, "/head_controller/follow_joint_trajectory")
        self.assertTrue(head.wait_for_server(timeout_sec=30.0))
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ["openarmx_head_pitch_joint", "openarmx_head_yaw_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [-0.35, math.atan2(math.sin(bearing), math.cos(bearing))]
        pt.time_from_start = Duration(sec=2)
        g.trajectory.points = [pt]
        send = head.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=10.0)
        # the honey detection nearest the jar: other yellow things in the
        # kitchen can be reported as honey too
        seen = []
        sub = self.node.create_subscription(
            Detection3DArray, "/mrRobot/detections",
            lambda m: seen.extend(d.bbox.center.position for d in m.detections
                                  if d.id == "honey"), 10)
        self.wait_for(lambda: len(seen) >= 5, 60.0, "honey detections")
        self.node.destroy_subscription(sub)
        p = min(seen, key=lambda q: math.hypot(q.x - honey[0], q.y - honey[1]))
        # allow AMCL's error
        self.assertLess(math.hypot(p.x - honey[0], p.y - honey[1]), 0.25)
        self.assertGreater(p.z, 0.85)
