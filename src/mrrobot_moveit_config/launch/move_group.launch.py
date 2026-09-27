"""move_group for mrRobot.

The URDF is mrrobot_description's own xacro (expanded here with the Webots
device block off -- MoveIt does not need it), the SRDF and the rest come from
this package's config/. Execution goes through the ros2_control trajectory
controllers that mrrobot_bringup already spawns; see moveit_controllers.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory("mrrobot_description"),
                        "urdf", "mrRobot.urdf.xacro")
    moveit_config = (
        MoveItConfigsBuilder("mrRobot", package_name="mrrobot_moveit_config")
        .robot_description(file_path=urdf,
                           mappings={"use_webots": "false",
                                     "use_ros2_control": "false"})
        .robot_description_semantic(file_path="config/mrRobot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True,
             # 0.25 rad, not the default 0.01: the microwave door drags the
             # hand a few centimetres between the steps of a pull, and the
             # next step must not be refused for it (measured: refused at
             # 0.05 after the door had opened 28 degrees).
             "trajectory_execution.allowed_start_tolerance": 0.25,
             # more than the controllers' goal_time (2.5 s): a hand that
             # ends a step pressed on the door is the controller's call
             # (goal tolerance), not move_group's clock (measured: cut off
             # at 2.6 s with the margin at 2.0)
             "trajectory_execution.allowed_goal_duration_margin": 3.5,
             "trajectory_execution.allowed_execution_duration_scaling": 2.0},
        ],
    )
    return LaunchDescription([move_group])
