"""Show the model on its own -- no simulator, no controllers.

Useful for checking the URDF in isolation: if a link is in the wrong place it
is much easier to see here than with Webots also in the picture. The sliders
come from joint_state_publisher_gui, which stands in for the controllers.
"""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mrrobot_description")
    # ros2_control and the Webots plugin are switched off: neither means
    # anything without a simulator, and both would just add noise here.
    # value_type=str matters: without it launch parses the expanded URDF as
    # YAML to guess the type, and dies on the leading "<?xml".
    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([pkg, "urdf", "mrRobot.urdf.xacro"]),
            " use_ros2_control:=false use_webots:=false",
        ]),
        value_type=str)

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([pkg, "rviz", "mrRobot.rviz"])],
        ),
    ])
