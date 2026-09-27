"""Just the simulator and the driver -- no RViz, no behaviour nodes.

Handy when you want to attach your own stack to the robot, or to check that the
Webots side comes up before adding anything on top.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("mrrobot_bringup"), "launch",
                 "mrRobot.launch.py"])),
            launch_arguments={"rviz": "false", "mission": "false"}.items(),
        ),
    ])
