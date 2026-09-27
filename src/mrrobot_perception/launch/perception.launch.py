"""The colour detector and the object memory (with its find service)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("mrrobot_perception"),
                          "config", "perception.yaml")
    sim = {"use_sim_time": True}
    return LaunchDescription([
        Node(package="mrrobot_perception", executable="color_detector",
             name="color_detector", output="screen", parameters=[params, sim]),
        Node(package="mrrobot_perception", executable="object_memory",
             name="object_memory", output="screen", parameters=[params, sim]),
    ])
