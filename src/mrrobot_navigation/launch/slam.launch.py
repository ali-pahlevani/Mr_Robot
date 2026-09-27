"""slam_toolbox, online async, for mapping the kitchen.

Supplies map->odom on top of the EKF's odom->base_link and builds /map. Run
map_kitchen.py to drive the tour, then save with map_saver_cli -- see
maps/README.md for the origin shift that makes the saved map's frame coincide
with the Webots world.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("mrrobot_navigation"),
                          "config", "slam_toolbox.yaml")
    return LaunchDescription([
        Node(package="slam_toolbox", executable="async_slam_toolbox_node",
             name="slam_toolbox", output="screen",
             parameters=[params, {"use_sim_time": True}]),
    ])
