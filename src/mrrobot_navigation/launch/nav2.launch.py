"""The Nav2 servers: controller, smoother, planner, behaviours, bt_navigator.

From RainBot's nav2.launch.py. As there, controller_server's cmd_vel is
remapped straight onto the drive controller: twist_mux's binary is broken on
this machine, and Nav2 and the mission's own docking creep never run at the
same time, so nothing needs arbitrating. The map->odom transform comes from
whichever of localization.launch.py (AMCL) or slam.launch.py is running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("mrrobot_navigation")
    params = os.path.join(share, "config", "nav2_params.yaml")
    bt_xml = os.path.join(share, "config", "bt_navigate_to_pose.xml")
    sim = {"use_sim_time": True}
    lifecycle_nodes = ["controller_server", "smoother_server",
                       "planner_server", "behavior_server", "bt_navigator"]

    return LaunchDescription([
        Node(package="nav2_controller", executable="controller_server",
             output="screen", parameters=[params, sim],
             remappings=[("cmd_vel", "/base_controller/cmd_vel_unstamped")]),
        Node(package="nav2_smoother", executable="smoother_server",
             name="smoother_server", output="screen",
             parameters=[params, sim]),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen",
             parameters=[params, sim]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen",
             parameters=[params, sim],
             remappings=[("cmd_vel", "/base_controller/cmd_vel_unstamped")]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen",
             parameters=[params, sim,
                         {"default_nav_to_pose_bt_xml": bt_xml}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[sim, {"autostart": True,
                               "node_names": lifecycle_nodes}]),
    ])
