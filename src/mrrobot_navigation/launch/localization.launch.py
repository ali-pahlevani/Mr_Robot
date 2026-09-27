"""map_server + AMCL + their lifecycle manager, on the saved kitchen map.

RainBot's localization.launch.py pattern, minus the simulator and RViz, which
mrrobot_bringup owns.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("mrrobot_navigation")
    sim = {"use_sim_time": True}
    map_yaml = LaunchConfiguration("map")
    surface_yaml = LaunchConfiguration("surface_map")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map", default_value=os.path.join(share, "maps", "kitchen.yaml"),
            description="Map yaml for the costmaps (and RViz)."),
        # AMCL localizes on the same map thinned to its visible surfaces
        # (scripts/surface_map.py): on the full map, with walls 2-3 cells
        # thick, its estimate slid up to 14 cm along the lane after every
        # hop (measured) and the scan matcher had to pull it back.
        DeclareLaunchArgument(
            "surface_map",
            default_value=os.path.join(share, "maps", "kitchen_surface.yaml"),
            description="Map yaml AMCL localizes on (surfaces only)."),
        Node(package="nav2_map_server", executable="map_server",
             name="map_server", output="screen",
             parameters=[sim, {"yaml_filename": map_yaml}]),
        Node(package="nav2_map_server", executable="map_server",
             name="surface_map_server", output="screen",
             parameters=[sim, {"yaml_filename": surface_yaml, "topic_name": "map_surface"}]),
        # The scan matcher's own map: the costmaps' map with the dining set
        # as the lidar sees it, legs, instead of the solid block the costmaps
        # need (maps/README.md). Against the block every fit by the table
        # was refused, and the jam's dock there went by dead reckoning.
        Node(package="nav2_map_server", executable="map_server",
             name="reloc_map_server", output="screen",
             parameters=[sim, {"yaml_filename": os.path.join(share, "maps", "kitchen_reloc.yaml"),
                               "topic_name": "map_reloc"}]),
        Node(package="nav2_amcl", executable="amcl", name="amcl",
             output="screen",
             parameters=[os.path.join(share, "config", "amcl.yaml"), sim]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", output="screen",
             parameters=[sim, {"autostart": True,
                               "node_names": ["map_server", "surface_map_server",
                                              "reloc_map_server", "amcl"]}]),
        # /relocalize: slides the scan over the map and re-seeds AMCL where
        # it fits (see the script); the mission calls it after docking.
        Node(package="mrrobot_navigation", executable="scan_relocalize.py",
             name="scan_relocalize", output="screen",
             parameters=[sim, {"map_topic": "/map_reloc"}]),
    ])
