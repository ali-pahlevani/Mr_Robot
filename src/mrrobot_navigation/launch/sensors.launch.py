"""What every consumer of the base's sensors needs, whichever mode is running:

  * the EKF (robot_localization): /wheel_odom + /imu -> /odometry/filtered
    and the odom->base_link transform. See config/ekf.yaml.
  * the odometry rate: /base_controller/odom -> /wheel_odom, the wheel
    odometry with its twist re-derived from its (exact) pose; the
    controller's own twist runs 6-7 % fast. See scripts/odom_rate.py.
  * the scan fixer: /scan -> /scan_filtered, reversed into a conventional
    counter-clockwise scan and with the robot's own chassis cut out. See
    scripts/scan_fixer.py for what goes wrong without each of those.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("mrrobot_navigation")
    return LaunchDescription([
        Node(package="robot_localization", executable="ekf_node",
             name="ekf_filter_node", output="screen",
             parameters=[os.path.join(share, "config", "ekf.yaml"),
                         {"use_sim_time": True}]),
        Node(package="mrrobot_navigation", executable="odom_rate.py",
             name="odom_rate", output="screen",
             parameters=[{"use_sim_time": True}]),
        Node(package="mrrobot_navigation", executable="scan_fixer.py",
             name="scan_fixer", output="screen",
             parameters=[{"use_sim_time": True}]),
    ])
