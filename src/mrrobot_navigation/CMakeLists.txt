cmake_minimum_required(VERSION 3.8)
project(mrrobot_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/localization_error.py
  scripts/map_kitchen.py
  scripts/scan_fixer.py
  scripts/odom_rate.py
  scripts/finish_map.py
  scripts/scan_relocalize.py
  scripts/surface_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
