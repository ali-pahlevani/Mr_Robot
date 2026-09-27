cmake_minimum_required(VERSION 3.8)
project(mrrobot_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The SRDF names joints and links; they must be the URDF's. Same spirit as
  # the URDF<->PROTO cross-check in mrrobot_control.
  ament_add_pytest_test(test_srdf_matches_urdf test/test_srdf_matches_urdf.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
