cmake_minimum_required(VERSION 3.8)
project(mrrobot_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# The Python package holds the nodes AND the driver plugin: webots_ros2_driver
# imports mrrobot_perception.ground_truth.GroundTruth from this install.
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/color_detector
  scripts/object_memory
  scripts/detection_error
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_color_detector test/test_color_detector.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
