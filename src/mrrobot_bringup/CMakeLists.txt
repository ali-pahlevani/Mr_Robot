cmake_minimum_required(VERSION 3.8)
project(mrrobot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # The whole stack, headless, exercised end to end. It is a launch test
  # and takes a few minutes; colcon test runs it with the rest.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_bringup.py TIMEOUT 600)
endif()

ament_package()
