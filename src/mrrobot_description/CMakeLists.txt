cmake_minimum_required(VERSION 3.8)
project(mrrobot_description)

find_package(ament_cmake REQUIRED)

# webots_ros2_driver takes robot_description as a path to a PLAIN URDF -- it
# does not run xacro. So expand it here, at build time, and install the result
# alongside the source. Everything else (robot_state_publisher, RViz) keeps
# using the .xacro directly.
find_program(XACRO_EXECUTABLE xacro REQUIRED)
# The expansion pulls in the OpenFleX packages' xacros, meshes lists and
# inertia yamls, so those are inputs too: without them an edited finger
# limit or mass sat in the source tree while the sim ran the old one.
find_package(openarmx_description REQUIRED)
find_package(openarmx_head_description REQUIRED)
find_package(lift_slide_description REQUIRED)
file(GLOB_RECURSE _vendored_inputs
     ${openarmx_description_DIR}/../urdf/* ${openarmx_description_DIR}/../config/*
     ${openarmx_head_description_DIR}/../urdf/* ${lift_slide_description_DIR}/../urdf/*)
file(GLOB _own_inputs ${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.xacro)
set(MRROBOT_URDF ${CMAKE_CURRENT_BINARY_DIR}/mrRobot.urdf)
add_custom_command(
  OUTPUT ${MRROBOT_URDF}
  COMMAND ${XACRO_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/urdf/mrRobot.urdf.xacro
          -o ${MRROBOT_URDF}
  DEPENDS ${_own_inputs} ${_vendored_inputs}
  COMMENT "Expanding mrRobot.urdf.xacro for webots_ros2_driver"
  VERBATIM
)
add_custom_target(mrRobot_urdf ALL DEPENDS ${MRROBOT_URDF})

install(FILES ${MRROBOT_URDF} DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY urdf meshes rviz launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
