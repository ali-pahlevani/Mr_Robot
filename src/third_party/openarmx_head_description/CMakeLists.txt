cmake_minimum_required(VERSION 3.22)
project(openarmx_head_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
