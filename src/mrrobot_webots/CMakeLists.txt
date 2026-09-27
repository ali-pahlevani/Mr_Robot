cmake_minimum_required(VERSION 3.8)
project(mrrobot_webots)

find_package(ament_cmake REQUIRED)
find_package(mrrobot_description REQUIRED)

# The PROTO is GENERATED from mrrobot_description's expanded URDF by
# scripts/build_proto.py (urdf2webots plus the sensors, contact materials and
# mesh fixes it documents), so the simulator model cannot drift from the URDF.
# The generated tree -- MrRobot.proto and every mesh it references, copied
# next to it -- lands in the build dir and is installed from there.
set(_urdf "${mrrobot_description_DIR}/../urdf/mrRobot.urdf")
set(_protos "${CMAKE_CURRENT_BINARY_DIR}/protos")
add_custom_command(
  OUTPUT ${_protos}/MrRobot.proto
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_proto.py
          --urdf ${_urdf} --out ${_protos}
  DEPENDS ${_urdf} scripts/build_proto.py
  COMMENT "Generating MrRobot.proto from the URDF"
  VERBATIM
)
add_custom_target(mrRobot_proto ALL DEPENDS ${_protos}/MrRobot.proto)

# WebotsLauncher copies the world into /tmp before starting Webots, which breaks
# every relative path inside it: "../protos/MrRobot.proto" becomes
# "/protos/MrRobot.proto" and the robot silently never loads -- the only symptom
# is the driver saying "Cannot connect to Webots instance" until it gives up.
# So install a world whose EXTERNPROTO is absolute.
set(_installed_protos "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/protos")
file(GLOB _worlds RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/worlds
     ${CMAKE_CURRENT_SOURCE_DIR}/worlds/*.wbt)
foreach(_world ${_worlds})
  # The rewrite happens at configure time, so tell CMake the world is an
  # input: without this an edited world is not re-copied until something
  # else forces a re-configure, and the sim quietly runs the old one.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
               "worlds/${_world}")
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/worlds/${_world}" _text)
  string(REPLACE "EXTERNPROTO \"../protos/"
                 "EXTERNPROTO \"${_installed_protos}/" _text "${_text}")
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/worlds/${_world}" "${_text}")
endforeach()

# protos/ in the source tree holds the hand-written PROTOs (the microwave
# with a workable door); they land in the same installed directory as the
# generated MrRobot.proto so one absolute EXTERNPROTO prefix serves both.
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/worlds ${_protos} protos
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/build_proto.py DESTINATION lib/${PROJECT_NAME})

ament_package()
