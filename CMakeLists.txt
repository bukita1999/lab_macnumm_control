cmake_minimum_required(VERSION 3.0.2)
project(mecanum_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

add_message_files(
  FILES
  MecanumCommand.msg
  MecanumStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mecanum_controller_node src/mecanum_controller_node.py)
target_link_libraries(mecanum_controller_node ${catkin_LIBRARIES})
add_executable(command_interface_node src/command_interface_node.py)
target_link_libraries(command_interface_node ${catkin_LIBRARIES})
add_executable(keyboard_control_node src/keyboard_control_node.py)
target_link_libraries(keyboard_control_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  src/mecanum_controller_node.py
  src/command_interface_node.py
  src/keyboard_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)
