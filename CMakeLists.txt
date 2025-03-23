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

install(
  DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)
