cmake_minimum_required(VERSION 3.0.2)
project(mecanum_control)

## 查找所需包
find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
)

## 声明消息文件
add_message_files(
  FILES
  MecanumCommand.msg
  MecanumStatus.msg
)

## 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
)

## 声明catkin包
catkin_package(
  CATKIN_DEPENDS rospy message_runtime std_msgs
)

## 指定include目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 安装Python执行文件
catkin_install_python(PROGRAMS
  src/mecanum_controller_node.py
  src/command_interface_node.py
  src/keyboard_control_node.py
  scripts/can_device_scanner.py
  scripts/setup_socketcan.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装Python模块
install(DIRECTORY src/mecanum_control
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)

## 安装消息文件
install(
  DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## 安装配置文件
install(
  DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## 安装启动文件（如果有）
# install(
#   DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
