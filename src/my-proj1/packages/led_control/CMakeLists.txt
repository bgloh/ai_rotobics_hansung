cmake_minimum_required(VERSION 3.0.2)
project(led_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES led_control
#  CATKIN_DEPENDS rospy std_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
# include
)

catkin_install_python(PROGRAMS
  scripts/led_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)