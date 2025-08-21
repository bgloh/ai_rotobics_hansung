cmake_minimum_required(VERSION 3.0.2)
project(my_first_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_first_package
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
)

catkin_install_python(PROGRAMS
  scripts/my_publisher_node.py
  scripts/my_subscriber_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)