cmake_minimum_required(VERSION 3.0.2)
project(camera_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_control
#  CATKIN_DEPENDS rospy sensor_msgs cv_bridge
#  DEPENDS system_lib
)

include_directories(
# include
)

catkin_install_python(PROGRAMS
  scripts/camera_controller_node.py
  scripts/yolo_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)