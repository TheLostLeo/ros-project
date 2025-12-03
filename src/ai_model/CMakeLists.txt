cmake_minimum_required(VERSION 2.8.3)
project(ai_model)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs
)

catkin_install_python(PROGRAMS
  scripts/ai_fish_monitor_node.py
  scripts/water_quality_node.py
  scripts/fish_behavior_node.py
  scripts/opencv_fish_presence_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
