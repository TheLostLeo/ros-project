cmake_minimum_required(VERSION 2.8.3)
project(ai)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
)

catkin_package()

install(PROGRAMS
  scripts/ai_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
