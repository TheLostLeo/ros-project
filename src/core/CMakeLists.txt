cmake_minimum_required(VERSION 2.8.3)
project(core)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
)

catkin_package()

install(PROGRAMS
  scripts/core_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
