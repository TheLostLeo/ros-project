cmake_minimum_required(VERSION 2.8.3)
project(esp)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

install(PROGRAMS
  scripts/esp_manager.py
  scripts/swarm_bot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
