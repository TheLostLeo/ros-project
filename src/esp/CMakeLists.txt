cmake_minimum_required(VERSION 2.8.3)
project(esp)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  std_srvs
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  geometry_msgs
  std_srvs
)

add_message_files(
  FILES
  BotCommand.msg
  BotStatus.msg
)

add_service_files(
  FILES
  TriggerCleaning.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs geometry_msgs std_srvs
)

catkin_install_python(PROGRAMS
  scripts/esp_websocket_node.py
  scripts/esp_decision_node.py
  scripts/swarm_manager_node.py
  scripts/ai_fish_monitor_node.py
  scripts/scheduler_node.py
  scripts/esp_master_node.py
  scripts/start_esp_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/esp_decision_node.py
  scripts/esp_websocket_node.py
  scripts/swarm_manager_node.py
  scripts/ai_fish_monitor_node.py
  scripts/scheduler_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

