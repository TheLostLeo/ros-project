cmake_minimum_required(VERSION 2.8.3)
project(swarm_manage)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  message_generation
  actionlib_msgs
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

add_action_files(
  DIRECTORY action
  FILES CleanTask.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs std_srvs actionlib_msgs
)

catkin_install_python(PROGRAMS
  scripts/esp_websocket_node.py
  scripts/esp_decision_node.py
  scripts/esp_decision_action_server.py
  scripts/swarm_manager_node.py
  scripts/clean_task_test_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
