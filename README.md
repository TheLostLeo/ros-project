Catkin workspace for Swarm Aquarium project

Packages:
- esp: ESP32 communication, websocket + swarm bot interfaces
- core: central manager, start/stop services, communication between packages
- ai: OpenCV-based simple fish behavior detector using cv_bridge

Build:
  cd ~/catkin_ws
  catkin_make

Run examples:
  source devel/setup.bash
  roslaunch core core.launch

Helper script:
  ~/catkin_ws/build_and_run.sh

Notes:
  - Ensure ROS1 (e.g., Noetic) is installed before building. Install cv_bridge and image_transport for the `ai` package.
  - The Python scripts are lightweight placeholders. Replace them with real implementations for ESP websocket communication and AI models.
