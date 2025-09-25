AI package

Nodes:
- ai_detector.py: subscribes to /camera/image_raw and publishes /ai/alert

Requires: OpenCV, cv_bridge, image_transport

Usage:
- If you have a local USB camera and want the node to open it directly, set the ROS param `~use_camera_device` to true (default). You can also set `~camera_id`.

Example launch (manual):
  rosparam set /ai_detector/use_camera_device true
  rosrun ai ai_detector.py

The node publishes JSON strings on `/ai/alert`, e.g.:
  {"state":"happy","num_fish":3,"avg_speed":4.2,"dispersion":72.5,"timestamp":...}

Notes:
- `cv_bridge` and `image_transport` must be installed for ROS integration. On ROS Noetic, install via apt: `sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport`.
- `opencv-python` and `numpy` are required for the standalone parts (pip install inside your ROS python environment if needed).

End-to-end test (WSL-friendly):
1. Download a sample video:
  ~/catkin_ws/src/ai/download_sample_video.sh
2. Build and source your workspace:
  cd ~/catkin_ws && catkin_make && source devel/setup.bash
3. Launch the test (this starts `publish_video.py` and `ai_detector.py`):
  roslaunch ai ai_test.launch
