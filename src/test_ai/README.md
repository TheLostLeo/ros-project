# Test AI Simulator

Publishes randomized AI-like signals to ROS topics and triggers system commands based on simple rules. Useful for end-to-end demos without a real model or camera.

## What it does
- Publishes JSON strings on:
  - `/ai/fish_presence` → `{label: fish|no_fish, conf: 0..1, timestamp}`
  - `/ai/fish_behavior` → `{label: calm|active|schooling|stressed, conf, timestamp}`
  - `/ai/water_quality` → `{sharpness, saturation_mean, turbidity_score, quality, stress_index, timestamp}`
- Emits `/system/command` as `start` when fish are calm AND water quality is poor.
- Emits `/system/command` as `stop` when fish become `active` or `stressed` while cleaning is active.

### New: OpenCV-based behavior from video
- Node: `opencv_behavior_node.py`
- Launch: `roslaunch test_ai opencv_behavior.launch source:=0` (use camera index) or `source:=/path/to/video.mp4`
- Publishes on `/ai/fish_behavior` JSON like:
  `{ "label": "calm"|"disturbed", "conf": 0..1, "motion_ratio": 0..1, "source": "opencv-mhi", "timestamp": ... }`
  - Tunables: `pixel_diff_thresh`, `motion_ratio_thresh`, `ema_window_sec`, `fps_limit`

## Parameters
- `period_sec` (float, default: 10.0): seconds between publish cycles.
- `rate_hz` (float, optional): overrides period if provided.
- `enable_presence` (bool, default: true)
- `enable_behavior` (bool, default: true)
- `enable_quality` (bool, default: true)

## Run inside Docker
```bash
# From repository root
cd docker
# Build and start container (first time may take a while)
docker compose up -d
# Enter the container
docker exec -it swarm_ros_dev bash
# Source ROS and workspace (usually done by container setup)
source /opt/ros/noetic/setup.bash && source /home/dev_ws/devel/setup.bash
# Launch simulator (10-second cadence by default)
roslaunch test_ai test_ai.launch period_sec:=10.0
```

Observe outputs in another terminal:
```bash
# Inside the same container
rostopic echo /ai/fish_behavior
rostopic echo /ai/water_quality
rostopic echo /system/command
```

Tip: Run your manager stack so `/system/command` is acted upon (e.g., `swarm_manager_node`).

## Run on host (bare-metal)
```bash
# Source ROS and your catkin workspace
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash  # adjust to your workspace
# Ensure roscore is running
roscore &
# In a new terminal, launch the simulator
roslaunch test_ai test_ai.launch
```

## Notes
- The messages are `std_msgs/String` containing JSON. Consumers should parse the JSON payload if needed.
- Default behavior cadence is low (0.1 Hz) to avoid log spam; adjust `period_sec` as needed.
- Rules are intentionally simple to exercise orchestration paths without a trained model.
