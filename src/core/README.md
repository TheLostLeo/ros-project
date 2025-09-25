Core package

Nodes:
- core_manager.py: provides start_system and stop_system services and publishes /swarm_cmd
- Launch: launch/core.launch

Schedule mode
---------------
You can schedule cleaning times using the ROS parameter `/core_manager/schedules` or by publishing JSON commands to `/core/schedule_update` (std_msgs/String).

Parameter format (example):
  rosparam set /core_manager/schedules "[{'start':'09:00','stop':'09:05'},{'start':'18:00'}]"

Runtime updates (publish JSON to `/core/schedule_update`):
- Add a schedule: `{"op":"add","start":"HH:MM","stop":"HH:MM"}`
- Clear schedules: `{"op":"clear"}`
- Set schedules list: `{"op":"set","schedules":[{"start":"09:00","stop":"09:05"}]}`

The core manager will publish `/swarm_cmd` messages (`start`, `stop`, `clean`) according to the schedule and will avoid retriggering within the same day.
