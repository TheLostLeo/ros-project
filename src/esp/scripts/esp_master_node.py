#!/usr/bin/env python3
"""Enhanced ESP master node.

Features:
- Creates latched publishers for each bot's /esp/<id>/cmd and /esp/<id>/status
- Publishes an initial 'start' to ESP-1 on startup
- Provides services to start/stop all bots
- Exposes a broadcast topic `/esp/broadcast_cmd` to send arbitrary commands to all bots
- Optional periodic scheduler to auto-start bots every N seconds (configurable via param)
"""

import rospy
import socket
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


class ESPMasterNode:
    def __init__(self):
        rospy.init_node("esp_master_node", anonymous=True)

        # How many bots? Default = 3
        self.num_bots = rospy.get_param("~num_bots", 3)

        # Scheduler interval in seconds (0 to disable)
        self.auto_start_interval = rospy.get_param("~auto_start_interval", 0)

        # Publishers for each ESP command + status
        self.cmd_pubs = {}
        self.status_pubs = {}

        for bot_id in range(1, self.num_bots + 1):
            cmd_topic = f"/esp/{bot_id}/cmd"
            status_topic = f"/esp/{bot_id}/status"

            self.cmd_pubs[bot_id] = rospy.Publisher(cmd_topic, String, queue_size=10, latch=True)
            self.status_pubs[bot_id] = rospy.Publisher(status_topic, String, queue_size=10, latch=True)

            rospy.loginfo(f"[Master] Created topics for ESP-{bot_id}: {cmd_topic}, {status_topic}")

        # Print WebSocket server IP
        ws_ip = self.get_local_ip()
        rospy.loginfo(f"[Master] WebSocket server running on ws://{ws_ip}:8765")

        # Publish an initial 'start' command to ESP-1 to ensure it starts
        try:
            start_msg = String(data='start')
            if 1 in self.cmd_pubs:
                rospy.loginfo("[Master] Publishing initial 'start' to ESP-1")
                self.cmd_pubs[1].publish(start_msg)
        except Exception as e:
            rospy.logerr(f"[Master] Failed to publish initial start to ESP-1: {e}")

        # Services to start/stop all bots
        self.start_all_srv = rospy.Service('start_all_bots', Trigger, self.handle_start_all)
        self.stop_all_srv = rospy.Service('stop_all_bots', Trigger, self.handle_stop_all)

        # Broadcast command topic (external nodes can publish a command to this topic to be forwarded)
        self.broadcast_sub = rospy.Subscriber('/esp/broadcast_cmd', String, self.handle_broadcast)

        # Scheduler options
        # Interval-based scheduler (in seconds). Deprecated if auto_start_time is used.
        self.scheduler_timer = None
        if self.auto_start_interval and self.auto_start_interval > 0:
            rospy.loginfo(f"[Master] Enabling auto-start scheduler every {self.auto_start_interval}s")
            self.scheduler_timer = rospy.Timer(rospy.Duration(self.auto_start_interval), self._scheduler_cb)

        # Daily time-based scheduler: param format 'HH:MM' local time. Example: '06:00'
        self.auto_start_time = rospy.get_param('~auto_start_time', '')
        if self.auto_start_time:
            try:
                self._schedule_daily_start(self.auto_start_time)
                rospy.loginfo(f"[Master] Scheduled daily auto-start at {self.auto_start_time}")
            except Exception as e:
                rospy.logerr(f"[Master] Failed to schedule daily auto-start ({self.auto_start_time}): {e}")

        rospy.loginfo("[Master] Ready")

    def handle_start_all(self, req):
        msg = String(data='start')
        for bot_id, pub in self.cmd_pubs.items():
            pub.publish(msg)
            rospy.loginfo(f"[Master] start_all: published 'start' to ESP-{bot_id}")
        return TriggerResponse(success=True, message="Started all ESP bots")

    def handle_stop_all(self, req):
        msg = String(data='stop')
        for bot_id, pub in self.cmd_pubs.items():
            pub.publish(msg)
            rospy.loginfo(f"[Master] stop_all: published 'stop' to ESP-{bot_id}")
        return TriggerResponse(success=True, message="Stopped all ESP bots")

    def handle_broadcast(self, msg):
        # Forward received message to all cmd publishers
        for bot_id, pub in self.cmd_pubs.items():
            pub.publish(msg)
            rospy.loginfo(f"[Master] broadcast: forwarded '{msg.data}' to ESP-{bot_id}")

    def _scheduler_cb(self, event):
        # Simple scheduler: call start_all periodically
        rospy.loginfo("[Master] Scheduler triggering start_all")
        self.handle_start_all(None)

    def _schedule_daily_start(self, time_str):
        """Schedule a daily timer to trigger start_all at local time HH:MM.

        This schedules a one-shot timer for the next occurrence, and when fired
        it will reschedule itself for the next day.
        """
        import datetime

        parts = time_str.split(":")
        if len(parts) != 2:
            raise ValueError("auto_start_time must be in HH:MM format")
        hour = int(parts[0])
        minute = int(parts[1])

        now = datetime.datetime.now()
        target = now.replace(hour=hour, minute=minute, second=0, microsecond=0)
        if target <= now:
            # schedule for next day
            target = target + datetime.timedelta(days=1)

        delta = (target - now).total_seconds()

        rospy.loginfo(f"[Master] Daily auto-start will run in {delta} seconds (at {target})")

        # create one-shot timer. When it fires, it will call _daily_start_cb which reschedules
        rospy.Timer(rospy.Duration(delta), self._daily_start_cb, oneshot=True)

    def _daily_start_cb(self, event):
        # Trigger start_all
        rospy.loginfo("[Master] Daily auto-start triggered")
        self.handle_start_all(None)
        # Reschedule for next day
        try:
            self._schedule_daily_start(self.auto_start_time)
        except Exception as e:
            rospy.logerr(f"[Master] Failed to reschedule daily auto-start: {e}")

    def get_local_ip(self):
        """Get LAN IP of the machine running this node"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))   # dummy connect to Google DNS
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    node = ESPMasterNode()
    node.spin()
