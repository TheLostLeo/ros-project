#!/usr/bin/env python3
"""
scheduler_node.py
------------------
Simple scheduler that can:
- Trigger a start command every day at a configured local time (HH:MM or HH:MM:SS)
- Trigger a one-shot start after a countdown in seconds

Topics (subscribe):
- /scheduler/set_daily_alarm (std_msgs/String): payload like "01:00" to enable; "off" to disable
- /scheduler/set_timer (std_msgs/Int32): payload seconds > 0 to start a countdown

Param:
- ~command_topic (str): where to publish start command (default: /system/command)
"""
import time
from datetime import datetime, timedelta

import rospy
from std_msgs.msg import String, Int32


def parse_time_str(s: str):
    s = (s or '').strip().lower()
    if not s:
        return None
    if s in ('off', 'none', 'disable', 'disabled', 'no', '0'):
        return 'off'
    # Accept "H:MM", "HH:MM", "H.MM", "HH.MM", and with optional seconds HH:MM:SS
    s = s.replace('.', ':')
    parts = s.split(':')
    try:
        if len(parts) == 2:
            h, m = int(parts[0]), int(parts[1])
            return (h, m, 0)
        elif len(parts) == 3:
            h, m, sec = int(parts[0]), int(parts[1]), int(parts[2])
            return (h, m, sec)
    except Exception:
        return None
    return None


class SchedulerNode:
    def __init__(self):
        rospy.init_node('scheduler_node')
        self.command_topic = rospy.get_param('~command_topic', '/system/command')
        self.pub_cmd = rospy.Publisher(self.command_topic, String, queue_size=10)

        # State
        self.alarm_enabled = False
        self.alarm_hms = (1, 0, 0)  # default 01:00
        self.alarm_next_dt = None  # datetime of next fire
        self.timer_deadline = None  # monotonic seconds

        rospy.Subscriber('/scheduler/set_daily_alarm', String, self._alarm_cb)
        rospy.Subscriber('/scheduler/set_timer', Int32, self._timer_cb)

        # Tick every 0.5s
        self.timer = rospy.Timer(rospy.Duration(0.5), self._tick)
        rospy.loginfo('Scheduler node started. Topics: /scheduler/set_daily_alarm (String), /scheduler/set_timer (Int32)')

    def _compute_next_alarm(self):
        h, m, s = self.alarm_hms
        now = datetime.now()
        candidate = now.replace(hour=h % 24, minute=m % 60, second=s % 60, microsecond=0)
        if candidate <= now:
            candidate = candidate + timedelta(days=1)
        self.alarm_next_dt = candidate

    def _alarm_cb(self, msg: String):
        val = (msg.data or '').strip()
        parsed = parse_time_str(val)
        if parsed == 'off':
            self.alarm_enabled = False
            self.alarm_next_dt = None
            rospy.loginfo('Daily alarm disabled')
            return
        if isinstance(parsed, tuple):
            self.alarm_hms = parsed
            self.alarm_enabled = True
            self._compute_next_alarm()
            rospy.loginfo(f"Daily alarm set to {self.alarm_hms} next at {self.alarm_next_dt}")
        else:
            rospy.logwarn(f"Invalid alarm time string: '{val}' (expected HH:MM or HH:MM:SS)")

    def _timer_cb(self, msg: Int32):
        secs = int(msg.data)
        if secs > 0:
            self.timer_deadline = time.monotonic() + secs
            rospy.loginfo(f"Countdown timer set: {secs}s")
        else:
            self.timer_deadline = None
            rospy.loginfo("Countdown timer cleared")

    def _publish_start(self, reason: str):
        self.pub_cmd.publish(String(data='start'))
        rospy.loginfo(f"Scheduler published 'start' ({reason})")

    def _tick(self, _evt):
        # Check daily alarm
        if self.alarm_enabled and self.alarm_next_dt is not None:
            now = datetime.now()
            if now >= self.alarm_next_dt:
                self._publish_start('daily alarm')
                # schedule next day
                self._compute_next_alarm()

        # Check countdown timer
        if self.timer_deadline is not None:
            now_m = time.monotonic()
            if now_m >= self.timer_deadline:
                self._publish_start('countdown timer')
                self.timer_deadline = None

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = SchedulerNode()
    node.run()
