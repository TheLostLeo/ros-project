#!/usr/bin/env python3
"""Scheduler that calls /trigger_cleaning at configured times."""

import rospy
import datetime
import time
from esp.srv import TriggerCleaning


class SchedulerNode:
    def __init__(self):
        rospy.init_node('scheduler_node')
        self.times = rospy.get_param('~times', ['07:00', '19:00'])
        rospy.loginfo(f"Scheduler: times={self.times}")
        self.client = rospy.ServiceProxy('/trigger_cleaning', TriggerCleaning)
        # start a background timer thread
        rospy.Timer(rospy.Duration(60), self._tick)

    def _tick(self, event):
        now = datetime.datetime.now()
        cur = now.strftime('%H:%M')
        if cur in self.times:
            rospy.loginfo(f"Scheduler: triggering cleaning at {cur}")
            try:
                res = self.client(True)
                rospy.loginfo(f"Scheduler: trigger response: success={res.success} msg={res.message}")
            except Exception as e:
                rospy.logerr(f"Scheduler: failed to call service: {e}")

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    node = SchedulerNode()
    node.spin()
