#!/usr/bin/env python3
"""Decision node that maps aquarium and computes zig-zag coverage paths.
"""

import rospy
from std_msgs.msg import String
from esp.msg import BotCommand, BotStatus
import time


class ESPDecisionNode:
    def __init__(self):
        rospy.init_node('esp_decision_node')

        self.bot_id = rospy.get_param('~bot_id', 1)
        self.grid_rows = rospy.get_param('~grid_rows', 5)
        self.grid_cols = rospy.get_param('~grid_cols', 5)
        self.mapping_duration = rospy.get_param('~mapping_duration', 20.0)

        self.cmd_pub = rospy.Publisher('/bot/cmd', BotCommand, queue_size=10)
        rospy.Subscriber('/bot/status', BotStatus, self.status_cb)

        self.map = [[0 for _ in range(self.grid_cols)] for _ in range(self.grid_rows)]
        self.mapped = False

        rospy.loginfo("Decision: starting mapping phase")
        rospy.Timer(rospy.Duration(self.mapping_duration), self._end_mapping, oneshot=True)

    def status_cb(self, msg: BotStatus):
        # Optionally use status to build map; here we just note presence
        rospy.loginfo(f"Decision: received status from bot {msg.bot_id}")

    def _end_mapping(self, event):
        rospy.loginfo("Decision: mapping complete; computing zig-zag path")
        self.mapped = True
        path = self.compute_zigzag()
        rospy.loginfo(f"Decision: computed path with {len(path)} waypoints")
        # Publish commands for demonstration
        for zone_id in path:
            cmd = BotCommand()
            cmd.zone_id = zone_id
            cmd.action = 'clean'
            cmd.speed = 0.5
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.5)

    def compute_zigzag(self):
        path = []
        for r in range(self.grid_rows):
            row = list(range(r * self.grid_cols, (r + 1) * self.grid_cols))
            if r % 2 == 1:
                row.reverse()
            path.extend(row)
        return path

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    node = ESPDecisionNode()
    node.spin()
