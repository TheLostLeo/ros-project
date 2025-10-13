#!/usr/bin/env python3
"""Swarm manager: provides /trigger_cleaning service and coordinates cleaning."""

import rospy
from swarm_manage.srv import TriggerCleaning, TriggerCleaningResponse
from std_msgs.msg import String


class SwarmManager:
    def __init__(self):
        rospy.init_node('swarm_manager')
        self.trigger_srv = rospy.Service('/trigger_cleaning', TriggerCleaning, self.handle_trigger)

    def handle_trigger(self, req):
        rospy.loginfo(f"SwarmManager: trigger received start={req.start}")
        if req.start:
            # For now, simply publish a message to /bot/cmd via a small publisher
            pub = rospy.Publisher('/bot/cmd', String, queue_size=10)
            rospy.sleep(0.1)
            pub.publish(String(data='start_cleaning'))
            return TriggerCleaningResponse(success=True, message='Cleaning started')
        return TriggerCleaningResponse(success=False, message='Start flag not set')

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    node = SwarmManager()
    node.spin()
