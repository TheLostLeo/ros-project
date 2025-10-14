#!/usr/bin/env python3
"""
Swarm Manager Node
------------------
This node acts as a high-level controller for the swarm.
It listens for external commands (e.g., from a UI or a scheduler) and
translates them into actions for the swarm, such as starting or stopping a cleaning task.
"""

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest

class SwarmManagerNode:
    def __init__(self):
        rospy.init_node('swarm_manager_node')

        # The topic to listen on for commands.
        self.command_topic = rospy.get_param('~command_topic', '/system/command')

        # Subscriber for the external command.
        rospy.Subscriber(self.command_topic, String, self.command_cb)

        # Service clients to trigger the decision node.
        self.trigger_cleaning_service = None
        self.trigger_stop_service = None
        
        rospy.loginfo("Swarm Manager is running.")
        
        try:
            rospy.loginfo("Waiting for '/start_cleaning' service...")
            rospy.wait_for_service('/start_cleaning', timeout=30.0)
            self.trigger_cleaning_service = rospy.ServiceProxy('/start_cleaning', Trigger)
            rospy.loginfo("Successfully connected to '/start_cleaning' service.")

            rospy.loginfo("Waiting for '/stop_cleaning' service...")
            rospy.wait_for_service('/stop_cleaning', timeout=5.0)
            self.trigger_stop_service = rospy.ServiceProxy('/stop_cleaning', Trigger)
            rospy.loginfo("Successfully connected to '/stop_cleaning' service.")

            rospy.loginfo(f"Ready to receive commands on topic: {self.command_topic}")
        except rospy.ROSException:
            rospy.logerr("A required service did not become available. Is the decision node running?")
            rospy.signal_shutdown("Required service not available")

    def command_cb(self, msg: String):
        """Callback for the command topic."""
        command = msg.data.lower().strip()
        
        if command == 'start':
            rospy.loginfo("'start' command received. Calling the cleaning service...")
            if not self.trigger_cleaning_service:
                rospy.logerr("Start service is not available.")
                return
            try:
                response = self.trigger_cleaning_service(TriggerRequest())
                if response.success:
                    rospy.loginfo(f"Cleaning service started successfully: {response.message}")
                else:
                    rospy.logwarn(f"Cleaning service failed to start: {response.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

        elif command == 'stop':
            rospy.loginfo("'stop' command received. Calling the stop service...")
            if not self.trigger_stop_service:
                rospy.logerr("Stop service is not available.")
                return
            try:
                response = self.trigger_stop_service(TriggerRequest())
                if response.success:
                    rospy.loginfo(f"Stop service called successfully: {response.message}")
                else:
                    rospy.logwarn(f"Stop service failed: {response.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
        else:
            rospy.logwarn(f"Received unknown command: '{command}'. Ignoring.")

    def spin(self):
        """Keeps the node running."""
        rospy.spin()

if __name__ == '__main__':
    try:
        manager = SwarmManagerNode()
        manager.spin()
    except rospy.ROSInterruptException:
        pass
