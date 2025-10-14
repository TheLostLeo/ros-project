#!/usr/bin/env python3
"""
A state-driven decision node for managing an aquarium cleaning task.
It can be triggered by a service and uses feedback from the bot to execute a
zig-zag path.
"""

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from swarm_manage.msg import BotCommand, BotStatus

class ZigZag:
    """Generates a zig-zag (or raster scan) path."""
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
#!/usr/bin/env python3
"""
Decision node that drives the ESP bot with simple commands
(forward/backward/left/right/stop) while following a zig-zag (boustrophedon)
coverage pattern over a rows x cols grid.
"""

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
from swarm_manage.msg import BotStatus, BotCommand


class ZigZag:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols

    def generate_path(self):
        path = []
        for r in range(self.rows):
            row = list(range(r * self.cols, (r + 1) * self.cols))
            if r % 2 == 1:
                row.reverse()
            path.extend(row)
        return path


class ESPDecisionNode:
    def __init__(self):
        rospy.init_node('esp_decision_node')

        # Params
        self.grid_rows = rospy.get_param('~grid_rows', 5)
        self.grid_cols = rospy.get_param('~grid_cols', 5)

        # State
        self.state = 'IDLE'  # IDLE | CLEANING | FINISHED
        self.path = []
        self.i = -1  # waypoint index
        self.last_status = None

        # Planner
        self.planner = ZigZag(self.grid_rows, self.grid_cols)

        # ROS I/O
        self.simple_pub = rospy.Publisher('/bot/simple_cmd', String, queue_size=10)
        self.cmd_pub = rospy.Publisher('/bot/cmd', BotCommand, queue_size=10)
        rospy.Subscriber('/bot/status', BotStatus, self.status_cb)
        rospy.Service('/start_cleaning', Trigger, self.handle_start)
        rospy.Service('/stop_cleaning', Trigger, self.handle_stop)

        rospy.loginfo('Decision node ready (zig-zag).')

    # Services
    def handle_start(self, _req):
        if self.state not in ('IDLE', 'FINISHED'):
            return TriggerResponse(success=False, message=f'already {self.state}')
        self.path = self.planner.generate_path()
        self.i = 0
        self.state = 'CLEANING'
        rospy.loginfo(f'Starting cleaning with {len(self.path)} cells')
        self.issue_next_step(prev_zone=None, next_zone=self.path[self.i])
        return TriggerResponse(success=True, message='started')

    def handle_stop(self, _req):
        if self.state != 'CLEANING':
            return TriggerResponse(success=False, message=f'not cleaning ({self.state})')
        self.state = 'IDLE'
        self.path = []
        self.i = -1
        # Tell ESP to stop motors and publish structured stop
        self.simple_pub.publish(String(data='stop'))
        stop = BotCommand(); stop.action = 'stop'; stop.speed = 0
        self.cmd_pub.publish(stop)
        rospy.loginfo('Cleaning stopped -> IDLE')
        return TriggerResponse(success=True, message='stopped')

    # Feedback
    def status_cb(self, msg: BotStatus):
        self.last_status = msg
        if self.state != 'CLEANING':
            return
        # We advance when the bot reports it is not cleaning (idle) anymore
        if not msg.cleaning:
            self.i += 1
            if self.i >= len(self.path):
                self.state = 'FINISHED'
                rospy.loginfo('Cleaning complete')
                self.simple_pub.publish(String(data='stop'))
                return
            prev_zone = self.path[self.i - 1] if self.i - 1 >= 0 else None
            next_zone = self.path[self.i]
            self.issue_next_step(prev_zone, next_zone)

    # Movement logic (boustrophedon with simple commands)
    def zone_to_rc(self, zone_id):
        return zone_id // self.grid_cols, zone_id % self.grid_cols

    def issue_next_step(self, prev_zone, next_zone):
        # Publish a simple command sequence to reach next cell
        if prev_zone is None:
            # First move of the plan: just go forward into the first cell
            self.simple_pub.publish(String(data='forward'))
        else:
            pr, pc = self.zone_to_rc(prev_zone)
            nr, nc = self.zone_to_rc(next_zone)
            if pr == nr:
                # Same row -> advance one cell
                self.simple_pub.publish(String(data='forward'))
            else:
                # Row changed -> at row end; perform turn-down maneuver then advance
                if pr % 2 == 0:
                    # Was moving east: right, forward (step down), right
                    self.simple_pub.publish(String(data='right'))
                    rospy.sleep(0.2)
                    self.simple_pub.publish(String(data='forward'))
                    rospy.sleep(0.2)
                    self.simple_pub.publish(String(data='right'))
                else:
                    # Was moving west: left, forward (step down), left
                    self.simple_pub.publish(String(data='left'))
                    rospy.sleep(0.2)
                    self.simple_pub.publish(String(data='forward'))
                    rospy.sleep(0.2)
                    self.simple_pub.publish(String(data='left'))
                rospy.sleep(0.2)
                self.simple_pub.publish(String(data='forward'))

        # Also publish a structured command for compatibility (optional)
        bc = BotCommand()
        bc.zone_id = next_zone
        bc.action = 'clean_zone'
        bc.speed = 0.5
        self.cmd_pub.publish(bc)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ESPDecisionNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
