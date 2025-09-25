#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def cmd_cb(msg):
    rospy.loginfo('swarm_bot received cmd: %s' % msg.data)

def main():
    rospy.init_node('swarm_bot')
    rospy.Subscriber('/swarm_cmd', String, cmd_cb)
    status_pub = rospy.Publisher('/esp/status', String, queue_size=10)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        status_pub.publish('bot_alive')
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
