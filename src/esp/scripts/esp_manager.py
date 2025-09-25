#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('esp_manager')
    status_pub = rospy.Publisher('/esp/status', String, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        status_pub.publish('ok')
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
