#!/usr/bin/env python3
"""Placeholder AI fish monitor that listens to camera topic and logs receipt."""

import rospy
from sensor_msgs.msg import Image


def image_cb(msg):
    rospy.loginfo("AI Monitor: received an image")


def main():
    rospy.init_node('ai_fish_monitor')
    rospy.Subscriber('/camera/image_raw', Image, image_cb)
    rospy.spin()


if __name__ == '__main__':
    main()
