#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

rospy.init_node('fake_robot_publisher')
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
image_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
bridge = CvBridge()
rate = rospy.Rate(10)

x = -0.0

def create_image():
    img = np.zeros((240,320,3), dtype=np.uint8)
    cv2.putText(img, 'FakeCamera', (50,120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    return img

while not rospy.is_shutdown():
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = -8.0
    odom.pose.pose.position.z = 0
    odom_pub.publish(odom)

    img = create_image()
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
    except Exception as e:
        rospy.logerr(e)

    x += 0.01
    rate.sleep()
