#!/usr/bin/env python3
"""
publish_video.py
Publishes frames from a video file (or webcam) to /camera/image_raw for testing AI node in WSL.

Usage:
  rosrun ai publish_video.py _video_path:=/path/to/video.mp4
If video_path is not provided, the node will try to open device 0 (may not be available in WSL).
"""
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

def main():
    rospy.init_node('publish_video')
    video_path = rospy.get_param('~video_path', '')
    bridge = CvBridge()
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
    if video_path and os.path.exists(video_path):
        cap = cv2.VideoCapture(video_path)
    else:
        cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr('Unable to open video source: %s' % video_path)
        return

    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
        img_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        pub.publish(img_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
