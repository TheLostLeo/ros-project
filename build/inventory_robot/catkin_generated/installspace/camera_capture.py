#!/usr/bin/env python3
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
import datetime

class CameraCapture:
    def __init__(self):
        rospy.init_node('camera_capture_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_dir = rospy.get_param('~image_dir', '/tmp/inventory_images')
        self.low_light_mode = rospy.get_param('~low_light_mode', False)
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        self.image_sub = rospy.Subscriber('/inventory_robot/low_light_camera' if self.low_light_mode else '/camera/rgb/image_raw', Image, self.image_callback)
        self.capture_sub = rospy.Subscriber('/inventory_robot/capture_image', Bool, self.capture_callback)
        self.image_pub = rospy.Publisher('/inventory_robot/processed_image', Image, queue_size=10)
        self.latest_cv_image = None
        rospy.loginfo('Camera Capture Node Initialized')
    
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.latest_cv_image = cv_image
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except CvBridgeError as e:
            rospy.logerr('Error processing image: %s' % e)
    
    def capture_callback(self, msg):
        if not msg.data:
            return
        if self.latest_cv_image is None:
            rospy.logwarn('No image available to capture')
            return
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.image_dir, f'soil_zone_{timestamp}.jpg')
        try:
            cv2.imwrite(filename, self.latest_cv_image)
            rospy.loginfo('Image captured and saved to: %s' % filename)
        except Exception as e:
            rospy.logerr('Error saving image: %s' % e)

if __name__ == '__main__':
    try:
        node = CameraCapture()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
