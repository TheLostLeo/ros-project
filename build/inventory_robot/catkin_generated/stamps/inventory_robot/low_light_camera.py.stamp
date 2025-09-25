#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class LowLightCameraSimulator:
    def __init__(self):
        rospy.init_node('low_light_camera_simulator', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.light_level = rospy.get_param('~light_level', 0.3)
        self.noise_level = rospy.get_param('~noise_level', 0.1)
        self.blur_level = rospy.get_param('~blur_level', 1.0)
        
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/inventory_robot/low_light_camera', Image, queue_size=10)
        
        rospy.loginfo('Low Light Camera Simulator Initialized')
    
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            low_light = self.simulate_low_light(cv_image)
            low_light_msg = self.bridge.cv2_to_imgmsg(low_light, 'bgr8')
            low_light_msg.header = data.header
            self.image_pub.publish(low_light_msg)
        except CvBridgeError as e:
            rospy.logerr('Error processing image: %s' % e)
    
    def simulate_low_light(self, image):
        low_light = cv2.convertScaleAbs(image, alpha=self.light_level, beta=0)
        noise_amount = self.noise_level * (1.0 - self.light_level) * 50
        noise = np.zeros(low_light.shape, np.uint8)
        cv2.randn(noise, 0, noise_amount)
        noisy_image = cv2.add(low_light, noise)
        kernel_size = max(1, int(self.blur_level * 3))
        if kernel_size % 2 == 0:
            kernel_size += 1
        blurred_image = cv2.GaussianBlur(noisy_image, (kernel_size, kernel_size), 0)
        contrast_reduction = 0.7 + (0.3 * self.light_level)
        mean_value = np.mean(blurred_image)
        reduced_contrast = cv2.addWeighted(
            blurred_image, contrast_reduction,
            np.full_like(blurred_image, mean_value), 1 - contrast_reduction,
            0
        )
        rows, cols = reduced_contrast.shape[:2]
        x = np.linspace(-1, 1, cols)
        y = np.linspace(-1, 1, rows)
        x_grid, y_grid = np.meshgrid(x, y)
        radius = np.sqrt(x_grid**2 + y_grid**2)
        vignette = np.clip(1.0 - radius * (1.0 - self.light_level), 0, 1)
        vignette = vignette[:, :, np.newaxis]
        vignetted_image = (reduced_contrast * vignette).astype(np.uint8)
        return vignetted_image

if __name__ == '__main__':
    try:
        simulator = LowLightCameraSimulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
