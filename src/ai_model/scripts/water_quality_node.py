#!/usr/bin/env python3
"""
water_quality_node.py
---------------------
Estimates water quality from a camera feed using OpenCV heuristics.
Outputs a JSON-ish String on /ai/water_quality with:
- sharpness (variance of Laplacian)
- saturation_mean (HSV S channel mean)
- color_cast (greenish/bluish/neutral)
- turbidity_score (0..1, higher is clearer water)
- quality (good/moderate/poor)

Params:
- ~camera_topic (default: /camera/image_raw)
- ~publish_topic (default: /ai/water_quality)
"""
import json
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class WaterQualityNode:
    def __init__(self):
        rospy.init_node('water_quality_node')
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/image_raw')
        self.publish_topic = rospy.get_param('~publish_topic', '/ai/water_quality')

        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.publish_topic, String, queue_size=10)
        rospy.Subscriber(self.camera_topic, Image, self.image_cb, queue_size=1, buff_size=2**22)

    @staticmethod
    def variance_of_laplacian(gray):
        return float(cv2.Laplacian(gray, cv2.CV_64F).var())

    @staticmethod
    def color_cast_bgr(bgr):
        # Compare channel means to identify dominant cast
        b_mean = float(np.mean(bgr[:, :, 0]))
        g_mean = float(np.mean(bgr[:, :, 1]))
        r_mean = float(np.mean(bgr[:, :, 2]))
        if g_mean > r_mean + 10 and g_mean > b_mean + 10:
            return 'greenish'
        if b_mean > r_mean + 10 and b_mean > g_mean + 10:
            return 'bluish'
        return 'neutral'

    def score_quality(self, sharpness, sat_mean):
        # Normalize heuristics to a turbidity score 0..1
        # Sharpness ~ 0..1000+ (depends on camera); clamp to [0, 800]
        sharp_norm = max(0.0, min(1.0, sharpness / 800.0))
        # Lower saturation may imply haziness; invert and clamp
        sat_norm = 1.0 - max(0.0, min(1.0, sat_mean / 200.0))
        # Combine (weight sharpness more)
        turbidity_score = 0.7 * sharp_norm + 0.3 * sat_norm
        if turbidity_score > 0.66:
            quality = 'good'
        elif turbidity_score > 0.33:
            quality = 'moderate'
        else:
            quality = 'poor'
        return float(turbidity_score), quality

    def image_cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn(f"cv_bridge failed: {e}")
            return
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        sharpness = self.variance_of_laplacian(gray)
        sat_mean = float(np.mean(hsv[:, :, 1]))
        color_cast = self.color_cast_bgr(bgr)
        turbidity_score, quality = self.score_quality(sharpness, sat_mean)
        out = {
            'sharpness': round(sharpness, 2),
            'saturation_mean': round(sat_mean, 2),
            'color_cast': color_cast,
            'turbidity_score': round(turbidity_score, 3),
            'quality': quality,
        }
        self.pub.publish(String(data=json.dumps(out)))


def main():
    WaterQualityNode()
    rospy.spin()


if __name__ == '__main__':
    main()
