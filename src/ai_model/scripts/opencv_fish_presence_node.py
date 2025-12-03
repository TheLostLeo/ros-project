#!/usr/bin/env python3
"""
OpenCV Fish Presence Node
-------------------------
Detects whether a fish is present in incoming images using classical CV heuristics
and publishes a presence result as JSON on a std_msgs/String topic.

Methods (select via ~method):
- shape: Edge+contour heuristics (area, aspect ratio, solidity)
- hsv:   HSV color segmentation with area ratio threshold
- motion:Simple background subtraction (for video streams)

Params:
- ~camera_topic (str): input images (default: /camera/image_raw)
- ~publish_topic (str): output topic (default: /ai/fish_presence)
- ~method (str): one of ['shape','hsv','motion'] (default: 'shape')
- ~resize_w (int), ~resize_h (int): processing size (default: 320x240)
- ~shape_min_area (int): min contour area at processing scale (default: 800)
- ~shape_min_ar (float): min aspect ratio (min(width/height, height/width)) (default: 0.2)
- ~shape_min_solidity (float): min solidity (area/convex_area) (default: 0.5)
- ~hsv_low (int,int,int): lower HSV (default: 0,30,30)
- ~hsv_high (int,int,int): upper HSV (default: 180,255,255)
- ~hsv_min_ratio (float): min mask area ratio (default: 0.015)
- ~motion_ratio_thresh (float): moving px ratio to consider presence (default: 0.01)
- ~fps_limit (float): if >0, sleep to cap rate (default: 0)

Publishes JSON like:
  { "label": "fish"|"no_fish", "conf": 0..1, "method": "shape|hsv|motion", "metrics": {...}, "timestamp": float }
"""
import json
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class OpenCVFishPresence:
    def __init__(self):
        rospy.init_node('opencv_fish_presence')
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/image_raw')
        self.publish_topic = rospy.get_param('~publish_topic', '/ai/fish_presence')
        self.method = str(rospy.get_param('~method', 'shape')).strip().lower()
        self.resize_w = int(rospy.get_param('~resize_w', 320))
        self.resize_h = int(rospy.get_param('~resize_h', 240))
        # Shape params
        self.shape_min_area = int(rospy.get_param('~shape_min_area', 800))
        self.shape_min_ar = float(rospy.get_param('~shape_min_ar', 0.2))
        self.shape_min_solidity = float(rospy.get_param('~shape_min_solidity', 0.5))
        # HSV params
        self.hsv_low = tuple(int(x) for x in rospy.get_param('~hsv_low', [0, 30, 30]))
        self.hsv_high = tuple(int(x) for x in rospy.get_param('~hsv_high', [180, 255, 255]))
        self.hsv_min_ratio = float(rospy.get_param('~hsv_min_ratio', 0.015))
        # Motion params
        self.motion_ratio_thresh = float(rospy.get_param('~motion_ratio_thresh', 0.01))
        self.fps_limit = float(rospy.get_param('~fps_limit', 0.0))

        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.publish_topic, String, queue_size=10)
        rospy.Subscriber(self.camera_topic, Image, self.image_cb, queue_size=1, buff_size=2**22)

        self.bg_sub = cv2.createBackgroundSubtractorMOG2(history=200, varThreshold=25, detectShadows=False)

    def _preproc(self, bgr):
        if self.resize_w > 0 and self.resize_h > 0:
            bgr = cv2.resize(bgr, (self.resize_w, self.resize_h), interpolation=cv2.INTER_AREA)
        return bgr

    def _detect_shape(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 40, 120)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations=2)
        cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_score = 0.0
        best_area = 0
        for c in cnts:
            area = cv2.contourArea(c)
            if area < self.shape_min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            ar = min(float(w)/max(1,h), float(h)/max(1,w))
            if ar < self.shape_min_ar:
                continue
            hull = cv2.convexHull(c)
            hull_area = max(1.0, cv2.contourArea(hull))
            solidity = float(area) / hull_area
            if solidity < self.shape_min_solidity:
                continue
            score = (area / float(self.resize_w*self.resize_h)) * ar * solidity
            if score > best_score:
                best_score = score
                best_area = int(area)
        present = best_score > 0.002  # heuristic threshold
        # Confidence scaled from score (cap to 1)
        conf = float(np.clip(best_score * 50.0, 0.0, 1.0))
        return present, conf, { 'best_area': best_area, 'score': round(best_score,6) }

    def _detect_hsv(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        low = np.array(self.hsv_low, dtype=np.uint8)
        high = np.array(self.hsv_high, dtype=np.uint8)
        mask = cv2.inRange(hsv, low, high)
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=1)
        ratio = float(np.count_nonzero(mask)) / float(mask.size)
        present = ratio >= self.hsv_min_ratio
        # Confidence grows with ratio
        conf = float(np.clip((ratio - self.hsv_min_ratio) / max(1e-6, self.hsv_min_ratio*3), 0.0, 1.0))
        return present, conf, { 'hsv_ratio': round(ratio,6) }

    def _detect_motion(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        fg = self.bg_sub.apply(gray)
        fg = cv2.threshold(fg, 127, 255, cv2.THRESH_BINARY)[1]
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=1)
        ratio = float(np.count_nonzero(fg)) / float(fg.size)
        present = ratio >= self.motion_ratio_thresh
        conf = float(np.clip((ratio - self.motion_ratio_thresh) / max(1e-6, self.motion_ratio_thresh*3), 0.0, 1.0))
        return present, conf, { 'motion_ratio': round(ratio,6) }

    def image_cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"cv_bridge failed: {e}")
            return
        bgr = self._preproc(bgr)
        if self.method == 'hsv':
            present, conf, metrics = self._detect_hsv(bgr)
            method = 'hsv'
        elif self.method == 'motion':
            present, conf, metrics = self._detect_motion(bgr)
            method = 'motion'
        else:
            present, conf, metrics = self._detect_shape(bgr)
            method = 'shape'
        out = {
            'timestamp': time.time(),
            'label': 'fish' if present else 'no_fish',
            'conf': round(conf, 3),
            'method': method,
            'metrics': metrics
        }
        self.pub.publish(String(data=json.dumps(out)))


def main():
    node = OpenCVFishPresence()
    rospy.spin()


if __name__ == '__main__':
    main()
