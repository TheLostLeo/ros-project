#!/usr/bin/env python3
"""
AI detector node

Features:
- subscribes to `/camera/image_raw` (if available) or opens local camera device
- uses OpenCV background subtraction and simple blob-tracking to detect fish-like objects
- computes simple metrics (count, average speed, dispersion) and classifies state as
  'happy', 'stressed', or 'unknown' using heuristic rules
- publishes JSON-like status strings on `/ai/alert`
"""

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from collections import deque
import time
import json


class SimpleTracker:
    """Very small centroid tracker (not for production)."""
    def __init__(self, max_history=8, max_distance=50):
        self.tracks = []  # list of deques of (x,y)
        self.max_history = max_history
        self.max_distance = max_distance

    def update(self, centroids):
        assigned = [False]*len(centroids)
        # try to match existing tracks to centroids
        for t in self.tracks:
            last = t[-1]
            best_i = -1
            best_d = None
            for i,c in enumerate(centroids):
                if assigned[i]:
                    continue
                d = np.hypot(c[0]-last[0], c[1]-last[1])
                if best_d is None or d < best_d:
                    best_d = d
                    best_i = i
            if best_i != -1 and best_d <= self.max_distance:
                t.append(centroids[best_i])
                if len(t) > self.max_history:
                    t.popleft()
                assigned[best_i] = True
            else:
                # no match -> shorten track history (aging)
                if len(t) > 1:
                    t.append(t[-1])
                    if len(t) > self.max_history:
                        t.popleft()

        # create new tracks for unassigned centroids
        for i,c in enumerate(centroids):
            if not assigned[i]:
                dq = deque(maxlen=self.max_history)
                dq.append(c)
                self.tracks.append(dq)

        # prune empty/very old tracks
        self.tracks = [t for t in self.tracks if len(t) > 0]

    def get_speeds(self):
        speeds = []
        for t in self.tracks:
            if len(t) >= 2:
                dx = t[-1][0] - t[-2][0]
                dy = t[-1][1] - t[-2][1]
                speeds.append(np.hypot(dx, dy))
        return speeds


class AIDetectorNode:
    def __init__(self):
        rospy.init_node('ai_detector', anonymous=True)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/ai/alert', String, queue_size=2)

        # Parameters
        self.use_camera = rospy.get_param('~use_camera_device', True)
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.show_window = rospy.get_param('~show_window', False)
        self.min_area = rospy.get_param('~min_area', 200)
        self.speed_thresh_low = rospy.get_param('~speed_thresh_low', 2.0)
        self.speed_thresh_high = rospy.get_param('~speed_thresh_high', 8.0)

        # Background subtractor
        self.backsub = cv2.createBackgroundSubtractorMOG2(history=300, varThreshold=16, detectShadows=False)
        self.tracker = SimpleTracker(max_history=8, max_distance=60)

        self.last_process_time = time.time()

        # Try subscribing to ROS image topic; if not available or if use_camera is True we'll open device
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_cb, queue_size=1)
        self.cap = None
        if self.use_camera:
            try:
                self.cap = cv2.VideoCapture(self.camera_id)
                if not self.cap.isOpened():
                    rospy.logwarn('Unable to open camera device %s' % str(self.camera_id))
                    self.cap = None
                else:
                    rospy.loginfo('Opened camera device %s' % str(self.camera_id))
            except Exception as e:
                rospy.logwarn('Camera open failed: %s' % e)

        rospy.loginfo('AI Detector initialized (use_camera=%s)' % str(self.cap is not None))

    def image_cb(self, msg):
        # prefer ROS images only if camera device not available
        if self.cap is not None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr('cv_bridge error: %s' % e)
            return
        self.process_frame(frame)

    def process_frame(self, frame):
        # Resize for speed
        frame_small = cv2.resize(frame, (640, 360))
        fg = self.backsub.apply(frame_small)
        _, th = cv2.threshold(fg, 200, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel, iterations=1)
        contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        centroids = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            centroids.append((cx, cy, area))
            cv2.circle(frame_small, (cx, cy), 4, (0,255,0), -1)
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(frame_small, (x,y), (x+w,y+h), (0,255,0), 1)

        # Update tracker with 2D points only
        points = [(c[0], c[1]) for c in centroids]
        self.tracker.update(points)
        speeds = self.tracker.get_speeds()
        avg_speed = float(np.mean(speeds)) if len(speeds)>0 else 0.0

        # Dispersion: average pairwise distance between centroids
        dispersion = 0.0
        if len(points) >= 2:
            dists = []
            for i in range(len(points)):
                for j in range(i+1, len(points)):
                    dists.append(np.hypot(points[i][0]-points[j][0], points[i][1]-points[j][1]))
            dispersion = float(np.mean(dists)) if len(dists)>0 else 0.0

        # Simple heuristic classification
        label = 'unknown'
        num_fish = len(points)
        if num_fish == 0:
            label = 'no_fish'
        else:
            if avg_speed < self.speed_thresh_low and dispersion < 80:
                label = 'stressed'
            elif avg_speed > self.speed_thresh_high and dispersion < 50:
                label = 'panic'
            elif avg_speed >= self.speed_thresh_low and avg_speed <= self.speed_thresh_high and dispersion > 60:
                label = 'happy'
            else:
                label = 'unknown'

        # Publish a JSON-like alert
        msg = {
            'state': label,
            'num_fish': int(num_fish),
            'avg_speed': float(round(avg_speed,2)),
            'dispersion': float(round(dispersion,2)),
            'timestamp': time.time()
        }
        self.pub.publish(json.dumps(msg))

        if self.show_window:
            cv2.putText(frame_small, '%s | n=%d | v=%.2f d=%.1f' % (label, num_fish, avg_speed, dispersion), (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255),2)
            cv2.imshow('ai_detector', frame_small)
            cv2.waitKey(1)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.cap is not None:
                ret, frame = self.cap.read()
                if ret:
                    self.process_frame(frame)
            rate.sleep()


def main():
    node = AIDetectorNode()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
