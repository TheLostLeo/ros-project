#!/usr/bin/env python3
"""
opencv_behavior_node.py
-----------------------
Reads a video stream (camera or file) using OpenCV and infers fish behavior as
"calm" or "disturbed" based on motion intensity (frame differencing).
Optionally, it first detects fish presence using OpenCV (HSV mask) and will
only print/publish a message if fish are detected.

Publishes on /ai/fish_behavior (std_msgs/String JSON):
  { "label": "calm"|"disturbed", "conf": 0..1, "motion_ratio": 0..1, "source": "opencv-mhi", "timestamp": float }

Params:
- ~source (str): camera index (e.g., "0") or file path. Default: "0" (camera 0)
- ~width (int): optional resize width. Default: 320
- ~height (int): optional resize height. Default: 240
- ~pixel_diff_thresh (int): diff threshold per-pixel (0..255). Default: 25
- ~motion_ratio_thresh (float): threshold of moving pixel ratio to call disturbed. Default: 0.015
- ~ema_window_sec (float): smoothing window seconds for EMA of motion ratio. Default: 2.0
- ~behavior_topic (str): publish topic. Default: /ai/fish_behavior
- ~fps_limit (float): if > 0, sleep to cap processing rate. Default: 0 (no cap)
 - ~presence_enable (bool): enable fish presence gating (default: true)
 - ~presence_hsv_low (list[int,int,int]): lower HSV for fish mask (default: [0,30,30])
 - ~presence_hsv_high (list[int,int,int]): upper HSV for fish mask (default: [180,255,255])
 - ~presence_min_ratio (float): minimum mask area ratio to consider fish present (default: 0.01)
"""
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from std_msgs.msg import String


def open_capture(source_str: str):
    # If numeric string, use as camera index; otherwise treat as path
    cap = None
    if source_str.isdigit():
        cap = cv2.VideoCapture(int(source_str))
    else:
        # Expand ~ and make absolute
        path = str(Path(source_str).expanduser())
        cap = cv2.VideoCapture(path)
    return cap


class OpenCVBehaviorNode:
    def __init__(self):
        rospy.init_node('opencv_behavior_node')
        self.source = str(rospy.get_param('~source', '0'))
        self.width = int(rospy.get_param('~width', 320))
        self.height = int(rospy.get_param('~height', 240))
        self.pixel_diff_thresh = int(rospy.get_param('~pixel_diff_thresh', 25))
        self.motion_ratio_thresh = float(rospy.get_param('~motion_ratio_thresh', 0.015))
        self.ema_window_sec = float(rospy.get_param('~ema_window_sec', 2.0))
        self.behavior_topic = rospy.get_param('~behavior_topic', '/ai/fish_behavior')
        self.fps_limit = float(rospy.get_param('~fps_limit', 0.0))
        # Presence gating (HSV mask)
        self.presence_enable = bool(rospy.get_param('~presence_enable', True))
        low = rospy.get_param('~presence_hsv_low', [0, 30, 30])
        high = rospy.get_param('~presence_hsv_high', [180, 255, 255])
        self.presence_hsv_low = np.array([int(low[0]), int(low[1]), int(low[2])], dtype=np.uint8)
        self.presence_hsv_high = np.array([int(high[0]), int(high[1]), int(high[2])], dtype=np.uint8)
        self.presence_min_ratio = float(rospy.get_param('~presence_min_ratio', 0.01))

        self.pub = rospy.Publisher(self.behavior_topic, String, queue_size=10)

        self.cap = None
        self.prev_gray = None
        self.ema_motion = 0.0
        self.last_time = time.time()

    def _ensure_open(self):
        if self.cap is None or not self.cap.isOpened():
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass
            self.cap = open_capture(self.source)
            if not self.cap or not self.cap.isOpened():
                rospy.logwarn_throttle(5.0, f"OpenCVBehavior: cannot open source '{self.source}', retrying...")
                return False
            # Try to hint capture size for webcams
            try:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            except Exception:
                pass
        return True

    def _read_frame(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        if self.width > 0 and self.height > 0:
            frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)
        return frame

    def _compute_motion_ratio(self, gray):
        if self.prev_gray is None:
            self.prev_gray = gray
            return 0.0
        diff = cv2.absdiff(gray, self.prev_gray)
        _, mask = cv2.threshold(diff, self.pixel_diff_thresh, 255, cv2.THRESH_BINARY)
        moving = float(np.count_nonzero(mask))
        total = float(mask.size)
        ratio = moving / max(1.0, total)
        self.prev_gray = gray
        return ratio

    def _update_ema(self, value, dt):
        # EMA alpha from window seconds
        if self.ema_window_sec <= 0:
            alpha = 1.0
        else:
            # alpha ~ 1 - exp(-dt/window)
            alpha = 1.0 - np.exp(-dt / max(1e-6, self.ema_window_sec))
        self.ema_motion = (1.0 - alpha) * self.ema_motion + alpha * value
        return self.ema_motion

    def _classify(self, motion_ema):
        label = 'disturbed' if motion_ema >= self.motion_ratio_thresh else 'calm'
        # Confidence ramps from threshold +/- 50% band
        lo = 0.5 * self.motion_ratio_thresh
        hi = 1.5 * self.motion_ratio_thresh
        if motion_ema <= lo:
            conf = 1.0 if label == 'calm' else 0.0
        elif motion_ema >= hi:
            conf = 1.0 if label == 'disturbed' else 0.0
        else:
            # Linear interpolation across band
            t = (motion_ema - lo) / max(1e-6, (hi - lo))
            conf = t if label == 'disturbed' else (1.0 - t)
        return label, float(np.clip(conf, 0.0, 1.0))

    def _detect_presence_hsv(self, bgr):
        """Return (present: bool, ratio: float) using HSV mask area ratio."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.presence_hsv_low, self.presence_hsv_high)
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=1)
        ratio = float(np.count_nonzero(mask)) / float(mask.size)
        return (ratio >= self.presence_min_ratio), ratio

    def run(self):
        rate = rospy.Rate(100)  # internal loop; we still honor fps_limit
        while not rospy.is_shutdown():
            if not self._ensure_open():
                rate.sleep()
                continue

            frame = self._read_frame()
            if frame is None:
                # Try to reopen
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None
                rate.sleep()
                continue

            now = time.time()
            dt = max(1e-3, now - self.last_time)
            self.last_time = now

            # Presence gate: only print/publish if fish present
            present_ratio = None
            if self.presence_enable:
                present, present_ratio = self._detect_presence_hsv(frame)
                if not present:
                    # Skip printing/publishing when no fish detected
                    if self.fps_limit and self.fps_limit > 0:
                        time.sleep(max(0.0, (1.0 / self.fps_limit) - dt))
                    else:
                        rate.sleep()
                    continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)

            motion_ratio = self._compute_motion_ratio(gray)
            motion_ema = self._update_ema(motion_ratio, dt)
            label, conf = self._classify(motion_ema)

            msg = {
                'timestamp': now,
                'label': label,
                'conf': round(conf, 3),
                'motion_ratio': round(float(motion_ema), 6),
                'source': 'opencv-mhi'
            }
            if present_ratio is not None:
                msg['presence_ratio'] = round(float(present_ratio), 6)
            # Print to console only when fish detected (presence gate above ensures this)
            rospy.loginfo(f"[OpenCVBehavior] {msg}")
            self.pub.publish(String(data=json.dumps(msg)))

            # Optional FPS cap
            if self.fps_limit and self.fps_limit > 0:
                time.sleep(max(0.0, (1.0 / self.fps_limit) - dt))
            else:
                rate.sleep()


if __name__ == '__main__':
    node = OpenCVBehaviorNode()
    node.run()
