#!/usr/bin/env python3
"""
fish_behavior_node.py
---------------------
Estimates fish behavior from camera frames. If an ONNX model exists at
models/fish_behavior.onnx, it will run inference; otherwise falls back to
simple motion heuristics (optical flow magnitude) to classify behavior as
calm/active/schooling (heuristic).

Publishes JSON-ish String messages to /ai/fish_behavior.

Params:
- ~model_path (default: models/fish_behavior.onnx)
- ~camera_topic (default: /camera/image_raw)
- ~publish_topic (default: /ai/fish_behavior)
"""
import os
import json
import time
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

try:
    import onnxruntime as ort
except Exception:
    ort = None


class FishBehaviorNode:
    def __init__(self):
        rospy.init_node('fish_behavior_node')
        self.model_path = rospy.get_param('~model_path', 'models/fish_behavior.onnx')
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/image_raw')
        self.publish_topic = rospy.get_param('~publish_topic', '/ai/fish_behavior')

        self.session = None
        if os.path.exists(self.model_path) and ort is not None:
            try:
                self.session = ort.InferenceSession(self.model_path, providers=['CUDAExecutionProvider','CPUExecutionProvider'])
                rospy.loginfo(f"Loaded fish behavior model: {self.model_path}")
            except Exception as e:
                rospy.logwarn(f"Failed to load behavior ONNX: {e}")

        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.publish_topic, String, queue_size=10)
        self.prev_gray = None

        rospy.Subscriber(self.camera_topic, Image, self.image_cb, queue_size=1, buff_size=2**22)

    def preprocess(self, bgr):
        img = cv2.resize(bgr, (224, 224))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img = (img - mean) / std
        chw = np.transpose(img, (2, 0, 1))
        return np.expand_dims(chw, axis=0)

    def infer_model(self, bgr):
        x = self.preprocess(bgr)
        input_name = self.session.get_inputs()[0].name
        output_name = self.session.get_outputs()[0].name
        logits = self.session.run([output_name], {input_name: x})[0]
        probs = self.softmax(logits[0])
        idx = int(np.argmax(probs))
        # Example labels; adjust if your model differs
        labels = ['calm', 'active', 'schooling']
        label = labels[idx] if idx < len(labels) else str(idx)
        return {
            'label': label,
            'conf': float(probs[idx])
        }

    @staticmethod
    def softmax(x):
        x = x - np.max(x)
        e = np.exp(x)
        return e / np.sum(e)

    def infer_heuristic(self, bgr):
        # Use optical flow magnitude to estimate activity
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self.prev_gray is None:
            self.prev_gray = gray
            return {'label': 'unknown', 'conf': 0.0, 'motion': 0.0}
        flow = cv2.calcOpticalFlowFarneback(self.prev_gray, gray, None,
                                            pyr_scale=0.5, levels=3, winsize=15,
                                            iterations=3, poly_n=5, poly_sigma=1.2, flags=0)
        self.prev_gray = gray
        mag, _ = cv2.cartToPolar(flow[...,0], flow[...,1])
        motion = float(np.mean(mag))
        # Thresholds are heuristic; tune for your camera/fish
        if motion < 0.3:
            label = 'calm'
        elif motion < 1.0:
            label = 'active'
        else:
            label = 'schooling'
        return {'label': label, 'conf': 0.5, 'motion': motion}

    def image_cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn(f"cv_bridge failed: {e}")
            return
        if self.session is not None:
            res = self.infer_model(bgr)
            res['source'] = 'model'
        else:
            res = self.infer_heuristic(bgr)
            res['source'] = 'heuristic'
        res['timestamp'] = time.time()
        self.pub.publish(String(data=json.dumps(res)))


def main():
    FishBehaviorNode()
    rospy.spin()


if __name__ == '__main__':
    main()
