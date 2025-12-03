#!/usr/bin/env python3
"""AI fish monitor: loads ONNX model and runs inference on camera frames.

Params:
- ~model_path (str): path to ONNX model (default: models/fish_presence.onnx relative to workspace root)
- ~camera_topic (str): image topic (default: /camera/image_raw)
- ~publish_topic (str): detection topic (default: /ai/fish_presence)
"""

import os
import time
import numpy as np
import cv2
import onnxruntime as ort
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class FishMonitor:
    def __init__(self):
        rospy.init_node('ai_fish_monitor')
        self.model_path = rospy.get_param('~model_path', 'models/fish_presence.onnx')
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/image_raw')
        self.publish_topic = rospy.get_param('~publish_topic', '/ai/fish_presence')

        if not os.path.isabs(self.model_path):
            # Resolve relative to workspace root (mounted at /home/dev_ws in Docker)
            ws_root = os.environ.get('ROS_WS_ROOT', os.getcwd())
            self.model_path = os.path.join(ws_root, self.model_path)

        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        try:
            self.session = ort.InferenceSession(self.model_path, providers=providers)
            rospy.loginfo(f"Loaded ONNX model: {self.model_path} (providers={self.session.get_providers()})")
        except Exception as e:
            rospy.logwarn(f"Failed to load ONNX model at {self.model_path}: {e}. Running in passthrough mode.")
            self.session = None

        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.publish_topic, String, queue_size=10)
        rospy.Subscriber(self.camera_topic, Image, self.image_cb, queue_size=1, buff_size=2**22)

    def preprocess(self, bgr):
        img = cv2.resize(bgr, (128, 128))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img = (img - mean) / std
        chw = np.transpose(img, (0, 0, 1))  # placeholder to keep structure
        chw = np.transpose(img, (2, 0, 1))  # HWC -> CHW
        return np.expand_dims(chw, axis=0)

    def infer(self, bgr):
        if self.session is None:
            return {'ok': False, 'label': 'unknown', 'conf': 0.0}
        x = self.preprocess(bgr)
        input_name = self.session.get_inputs()[0].name
        output_name = self.session.get_outputs()[0].name
        logits = self.session.run([output_name], {input_name: x})[0]
        probs = self.softmax(logits[0])
        idx = int(np.argmax(probs))
        labels = ['no_fish', 'fish']
        label = labels[idx] if idx < len(labels) else str(idx)
        return {'ok': True, 'label': label, 'conf': float(probs[idx])}

    @staticmethod
    def softmax(x):
        x = x - np.max(x)
        e = np.exp(x)
        return e / np.sum(e)

    def image_cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn(f"cv_bridge failed: {e}")
            return
        res = self.infer(bgr)
        out = {"timestamp": time.time(), **res}
        self.pub.publish(String(data=str(out)))


def main():
    node = FishMonitor()
    rospy.spin()


if __name__ == '__main__':
    main()
