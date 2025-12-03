#!/usr/bin/env python3
"""
ai_simulator_node.py
--------------------
Publishes random AI outputs at a fixed rate to mimic AI nodes:
- /ai/fish_presence: { label: fish|no_fish, conf: 0..1 }
- /ai/fish_behavior: { label: calm|active|schooling|stressed, conf: 0..1 }
- /ai/water_quality: { sharpness, saturation_mean, turbidity_score, quality, stress_index }

Also publishes control signals to /system/command based on simple rules:
- If fish is calm AND water quality is poor, publish 'start' (once) to begin cleaning
- If fish becomes agitated (active or stressed) while cleaning, publish 'stop'

Params:
- ~period_sec (float): seconds between publishes (default: 10.0)
- ~rate_hz (float): alternative to period; if set, overrides period
- ~enable_presence (bool)
- ~enable_behavior (bool)
- ~enable_quality (bool)
"""
import json
import random
import time
import rospy
from std_msgs.msg import String


class AISimulator:
    def __init__(self):
        rospy.init_node('ai_simulator')
        # One cycle every 10 seconds by default
        self.period_sec = float(rospy.get_param('~period_sec', 10.0))
        default_rate = 1.0 / max(0.001, self.period_sec)
        self.rate_hz = float(rospy.get_param('~rate_hz', default_rate))
        self.enable_presence = bool(rospy.get_param('~enable_presence', True))
        self.enable_behavior = bool(rospy.get_param('~enable_behavior', True))
        self.enable_quality = bool(rospy.get_param('~enable_quality', True))

        self.pub_presence = rospy.Publisher('/ai/fish_presence', String, queue_size=10)
        self.pub_behavior = rospy.Publisher('/ai/fish_behavior', String, queue_size=10)
        self.pub_quality = rospy.Publisher('/ai/water_quality', String, queue_size=10)
        # Swarm manager command topic
        self.pub_system_cmd = rospy.Publisher('/system/command', String, queue_size=10)

        # Track if we have started cleaning
        self.cleaning_active = False

    @staticmethod
    def rand_presence():
        label = random.choices(['fish', 'no_fish'], weights=[0.6, 0.4])[0]
        conf = round(random.uniform(0.5, 1.0), 3)
        return {'label': label, 'conf': conf}

    @staticmethod
    def rand_behavior():
        # Add 'stressed' state occasionally
        label = random.choices(['calm', 'active', 'schooling', 'stressed'], weights=[0.4, 0.3, 0.2, 0.1])[0]
        conf = round(random.uniform(0.4, 0.95), 3)
        return {'label': label, 'conf': conf}

    @staticmethod
    def rand_quality():
        sharp = round(random.uniform(50, 600), 1)
        sat = round(random.uniform(30, 180), 1)
        turbidity = round(random.uniform(0.2, 0.95), 3)
        quality = 'good' if turbidity > 0.66 else ('moderate' if turbidity > 0.33 else 'poor')
        # Derive a simple stress index blending behavior and quality proxies
        stress_index = round(random.uniform(0.0, 1.0) * (1.0 - turbidity), 3)
        return {
            'sharpness': sharp,
            'saturation_mean': sat,
            'turbidity_score': turbidity,
            'quality': quality,
            'stress_index': stress_index,
        }

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            ts = time.time()
            if self.enable_presence:
                p = {'timestamp': ts, **self.rand_presence()}
                self.pub_presence.publish(String(data=json.dumps(p)))

            b = None
            q = None
            if self.enable_behavior:
                b = {'timestamp': ts, **self.rand_behavior()}
                self.pub_behavior.publish(String(data=json.dumps(b)))
            if self.enable_quality:
                q = {'timestamp': ts, **self.rand_quality()}
                self.pub_quality.publish(String(data=json.dumps(q)))

            # Event logic
            try:
                behavior_label = (b or {}).get('label') if b else None
                water_quality = (q or {}).get('quality') if q else None
                if not self.cleaning_active and behavior_label == 'calm' and water_quality == 'poor':
                    self.pub_system_cmd.publish(String(data='start'))
                    rospy.loginfo('[AI Sim] Calm fish + poor water -> start')
                    self.cleaning_active = True
                elif self.cleaning_active and behavior_label in ('active', 'stressed'):
                    self.pub_system_cmd.publish(String(data='stop'))
                    rospy.loginfo('[AI Sim] Fish agitated -> stop')
                    self.cleaning_active = False
            except Exception:
                pass

            rate.sleep()


if __name__ == '__main__':
    node = AISimulator()
    node.run()
