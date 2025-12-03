#!/usr/bin/env python3
"""
esp_decision_node.py
---------------------
Simple decision node that drives the bot in a zig-zag pattern using plain text
commands on /bot/cmd and reads bot status from /bot/status_raw (as emitted by
esp_websocket_node).

Behavior:
- Start moving forward until an edge is detected in inbound status JSON.
- When edge detected: stop, turn (alternate left/right at each row end), and continue.
- Keeps looping (zig-zag) until shutdown.

Assumptions:
- The ESP32 sends periodic JSON strings over WebSocket which the bridge publishes on
  /bot/status_raw. We expect keys like {"edge": true/false} optionally.
- Commands are strings: "forward", "backward", "left", "right", "stop".

Params:
- ~cmd_topic (str): command topic to publish strings. Default: /bot/cmd
- ~status_topic (str): inbound raw status topic. Default: /bot/status_raw
- ~row_pause (float): seconds to pause after turn. Default: 0.25
- ~step_interval (float): seconds between status polls when moving. Default: 0.3
- ~require_edge_key (bool): if true, only act on JSON with an explicit 'edge' key. Default: false

"""
import json
import threading
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


class DecisionNode:
    def __init__(self):
        rospy.init_node('esp_decision_node')
        self.cmd_topic = rospy.get_param('~cmd_topic', '/bot/cmd')
        self.status_topic = rospy.get_param('~status_topic', '/bot/status_raw')
        self.row_pause = float(rospy.get_param('~row_pause', 0.25))
        self.step_interval = float(rospy.get_param('~step_interval', 0.3))
        self.require_edge_key = bool(rospy.get_param('~require_edge_key', False))

        self.pub = rospy.Publisher(self.cmd_topic, String, queue_size=10)
        rospy.Subscriber(self.status_topic, String, self.status_cb)

        self.lock = threading.Lock()
        self.last_status = {}
        self.edge_detected = False
        self.turn_left_next = True  # alternate turns for zig-zag
        self.running = False
        self.worker = None

        # Services to start/stop cleaning
        self.start_srv = rospy.Service('/start_cleaning', Trigger, self.start_cb)
        self.stop_srv = rospy.Service('/stop_cleaning', Trigger, self.stop_cb)

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo('ESP Decision Node ready. Use /start_cleaning to begin zig-zag.')

    def status_cb(self, msg: String):
        text = msg.data.strip()
        try:
            data = json.loads(text)
            with self.lock:
                self.last_status = data
                # Edge logic: look for explicit boolean field 'edge', else infer from status
                edge = None
                if isinstance(data, dict):
                    if 'edge' in data:
                        edge = bool(data.get('edge'))
                    elif not self.require_edge_key:
                        # accept synonyms or heuristics
                        if 'distance_cm' in data and data['distance_cm'] is not None:
                            try:
                                dist = float(data['distance_cm'])
                                edge = (dist > 0 and dist < 12.0)  # same as ESP threshold
                            except Exception:
                                pass
                        if edge is None and 'status' in data:
                            # e.g., status could be 'idle' immediately after an auto-stop
                            s = str(data['status']).lower()
                            if s in ('edge', 'stopped'):
                                edge = True
                if edge is not None:
                    self.edge_detected = edge
        except Exception:
            # Not JSON; ignore
            pass

    def send(self, text: str):
        self.pub.publish(String(data=text))

    def zigzag_loop(self):
        rate = rospy.Rate(max(1, int(1.0 / max(0.01, self.step_interval))))
        while not rospy.is_shutdown() and self.running:
            # Move forward while no edge detected
            self.send('forward')
            self.edge_detected = False
            while not rospy.is_shutdown() and self.running and not self.edge_detected:
                # Ask for status (optional: if ESP handles 'status' requests)
                self.send('status')
                rate.sleep()

            if rospy.is_shutdown():
                break

            # Edge detected: stop and turn
            self.send('stop')
            rospy.sleep(0.1)
            if self.turn_left_next:
                self.send('left')
            else:
                self.send('right')
            self.turn_left_next = not self.turn_left_next
            rospy.sleep(self.row_pause)

        # On shutdown, ensure stop
        self.send('stop')

    def start_cb(self, _req) -> TriggerResponse:
        if self.running:
            return TriggerResponse(success=True, message='Already running')
        self.running = True
        self.worker = threading.Thread(target=self.zigzag_loop, daemon=True)
        self.worker.start()
        return TriggerResponse(success=True, message='Zig-zag started')

    def stop_cb(self, _req) -> TriggerResponse:
        if not self.running:
            return TriggerResponse(success=True, message='Already stopped')
        self.running = False
        # Allow loop to exit
        if self.worker and self.worker.is_alive():
            self.worker.join(timeout=2.0)
        self.send('stop')
        return TriggerResponse(success=True, message='Zig-zag stopped')

    def _on_shutdown(self):
        try:
            self.running = False
            if self.worker and self.worker.is_alive():
                self.worker.join(timeout=2.0)
            self.send('stop')
        except Exception:
            pass

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    node = DecisionNode()
    node.spin()
