#!/usr/bin/env python3
"""ROS node that either embeds the websocket server (in-process) or acts as a websocket client bridge.

By default it will attempt to embed the server (useful for local deployments). If param '~mode' == 'client'
it will run in client mode and connect to an external server URI.
"""

import rospy
import asyncio
from std_msgs.msg import String
from esp.msg import BotStatus
import importlib.util
import os
import sys


def import_websocket_module():
    # Attempt to import the local websocket.py as a module
    here = os.path.dirname(__file__)
    path = os.path.join(here, 'websocket.py')
    spec = importlib.util.spec_from_file_location('esp.websocket', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ESPWebsocketNode:
    def __init__(self):
        rospy.init_node('esp_websocket_node', anonymous=True)
        self.mode = rospy.get_param('~mode', 'embedded')

        if self.mode == 'client':
            # client mode will connect to external websocket server
            self.uri = rospy.get_param('~uri', 'ws://localhost:8765/esp/1')
            self._start_client()
        else:
            # embedded server mode: import websocket.py and start server
            ws_mod = import_websocket_module()
            self.server_node = ws_mod.ESPServerNode()
            self.server_node.start()

            # Expose send_command via ROS topic subscription for convenience
            rospy.Subscriber('/bot/cmd', String, self._on_cmd)

            # Mirror bot status if needed by other nodes (already published by server)
            self.status_pub = rospy.Publisher('/bot/status', BotStatus, queue_size=10)

    def _on_cmd(self, msg):
        # Forward to server's send_command; assumes single bot id '1' by default
        try:
            self.server_node.send_command('1', msg.data)
        except Exception as e:
            rospy.logerr(f"Failed to forward command to server: {e}")

    def _start_client(self):
        # Lazy import to avoid websockets dependency in embedded mode
        import websockets
        import json

        async def run_client(uri, status_pub):
            async with websockets.connect(uri) as websocket:
                rospy.loginfo(f"Connected to {uri}")
                async for message in websocket:
                    rospy.loginfo(f"Received from WS: {message}")
                    try:
                        data = json.loads(message)
                        bot_status = BotStatus()
                        bot_status.bot_id = int(data.get('bot_id', 0))
                        bot_status.cleaning = bool(data.get('cleaning', False))
                        bot_status.battery_level = float(data.get('battery_level', 1.0))
                        bot_status.status = json.dumps(data)
                    except Exception:
                        bot_status = BotStatus()
                        bot_status.bot_id = 0
                        bot_status.cleaning = 'cleaning' in message.lower()
                        bot_status.battery_level = 1.0
                        bot_status.status = message
                    status_pub.publish(bot_status)

        self.uri = rospy.get_param('~uri', 'ws://localhost:8765/esp/1')
        self.status_pub = rospy.Publisher('/bot/status', BotStatus, queue_size=10)
        loop = asyncio.get_event_loop()
        loop.run_until_complete(run_client(self.uri, self.status_pub))


if __name__ == '__main__':
    node = ESPWebsocketNode()
    rospy.spin()


class ESPWebsocketNode:
    def __init__(self):
        rospy.init_node('esp_websocket_node')

        self.uri = rospy.get_param('~uri', 'ws://localhost:8765')
        self.bot_id = rospy.get_param('~bot_id', 1)

        self.status_pub = rospy.Publisher('/bot/status', BotStatus, queue_size=10)
        rospy.Subscriber('/bot/cmd', BotCommand, self.cmd_cb)

        # Asyncio event loop in background thread
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._start_loop, daemon=True)
        self.thread.start()

        # Connect
        asyncio.run_coroutine_threadsafe(self._connect(), self.loop)

    def _start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def _connect(self):
        try:
            async with websockets.connect(self.uri) as ws:
                rospy.loginfo(f"Connected to {self.uri}")
                await self._recv_loop(ws)
        except Exception as e:
            rospy.logerr(f"WebSocket connection failed: {e}")

    async def _recv_loop(self, ws):
        async for msg in ws:
            # For simplicity, expect 'status:...' strings
            rospy.loginfo(f"Received from ESP: {msg}")
            status = BotStatus()
            status.bot_id = self.bot_id
            status.cleaning = 'cleaning' in msg
            status.battery_level = 1.0
            status.status = msg
            self.status_pub.publish(status)

    def cmd_cb(self, msg: BotCommand):
        # Forward command to ESP via websocket
        async def send_cmd():
            try:
                async with websockets.connect(self.uri) as ws:
                    await ws.send(f"CMD {msg.zone_id} {msg.action} {msg.speed}")
            except Exception as e:
                rospy.logerr(f"Failed to send command to ESP: {e}")
        asyncio.run_coroutine_threadsafe(send_cmd(), self.loop)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    node = ESPWebsocketNode()
    node.spin()
