#!/usr/bin/env python3
"""WebSocket server implementation that bridges ESP clients to ROS topics."""

import rospy
import asyncio
import websockets
from std_msgs.msg import String
from esp.msg import BotStatus
import threading
import json


class ESPServerNode:
    def __init__(self, host=None, port=None):
        # allow constructing without reinitializing rospy
        if not rospy.core.is_initialized():
            rospy.init_node("esp_manager", anonymous=True)

        self.host = host or rospy.get_param("~host", "0.0.0.0")
        self.port = port or rospy.get_param("~port", 8765)

        self.clients = {}  # bot_id -> websocket

        rospy.loginfo(f"Starting WebSocket server on {self.host}:{self.port}")

        # Command publisher for bot 1 (keeps backward compatibility)
        self.cmd_pub_esp1 = rospy.Publisher("/esp/1/cmd", String, queue_size=10, latch=True)
        rospy.loginfo("Created publisher for /esp/1/cmd")

        # BotStatus publisher for higher-level nodes
        self.bot_status_pub = rospy.Publisher('/bot/status', BotStatus, queue_size=10)
        rospy.loginfo("Created publisher for /bot/status")

        # Asyncio event loop in background thread
        self.loop = asyncio.new_event_loop()
        self.server = None
        self.thread = None

    def start(self):
        # start the websocket server in background thread
        asyncio.set_event_loop(self.loop)
        self.server = websockets.serve(self.handler, self.host, self.port, loop=self.loop)
        self.loop.run_until_complete(self.server)
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()

    async def handler(self, websocket, path):
        rospy.loginfo(f"New connection with path: {path}")

        parts = path.strip("/").split("/")
        if len(parts) == 2 and parts[0] == "esp":
            bot_id = parts[1]
        else:
            rospy.logwarn(f"Invalid path: {path}")
            await websocket.close()
            return

        self.clients[bot_id] = websocket
        rospy.loginfo(f"[ESP-{bot_id}] registered.")

        # Subscribe to the ROS cmd topic for this bot when connected
        rospy.Subscriber(f"esp/{bot_id}/cmd", String, self.make_cmd_callback(bot_id))

        try:
            async for message in websocket:
                rospy.loginfo(f"[ESP-{bot_id}] -> {message}")
                # Try to parse JSON status, otherwise treat as free text
                try:
                    data = json.loads(message)
                    status_text = json.dumps(data)
                    cleaning = bool(data.get('cleaning', False))
                    battery = float(data.get('battery_level', 1.0))
                except Exception:
                    status_text = message
                    cleaning = 'cleaning' in message.lower()
                    battery = 1.0

                bot_status = BotStatus()
                try:
                    bot_status.bot_id = int(bot_id)
                except Exception:
                    bot_status.bot_id = 0
                bot_status.cleaning = cleaning
                bot_status.battery_level = battery
                bot_status.status = status_text
                self.bot_status_pub.publish(bot_status)
        except websockets.exceptions.ConnectionClosed:
            rospy.logwarn(f"[ESP-{bot_id}] disconnected.")
        finally:
            if bot_id in self.clients:
                del self.clients[bot_id]

    def make_cmd_callback(self, bot_id):
        """ROS subscriber callback that forwards /esp/{id}/cmd → ESP-{id}"""
        def callback(msg):
            rospy.loginfo(f"Forwarding to ESP-{bot_id}: {msg.data}")
            self.send_command(bot_id, msg.data)
        return callback

    def send_command(self, bot_id, command):
        async def send_async():
            ws = self.clients.get(str(bot_id))
            if ws:
                try:
                    await ws.send(command)
                    rospy.loginfo(f"Sent to ESP-{bot_id}: {command}")
                except Exception as e:
                    rospy.logerr(f"Failed to send to ESP-{bot_id}: {e}")
            else:
                rospy.logwarn(f"No active WebSocket for ESP-{bot_id}, cannot send '{command}'")

        asyncio.run_coroutine_threadsafe(send_async(), self.loop)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ESPServerNode()
        node.start()
        node.spin()
    except rospy.ROSInterruptException:
        pass
