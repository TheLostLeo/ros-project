#!/usr/bin/env python3
"""
A ROS node that acts as a bridge between a WebSocket server and ROS topics.
It translates messages from ESP32 clients into ROS messages and vice-versa.
"""

import asyncio
import json
import websockets
import rospy
from std_msgs.msg import String
from swarm_manage.msg import BotCommand, BotStatus

class WebSocketBridge:
    def __init__(self):
        rospy.init_node('esp_websocket_node', anonymous=True)

        # --- WebSocket Server State ---
        self.clients = {}  # Maps bot_id (int) to websocket connection
        self.host = rospy.get_param('~host', '0.0.0.0')
        self.port = rospy.get_param('~port', 8765)
        self.loop = None  # asyncio event loop used by the server

        # --- ROS Communication ---
        self.status_pub = rospy.Publisher('/bot/status', BotStatus, queue_size=10)
        rospy.Subscriber('/bot/cmd', BotCommand, self.handle_bot_command)
        # Subscribe to per-bot simple command topics using namespacing convention: /bot/<id>/simple_cmd
        # For a small number of bots, we can pre-subscribe; for dynamic bots, parse connection map.
        # As a generic fallback, also listen to a default topic /bot/simple_cmd and route to bot 1.
        self.default_bot_id = rospy.get_param('~default_bot_id', 1)
        rospy.Subscriber('/bot/simple_cmd', String, self.handle_simple_cmd_default)

        rospy.loginfo(f"WebSocket bridge starting on ws://{self.host}:{self.port}")

    def handle_bot_command(self, msg: BotCommand):
        """Callback for when a command is published on /bot/cmd."""
        bot_id = self.default_bot_id  # In a multi-bot system, you'd get this from the msg or topic namespace
        
        if bot_id in self.clients:
            websocket = self.clients[bot_id]
            try:
                # Create a JSON command to send to the ESP32
                command_payload = {
                    "action": msg.action,
                    "zone_id": msg.zone_id,
                    "speed": msg.speed
                }
                command_str = json.dumps(command_payload)
                
                rospy.loginfo(f"[ROS->WS] Sending command to bot {bot_id}: {command_str}")
                
                # Create a future to send the message from the server's asyncio loop
                if self.loop is not None:
                    asyncio.run_coroutine_threadsafe(
                        websocket.send(command_str),
                        self.loop
                    )
                else:
                    rospy.logerr("Asyncio loop not initialized yet; cannot send command.")
            except Exception as e:
                rospy.logerr(f"Error sending command to bot {bot_id}: {e}")
        else:
            rospy.logwarn(f"Command received for bot {bot_id}, but it is not connected.")

    def handle_simple_cmd_default(self, msg):
        """Forward a plain-text command to the default bot_id (param ~default_bot_id)."""
        self._send_text_command(self.default_bot_id, msg.data)

    def _send_text_command(self, bot_id, text):
        if bot_id in self.clients:
            websocket = self.clients[bot_id]
            try:
                payload = text if isinstance(text, str) else str(text)
                rospy.loginfo(f"[ROS->WS] Sending simple cmd to bot {bot_id}: {payload}")
                if self.loop is not None:
                    asyncio.run_coroutine_threadsafe(
                        websocket.send(payload),
                        self.loop
                    )
                else:
                    rospy.logerr("Asyncio loop not initialized yet; cannot send simple command.")
            except Exception as e:
                rospy.logerr(f"Error sending simple command to bot {bot_id}: {e}")
        else:
            rospy.logwarn(f"Simple command for bot {bot_id}, but it is not connected.")

    async def handler(self, websocket, path):
        """Handles incoming WebSocket connections."""
        try:
            # Extract bot_id from the path, e.g., /esp/1
            bot_id = int(path.strip('/').split('/')[-1])
            self.clients[bot_id] = websocket
            rospy.loginfo(f"[WS] Bot {bot_id} connected from {websocket.remote_address}")
            # Create a per-bot simple command subscriber lazily on first connection
            topic = f"/bot/{bot_id}/simple_cmd"
            rospy.Subscriber(topic, String, lambda m, b=bot_id: self._send_text_command(b, m.data))

            async for message in websocket:
                rospy.loginfo(f"[WS->ROS] Received from bot {bot_id}: {message}")
                try:
                    # 1. Parse the JSON message from the ESP32
                    data = json.loads(message)

                    # 2. Create and populate the ROS BotStatus message
                    status_msg = BotStatus()
                    status_msg.bot_id = data.get('bot_id', bot_id)
                    status_msg.status = data.get('status', 'unknown')
                    status_msg.cleaning = data.get('cleaning', False)
                    status_msg.battery_level = data.get('battery_level', 0.0)
                    
                    # 3. Publish the message to the ROS network
                    self.status_pub.publish(status_msg)

                except json.JSONDecodeError:
                    rospy.logwarn(f"Received non-JSON message from bot {bot_id}: {message}")
                except Exception as e:
                    rospy.logerr(f"Error processing message from bot {bot_id}: {e}")

        except ValueError:
            rospy.logerr(f"Invalid path from client: {path}. Expected /esp/<bot_id>")
        except websockets.exceptions.ConnectionClosed as e:
            rospy.loginfo(f"Connection from {websocket.remote_address} closed: {e.reason}")
        finally:
            # Remove client on disconnect
            disconnected_id = None
            for bot_id, client_ws in self.clients.items():
                if client_ws == websocket:
                    disconnected_id = bot_id
                    break
            if disconnected_id:
                del self.clients[disconnected_id]
                rospy.loginfo(f"[WS] Bot {disconnected_id} disconnected.")

    async def start_server(self):
        """Starts the WebSocket server."""
        # Capture the running event loop so ROS callbacks can schedule coroutines
        self.loop = asyncio.get_running_loop()
        async with websockets.serve(self.handler, self.host, self.port):
            # Keep the server running while ROS is alive
            while not rospy.is_shutdown():
                await asyncio.sleep(0.1)

def main():
    bridge = WebSocketBridge()
    try:
        # This will block until the node is shut down
        asyncio.run(bridge.start_server())
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    rospy.loginfo("WebSocket bridge shutting down.")

if __name__ == '__main__':
    main()
