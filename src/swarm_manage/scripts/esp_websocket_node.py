#!/usr/bin/env python3
import asyncio
from typing import Set, Optional, Any

import rospy
import websockets
from std_msgs.msg import String
import signal
import socket


NODE_NAME = "ros_websocket_bridge"
TOPIC_NAME = "/bot/cmd"  # default; can be overridden by ROS param 'topic'
STATUS_TOPIC = "/bot/status_raw"  # raw inbound messages from bots

# Global state for broadcasting from ROS callback into asyncio loop
CONNECTED_CLIENTS: Set[Any] = set()
ASYNC_LOOP: Optional[asyncio.AbstractEventLoop] = None
STATUS_PUB: Optional[rospy.Publisher] = None


def log(kind: str, msg: str) -> None:
    print(f"[{kind}] {msg}")


async def broadcast(payload: str) -> None:
    """Send payload to all connected clients; prunes closed connections."""
    if not CONNECTED_CLIENTS:
        return
    dead: Set[Any] = set()
    coros = []
    ws_list = []
    for ws in CONNECTED_CLIENTS:
        if ws.closed:
            dead.add(ws)
            continue
        coros.append(ws.send(payload))
        ws_list.append(ws)
    if coros:
        # Send concurrently; ignore individual send errors and drop bad sockets.
        results = await asyncio.gather(*coros, return_exceptions=True)
        for ws, res in zip(ws_list, results):
            if isinstance(res, Exception):
                dead.add(ws)
    # Remove closed/broken sockets
    for ws in dead:
        CONNECTED_CLIENTS.discard(ws)


def ros_cmd_callback(msg: String) -> None:
    """ROS subscriber callback: forwards plain text from /bot/cmd to WebSocket clients."""
    global ASYNC_LOOP
    # Forward raw string payload as-is (e.g., "forward", "left", etc.)
    payload = msg.data if hasattr(msg, 'data') else str(msg)
    loop = ASYNC_LOOP
    if loop is None:
        # Loop not ready yet; skip
        log("WARN", "Async loop not initialized; dropping message")
        return
    asyncio.run_coroutine_threadsafe(broadcast(payload), loop)
    log("SENT", f"{TOPIC_NAME} -> WS: {payload}")


async def handle_client(websocket: Any, path: str):
    """Handle a single websocket client connection."""
    CONNECTED_CLIENTS.add(websocket)
    log("INFO", f"Client connected: {websocket.remote_address}, path={path}")
    try:
        # We don't publish to /bot/cmd anymore. Just consume messages (if any) and ignore.
        async for message in websocket:
            # Publish raw inbound message to ROS for consumers (e.g., decision node)
            if STATUS_PUB is not None:
                try:
                    STATUS_PUB.publish(String(data=str(message)))
                except Exception as e:
                    log("ERROR", f"Failed to publish inbound WS message: {e}")
            log("RECV", str(message)[:200])
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        CONNECTED_CLIENTS.discard(websocket)
        log("INFO", f"Client disconnected: {websocket.remote_address}")


async def main():
    global ASYNC_LOOP
    global STATUS_PUB
    global TOPIC_NAME
    # Initialize ROS node and subscriber
    rospy.init_node(NODE_NAME, anonymous=True)

    # Set running loop for use in ROS callbacks
    ASYNC_LOOP = asyncio.get_running_loop()

    # Allow overriding via ROS params
    # Use a literal default to avoid scope issues before TOPIC_NAME is (re)assigned
    topic = rospy.get_param("~topic", "/bot/cmd")
    host = rospy.get_param("~host", "0.0.0.0")
    port = int(rospy.get_param("~port", 8765))

    # Update global topic name used in logs/serialization
    TOPIC_NAME = topic

    # Subscribe to topic (std_msgs/String) and forward to WebSocket clients
    rospy.Subscriber(topic, String, ros_cmd_callback)

    # Publisher for raw inbound messages from bots
    STATUS_PUB = rospy.Publisher(STATUS_TOPIC, String, queue_size=20)

    # Start WebSocket server
    server = await websockets.serve(handle_client, host, port)
    log("INFO", f"WebSocket server started on ws://{host}:{port}")
    # Also show a reachable IP if bound to 0.0.0.0 for convenience
    display_host = host
    if host in ("0.0.0.0", "::"):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                display_host = s.getsockname()[0]
        except Exception:
            # Fallback: keep provided host
            pass
    log("INFO", f"WebSocket reachable at: ws://{display_host}:{port}")

    # Keep running until ROS shutdown
    stop = asyncio.Future()

    def on_shutdown():
        if not stop.done():
            stop.set_result(True)

    # Wire shutdown events: ROS and POSIX signals (Ctrl-C)
    rospy.on_shutdown(on_shutdown)
    try:
        loop = asyncio.get_running_loop()
        loop.add_signal_handler(signal.SIGINT, on_shutdown)
        loop.add_signal_handler(signal.SIGTERM, on_shutdown)
    except (NotImplementedError, RuntimeError):
        # Signal handlers not available (e.g., non-main thread or platform); rely on ROS
        pass

    await stop

    # Graceful shutdown: close all client sockets and the server
    log("INFO", "Shutting down WebSocket server and closing client connections...")
    try:
        # Close client connections
        clients = list(CONNECTED_CLIENTS)
        for ws in clients:
            try:
                await ws.close(code=1001, reason="Server shutdown")
            except Exception:
                pass
        CONNECTED_CLIENTS.clear()

        # Close the server
        server.close()
        await server.wait_closed()
        log("INFO", "WebSocket server closed.")
    except Exception as e:
        log("ERROR", f"Error during websocket shutdown: {e}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except rospy.ROSInterruptException:
        pass
