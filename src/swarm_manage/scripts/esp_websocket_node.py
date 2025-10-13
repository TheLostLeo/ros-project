#!/usr/bin/env python3
"""Simple WebSocket server at ws://localhost:8765 that reads and writes messages.

This script purposefully avoids ROS publishers/subscribers for now.
"""

import asyncio
import websockets

HOST = '0.0.0.0'
PORT = 8765


async def handler(websocket, path):
    peer = getattr(websocket, 'remote_address', None)
    print(f"[WS] Client connected: {peer} path={path}")
    try:
        # Send a greeting on connect
        await websocket.send("hello from server")

        async for message in websocket:
            print(f"[WS] <- {message}")
            # Echo back the payload; adjust here for custom logic
            await websocket.send(f"echo: {message}")
    except websockets.exceptions.ConnectionClosed as e:
        print(f"[WS] Client disconnected: {peer} code={e.code} reason={e.reason}")
    except Exception as e:
        print(f"[WS] Error: {e}")


async def main():
    print(f"[WS] Starting server at ws://{HOST}:{PORT}")
    async with websockets.serve(handler, HOST, PORT):
        # Run forever
        await asyncio.Future()


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[WS] Server stopped")
