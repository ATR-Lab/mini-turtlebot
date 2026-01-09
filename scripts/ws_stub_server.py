#!/usr/bin/env python3
import asyncio
import websockets

HOST = "0.0.0.0"
PORT = 9000

async def handler(ws):
    peer = ws.remote_address
    print(f"[stub] client connected: {peer}")
    try:
        async for msg in ws:
            # msg includes the '\n' you send; strip to show clean JSON
            print(f"[stub] RX: {msg.strip()}")
    except websockets.exceptions.ConnectionClosed as e:
        print(f"[stub] client closed: {peer} ({e.code})")

async def main():
    print(f"[stub] listening on ws://{HOST}:{PORT}/")
    async with websockets.serve(handler, HOST, PORT):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())

