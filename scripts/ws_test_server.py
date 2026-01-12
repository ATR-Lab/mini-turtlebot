#!/usr/bin/env python3
import asyncio
import websockets
import time
import json

async def handler(ws):
    # Send a heartbeat every second and print what the client sends
    async def tx():
        i = 0
        while True:
            msg = {"v":1,"type":"hb","ts_ms":int(time.time()*1000),"i":i}
            await ws.send(json.dumps(msg) + "\n")
            i += 1
            await asyncio.sleep(1)        
            
            
            
    async def rx():
        async for m in ws:
            print("RX FROM CLIENT:", m)

            # Handle JSONL input (may contain multiple lines)
            for line in str(m).splitlines():
                line = line.strip()
                if not line:
                    continue

                try:
                    obj = json.loads(line)
                except Exception:
                    continue

                if obj.get("type") == "cfg":
                    ack = {
                        "v": 1,
                        "type": "ack",
                        "id": obj.get("id", -1),
                        "ok": True,
                        "msg": "streams updated"
                    }
                    await ws.send(json.dumps(ack) + "\n")        
            



    await asyncio.gather(tx(), rx())

async def main():
    async with websockets.serve(handler, "127.0.0.1", 9000):
        print("WS test server on ws://127.0.0.1:9000")
        await asyncio.Future()

asyncio.run(main())

