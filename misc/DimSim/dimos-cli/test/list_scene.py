#!/usr/bin/env python3
"""List all named objects in the scene."""
import json, uuid, time, websocket

ws = websocket.WebSocket()
ws.connect("ws://localhost:8090?ch=control")
ws.settimeout(10)

msg_id = str(uuid.uuid4())[:8]
code = """
const items = [];
scene.traverse(obj => {
  if (obj.name) items.push({name: obj.name, type: obj.type, depth: 0});
});
return items;
"""
ws.send(json.dumps({"type": "exec", "id": msg_id, "code": code}))

deadline = time.time() + 10
while time.time() < deadline:
    raw = ws.recv()
    if isinstance(raw, bytes):
        continue
    msg = json.loads(raw)
    if msg.get("type") == "execResult" and msg.get("id") == msg_id:
        for obj in msg.get("result", []):
            print(f"  {obj['type']:20s}  {obj['name']}")
        break

ws.close()
