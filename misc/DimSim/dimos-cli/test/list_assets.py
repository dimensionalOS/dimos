#!/usr/bin/env python3
"""List top-level asset groups in the scene (the actual furniture/objects)."""
import json, uuid, time, websocket

ws = websocket.WebSocket()
ws.connect("ws://localhost:8090?ch=control")
ws.settimeout(10)

msg_id = str(uuid.uuid4())[:8]
code = """
const ag = scene.getObjectByName("assetsGroup");
if (!ag) return {error: "no assetsGroup"};
const items = [];
for (const child of ag.children) {
  let label = "";
  child.traverse(obj => {
    if (!label && obj.name) {
      const n = obj.name;
      if (n.includes("assetGroup:")) {
        const parts = n.split(":");
        if (parts.length >= 3) label = parts[2];
      } else if (n.includes("assetPrim:")) {
        const parts = n.split(":");
        if (parts.length >= 3 && !label) label = parts[2];
      }
    }
  });
  let meshCount = 0;
  child.traverse(obj => { if (obj.isMesh) meshCount++; });
  items.push({
    id: child.name || "(no name)",
    label: label || "(unnamed)",
    pos: {x: +child.position.x.toFixed(1), y: +child.position.y.toFixed(1), z: +child.position.z.toFixed(1)},
    meshCount
  });
}
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
        if not msg.get("success"):
            print(f"ERROR: {msg.get('error')}")
            break
        assets = msg.get("result", [])
        print(f"Found {len(assets)} assets:\n")
        for i, a in enumerate(assets):
            print(f"  [{i:2d}] {a['label']:30s}  pos=({a['pos']['x']}, {a['pos']['y']}, {a['pos']['z']})  meshes={a['meshCount']}")
            print(f"       id: {a['id']}")
        break

ws.close()
