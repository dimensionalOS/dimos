#!/usr/bin/env python3
"""Add a purple object with physics collider near the agent."""
import json, uuid, time, websocket

ws = websocket.WebSocket()
ws.connect("ws://localhost:8090?ch=control")
ws.settimeout(10)

mid = str(uuid.uuid4())[:8]
code = """
const ap = agent.getPosition();
const ax = ap[0], az = ap[2];

const group = new THREE.Group();
group.name = "test-purple-object";
const mat = new THREE.MeshStandardMaterial({color: 0x8B00FF, roughness: 0.3, metalness: 0.1});

// Shaft — cylinder
const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.5, 16), mat);
shaft.position.y = 0.35;
group.add(shaft);

// Tip — hemisphere
const tip = new THREE.Mesh(new THREE.SphereGeometry(0.12, 16, 16), mat);
tip.position.y = 0.65;
group.add(tip);

// Left ball
const ballL = new THREE.Mesh(new THREE.SphereGeometry(0.1, 16, 16), mat);
ballL.position.set(-0.12, 0.05, 0);
group.add(ballL);

// Right ball
const ballR = new THREE.Mesh(new THREE.SphereGeometry(0.1, 16, 16), mat);
ballR.position.set(0.12, 0.05, 0);
group.add(ballR);

group.position.set(ax + 2, 0.5, az);
scene.add(group);

const info = addCollider(group, "box");
return {name: group.name, pos: {x: +group.position.x.toFixed(1), y: +group.position.y.toFixed(1), z: +group.position.z.toFixed(1)}, collider: info}
"""

ws.send(json.dumps({"type": "exec", "id": mid, "code": code}))

deadline = time.time() + 10
while time.time() < deadline:
    raw = ws.recv()
    if isinstance(raw, bytes):
        continue
    msg = json.loads(raw)
    if msg.get("type") == "execResult" and msg.get("id") == mid:
        if msg.get("success"):
            print(f"Added: {msg['result']}")
        else:
            print(f"ERROR: {msg.get('error')}")
        break

ws.close()
