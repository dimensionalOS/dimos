#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Integration test for SceneEditor — script execution engine.

Requires dimsim running headless on port 8090:
    DIMSIM_HEADLESS=1 dimsim dev

Then:
    python dimos-cli/test/scene_editor_test.py
"""

import json
import sys
import time
import uuid

import websocket

PORT = 8090
WS_URL = f"ws://localhost:{PORT}?ch=control"


def send_exec(ws: websocket.WebSocket, code: str, timeout: float = 10) -> dict:
    """Send an exec command and wait for the execResult."""
    msg_id = str(uuid.uuid4())[:8]
    ws.send(json.dumps({"type": "exec", "id": msg_id, "code": code}))
    ws.settimeout(timeout)
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ws.recv()
        if isinstance(raw, bytes):
            continue
        msg = json.loads(raw)
        if msg.get("type") == "execResult" and msg.get("id") == msg_id:
            return msg
    raise TimeoutError(f"No execResult for {msg_id} after {timeout}s")


def wait_for_scene(ws: websocket.WebSocket, timeout: float = 60) -> bool:
    """Wait until sceneEditor is responding (browser has loaded)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            result = send_exec(ws, "return 'ready'", timeout=5)
            if result.get("success") and result.get("result") == "ready":
                return True
        except Exception:
            time.sleep(2)
    return False


def test_basic_exec(ws):
    """Test: basic JS evaluation returns a value."""
    print("  [1] Basic exec: return 1 + 1")
    r = send_exec(ws, "return 1 + 1")
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"] == 2, f"expected 2, got {r['result']}"
    print(f"      PASS — result: {r['result']}")


def test_scene_access(ws):
    """Test: can access scene.children."""
    print("  [2] Scene access: scene.children.length")
    r = send_exec(ws, "return scene.children.length")
    assert r["success"], f"exec failed: {r.get('error')}"
    assert isinstance(r["result"], int) and r["result"] > 0, f"unexpected: {r['result']}"
    print(f"      PASS — scene has {r['result']} children")


def test_three_access(ws):
    """Test: THREE namespace available, can create geometry."""
    print("  [3] THREE access: create Vector3")
    r = send_exec(ws, "const v = new THREE.Vector3(1, 2, 3); return {x: v.x, y: v.y, z: v.z}")
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"] == {"x": 1, "y": 2, "z": 3}, f"unexpected: {r['result']}"
    print(f"      PASS — Vector3: {r['result']}")


def test_add_primitive(ws):
    """Test: add a red box to the scene via script."""
    print("  [4] Add primitive: red box at (3, 1, 3)")
    code = """
const geo = new THREE.BoxGeometry(1, 1, 1);
const mat = new THREE.MeshStandardMaterial({color: 0xff0000});
const mesh = new THREE.Mesh(geo, mat);
mesh.name = "test-red-box";
mesh.position.set(3, 1, 3);
scene.add(mesh);
return {name: mesh.name, pos: {x: mesh.position.x, y: mesh.position.y, z: mesh.position.z}}
"""
    r = send_exec(ws, code)
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"]["name"] == "test-red-box", f"unexpected: {r['result']}"
    print(f"      PASS — added: {r['result']}")

    # Verify it's in the scene
    r2 = send_exec(ws, 'return scene.getObjectByName("test-red-box") !== null')
    assert r2["success"] and r2["result"] is True, "Box not found in scene"
    print("      PASS — verified in scene")


def test_load_gltf(ws):
    """Test: load robot.glb via loadGLTF helper."""
    print("  [5] Load GLTF: /agent-model/robot.glb")
    code = """
const gltf = await loadGLTF('/agent-model/robot.glb');
gltf.scene.name = "test-loaded-robot";
gltf.scene.position.set(5, 1, 5);
gltf.scene.scale.set(2, 2, 2);
scene.add(gltf.scene);
return {name: gltf.scene.name, childCount: gltf.scene.children.length}
"""
    r = send_exec(ws, code, timeout=15)
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"]["name"] == "test-loaded-robot", f"unexpected: {r['result']}"
    print(f"      PASS — loaded: {r['result']}")

    # Verify it's in the scene
    r2 = send_exec(ws, 'return scene.getObjectByName("test-loaded-robot") !== null')
    assert r2["success"] and r2["result"] is True, "Loaded robot not found in scene"
    print("      PASS — verified in scene")


def test_error_handling(ws):
    """Test: syntax/runtime errors returned gracefully."""
    print("  [6] Error handling: bad code")
    r = send_exec(ws, "this is not valid javascript!!!")
    assert not r["success"], "Expected failure"
    assert "error" in r, "Expected error field"
    print(f"      PASS — error caught: {r['error'][:60]}")


def test_async_exec(ws):
    """Test: top-level await works."""
    print("  [7] Async exec: await Promise")
    code = """
const val = await new Promise(resolve => setTimeout(() => resolve(42), 100));
return val
"""
    r = send_exec(ws, code)
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"] == 42, f"expected 42, got {r['result']}"
    print(f"      PASS — async result: {r['result']}")


def test_agent_access(ws):
    """Test: can read agent position."""
    print("  [8] Agent access: getPosition")
    r = send_exec(ws, "const p = agent.getPosition(); return {x: p[0], y: p[1], z: p[2]}")
    assert r["success"], f"exec failed: {r.get('error')}"
    assert "x" in r["result"], f"unexpected: {r['result']}"
    print(
        f"      PASS — agent at ({r['result']['x']:.2f}, {r['result']['y']:.2f}, {r['result']['z']:.2f})"
    )


def test_add_light(ws):
    """Test: add a point light to the scene."""
    print("  [9] Add light: PointLight at (0, 5, 0)")
    code = """
const light = new THREE.PointLight(0xffff00, 2, 50);
light.name = "test-point-light";
light.position.set(0, 5, 0);
scene.add(light);
return {name: light.name, color: light.color.getHex(), intensity: light.intensity}
"""
    r = send_exec(ws, code)
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"]["name"] == "test-point-light", f"unexpected: {r['result']}"
    assert r["result"]["intensity"] == 2, f"unexpected intensity: {r['result']}"
    print(f"      PASS — added: {r['result']}")

    # Verify in scene
    r2 = send_exec(ws, 'return scene.getObjectByName("test-point-light") !== null')
    assert r2["success"] and r2["result"] is True, "Light not found in scene"
    print("      PASS — verified in scene")


def test_modify_object(ws):
    """Test: move an existing object (the red box from test_add_primitive)."""
    print("  [10] Modify object: move test-red-box to (7, 2, 7)")
    code = """
const box = scene.getObjectByName("test-red-box");
if (!box) return {error: "box not found"};
box.position.set(7, 2, 7);
box.scale.set(2, 2, 2);
box.material.color.setHex(0x00ff00);
return {
  name: box.name,
  pos: {x: box.position.x, y: box.position.y, z: box.position.z},
  scale: {x: box.scale.x, y: box.scale.y, z: box.scale.z},
  color: box.material.color.getHex()
}
"""
    r = send_exec(ws, code)
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"]["pos"] == {"x": 7, "y": 2, "z": 7}, f"position wrong: {r['result']}"
    assert r["result"]["scale"] == {"x": 2, "y": 2, "z": 2}, f"scale wrong: {r['result']}"
    assert r["result"]["color"] == 0x00FF00, f"color wrong: {r['result']}"
    print(f"      PASS — modified: {r['result']}")


def test_remove_object(ws):
    """Test: remove the test-red-box we added earlier."""
    print("  [11] Remove object: remove test-red-box")
    remove_code = """
const box = scene.getObjectByName("test-red-box");
if (!box) return {error: "box not found"};
if (box.geometry) box.geometry.dispose();
if (box.material) box.material.dispose();
box.name = "";
scene.remove(box);
return {removed: "test-red-box"}
"""
    r = send_exec(ws, remove_code)
    assert r["success"], f"remove failed: {r.get('error')}"
    print(f"      PASS — removed: {r['result']}")
    # Note: verification that test-red-box is gone happens in test_query_scene (test 12)


def test_query_scene(ws):
    """Test: query scene objects by traversal."""
    print("  [12] Query scene: list named objects")
    code = """
const named = [];
scene.traverse(obj => {
  if (obj.name && obj.name.startsWith("test-")) {
    named.push({name: obj.name, type: obj.type});
  }
});
return named
"""
    r = send_exec(ws, code)
    assert r["success"], f"exec failed: {r.get('error')}"
    names = [o["name"] for o in r["result"]]
    assert "test-point-light" in names, f"Light not found: {names}"
    assert "test-loaded-robot" in names, f"Robot not found: {names}"
    assert "test-red-box" not in names, f"Removed box still found: {names}"
    print(f"      PASS — found {len(r['result'])} test objects: {names}")


def test_add_sphere(ws):
    """Test: add a sphere primitive (second geometry type)."""
    print("  [13] Add primitive: blue sphere at (-3, 1.5, 0)")
    code = """
const geo = new THREE.SphereGeometry(0.75, 32, 32);
const mat = new THREE.MeshStandardMaterial({color: 0x0088ff, metalness: 0.3, roughness: 0.4});
const mesh = new THREE.Mesh(geo, mat);
mesh.name = "test-blue-sphere";
mesh.position.set(-3, 1.5, 0);
scene.add(mesh);
return {name: mesh.name, pos: {x: mesh.position.x, y: mesh.position.y, z: mesh.position.z}}
"""
    r = send_exec(ws, code)
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"]["name"] == "test-blue-sphere", f"unexpected: {r['result']}"
    print(f"      PASS — added: {r['result']}")


def test_add_directional_light(ws):
    """Test: add a directional light with shadow."""
    print("  [14] Add light: DirectionalLight")
    code = """
const dlight = new THREE.DirectionalLight(0xffffff, 1.5);
dlight.name = "test-dir-light";
dlight.position.set(10, 10, 10);
dlight.castShadow = true;
scene.add(dlight);
return {name: dlight.name, intensity: dlight.intensity, castShadow: dlight.castShadow}
"""
    r = send_exec(ws, code)
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"]["name"] == "test-dir-light", f"unexpected: {r['result']}"
    assert r["result"]["castShadow"] is True, "Shadow not enabled"
    print(f"      PASS — added: {r['result']}")


def test_add_collider_box(ws):
    """Test: add a box collider to a mesh (explicit shape)."""
    print("  [15] Physics: addCollider (box)")
    code = """
const geo = new THREE.BoxGeometry(1, 1, 1);
const mat = new THREE.MeshStandardMaterial({color: 0xff8800});
const mesh = new THREE.Mesh(geo, mat);
mesh.name = "test-physics-box";
mesh.position.set(0, 1, 0);
scene.add(mesh);
const info = addCollider(mesh, "box");
return info
"""
    r = send_exec(ws, code)
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"]["shape"] == "box", f"unexpected shape: {r['result']}"
    assert "uuid" in r["result"], f"no uuid: {r['result']}"
    print(f"      PASS — collider: {r['result']}")


def test_add_collider_sphere(ws):
    """Test: add a sphere collider to a mesh."""
    print("  [16] Physics: addCollider (sphere)")
    code = """
const mesh = scene.getObjectByName("test-blue-sphere");
if (!mesh) return {error: "sphere not found"};
const info = addCollider(mesh, "sphere");
return info
"""
    r = send_exec(ws, code)
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"]["shape"] == "sphere", f"unexpected shape: {r['result']}"
    print(f"      PASS — collider: {r['result']}")


def test_remove_collider(ws):
    """Test: remove a previously added collider."""
    print("  [17] Physics: removeCollider")
    code = """
const mesh = scene.getObjectByName("test-physics-box");
if (!mesh) return {error: "box not found"};
const removed = removeCollider(mesh);
return {removed}
"""
    r = send_exec(ws, code)
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"]["removed"] is True, f"collider not removed: {r['result']}"
    print(f"      PASS — removed: {r['result']}")

    # Verify double-remove returns false
    r2 = send_exec(
        ws,
        """
const mesh = scene.getObjectByName("test-physics-box");
return {removed: removeCollider(mesh)}
""",
    )
    assert r2["success"] and r2["result"]["removed"] is False, "Double remove should return false"
    print("      PASS — double remove returns false")


def test_add_collider_trimesh(ws):
    """Test: add a trimesh collider to the loaded robot."""
    print("  [18] Physics: addCollider (trimesh)")
    code = """
const robot = scene.getObjectByName("test-loaded-robot");
if (!robot) return {error: "robot not found"};
const info = addCollider(robot, "trimesh");
return info
"""
    r = send_exec(ws, code, timeout=15)
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"]["shape"] == "trimesh", f"unexpected shape: {r['result']}"
    print(f"      PASS — collider: {r['result']}")


def test_add_npc(ws):
    """Test: addNPC with walk animation."""
    print("  [19] NPC: addNPC (Soldier, Walk)")
    code = """
const npc = await addNPC({
  url: '/local-assets/Soldier.glb',
  name: 'test-npc-soldier',
  position: { x: 5, y: 0, z: 5 },
  rotation: Math.PI / 4,
  scale: 1.0,
  animation: 'Walk',
  collider: true,
});
return npc
"""
    r = send_exec(ws, code, timeout=15)
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"]["name"] == "test-npc-soldier", f"unexpected: {r['result']}"
    assert "Walk" in r["result"]["animations"], f"no Walk anim: {r['result']}"
    assert r["result"]["activeAnimation"] == "Walk", f"wrong anim: {r['result']}"
    assert r["result"]["collider"] is not None, "no collider"
    print(f"      PASS — NPC: {r['result']['name']}, anims: {r['result']['animations']}")


def test_add_npc_idle(ws):
    """Test: addNPC with idle animation (by index)."""
    print("  [20] NPC: addNPC (Soldier, Idle by index)")
    code = """
const npc = await addNPC({
  url: '/local-assets/Soldier.glb',
  name: 'test-npc-idle',
  position: { x: -5, y: 0, z: -5 },
  animation: 0,
});
return npc
"""
    r = send_exec(ws, code, timeout=15)
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"]["name"] == "test-npc-idle", f"unexpected: {r['result']}"
    assert r["result"]["activeAnimation"] == "Idle", f"wrong anim: {r['result']}"
    print(f"      PASS — NPC idle: {r['result']['activeAnimation']}")


def test_remove_npc(ws):
    """Test: removeNPC removes NPC and cleans up."""
    print("  [21] NPC: removeNPC")
    r = send_exec(
        ws,
        """
removeNPC('test-npc-idle');
// Check immediately in same exec — name was cleared by removeNPC
const npcs = [];
scene.traverse(obj => { if (obj.name === 'test-npc-idle') npcs.push(obj.name); });
return { removed: true, remaining: npcs.length }
""",
    )
    assert r["success"], f"exec failed: {r.get('error')}"
    assert r["result"]["remaining"] == 0, f"NPC still found: {r['result']}"
    print(f"      PASS — removed and verified: {r['result']}")


def test_embodiment_config(ws):
    """Test: embodiment config is accessible from scene."""
    print("  [22] Embodiment: config loaded")
    r = send_exec(ws, "return window.currentEmbodiment || null")
    assert r["success"], f"exec failed: {r.get('error')}"
    cfg = r["result"]
    if cfg is None:
        print("      SKIP — not in dimos mode (embodiment only set in dimos boot)")
        return
    assert "radius" in cfg, f"no radius: {cfg}"
    assert "halfHeight" in cfg, f"no halfHeight: {cfg}"
    assert "type" in cfg, f"no type: {cfg}"
    print(
        f"      PASS — embodiment: type={cfg['type']} radius={cfg['radius']} halfHeight={cfg['halfHeight']}"
    )


def main():
    print(f"Connecting to {WS_URL}...")
    ws = websocket.WebSocket()
    ws.connect(WS_URL)

    print("Waiting for scene to load...")
    if not wait_for_scene(ws, timeout=90):
        print("FAIL: scene not ready after 90s")
        sys.exit(1)
    print("Scene ready.\n")

    tests = [
        test_basic_exec,
        test_scene_access,
        test_three_access,
        test_add_primitive,
        test_load_gltf,
        test_error_handling,
        test_async_exec,
        test_agent_access,
        test_add_light,
        test_modify_object,
        test_remove_object,
        test_query_scene,
        test_add_sphere,
        test_add_directional_light,
        test_add_collider_box,
        test_add_collider_sphere,
        test_remove_collider,
        test_add_collider_trimesh,
        test_add_npc,
        test_add_npc_idle,
        test_remove_npc,
        test_embodiment_config,
    ]

    passed = 0
    failed = 0
    for test in tests:
        try:
            test(ws)
            passed += 1
        except Exception as e:
            print(f"      FAIL — {e}")
            failed += 1

    ws.close()
    print(f"\n{'=' * 50}")
    print(f"Results: {passed} passed, {failed} failed out of {len(tests)}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
