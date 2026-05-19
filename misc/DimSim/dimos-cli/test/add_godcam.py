#!/usr/bin/env python3
"""Add a PIP god-view camera (top-down minimap) in the bottom-right corner."""
import json, uuid, time, websocket

ws = websocket.WebSocket()
ws.connect("ws://localhost:8090?ch=control")
ws.settimeout(10)

mid = str(uuid.uuid4())[:8]
code = """
// Remove existing PIP if re-running
if (window.__pipCleanup) { window.__pipCleanup(); }

// God camera — orthographic top-down
const pipCam = new THREE.OrthographicCamera(-10, 10, 10, -10, 0.1, 100);
pipCam.position.set(0, 40, 0);
pipCam.lookAt(0, 0, 0);

// Monkey-patch renderer to add PIP pass after main render
const _origRender = renderer.render.bind(renderer);
const pipSize = 250;

renderer.render = function(scn, cam) {
  // Main render (full viewport)
  _origRender(scn, cam);

  // PIP pass — bottom-right corner
  const w = renderer.domElement.width;
  const h = renderer.domElement.height;
  const margin = 10;

  // Follow agent
  const ap = agent.getPosition();
  pipCam.position.set(ap[0], 40, ap[2]);
  pipCam.lookAt(ap[0], 0, ap[2]);

  renderer.setViewport(w - pipSize - margin, margin, pipSize, pipSize);
  renderer.setScissor(w - pipSize - margin, margin, pipSize, pipSize);
  renderer.setScissorTest(true);
  renderer.autoClear = false;
  _origRender(scn, pipCam);
  renderer.autoClear = true;
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, w, h);
};

// Draw a border via CSS overlay
const border = document.createElement('div');
border.id = 'pip-border';
border.style.cssText = 'position:fixed;bottom:10px;right:10px;width:250px;height:250px;border:2px solid rgba(255,255,255,0.6);border-radius:4px;pointer-events:none;z-index:99999;';
document.body.appendChild(border);

// Cleanup function for re-running
window.__pipCleanup = () => {
  renderer.render = _origRender;
  border.remove();
  window.__pipCleanup = null;
};

return {godCam: true, pipSize}
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
            print(f"OK: {msg['result']}")
        else:
            print(f"ERROR: {msg.get('error')}")
        break

ws.close()
