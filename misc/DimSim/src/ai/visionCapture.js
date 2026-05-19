import * as THREE from "three";

// =============================================================================
// AGENT POV CAPTURE SYSTEM
// =============================================================================
// 
// For Gaussian splats to render correctly, the SparkRenderer needs to sort splats
// based on the camera position. This sorting happens during the render call.
// 
// ARCHITECTURE:
// Instead of capturing mid-frame, we use a request/callback system:
// 1. Agent requests a capture -> we queue the request
// 2. Main render loop sees pending request
// 3. Main loop renders from AGENT's POV first (splats get sorted for agent)
// 4. Capture the result
// 5. Then render from PLAYER's POV (splats get re-sorted for player)
//
// This ensures the agent always gets a properly rendered frame.

const _lampByAgent = new WeakMap();

// Pending capture requests: Map<agentId, { agent, resolve, reject, params }>
const _pendingCaptures = new Map();

// Track if we have a pending capture
export function hasPendingCapture() {
  return _pendingCaptures.size > 0;
}

// Get all pending captures
export function getPendingCaptures() {
  return Array.from(_pendingCaptures.values());
}

// Clear a pending capture after it's processed
export function clearPendingCapture(agentId) {
  _pendingCaptures.delete(agentId);
}

/**
 * Request a capture - returns a promise that resolves when the capture is complete.
 * The actual capture is performed by the main render loop via processPendingCaptures().
 */
export function requestAgentCapture({
  agent,
  renderer,
  scene,
  mainCamera,
  width = 960,        // Wider for more human-like FOV
  height = 432,       // ~2.2:1 aspect ratio (wider than 16:9)
  eyeHeight = 0.55,
  jpegQuality = 0.75,
  fov = 80,           // Wider vertical FOV for more peripheral vision
  near = 0.05,
  far = 2000,
  headLamp = null,
  preRender = null,
  renderFn = null,    // optional: (renderer, scene, camera) => void — renders active view mode
}) {
  return new Promise((resolve, reject) => {
    if (!agent) {
      reject(new Error("No agent provided"));
      return;
    }
    
    const agentId = agent.id || "default";
    
    // Store the capture request
    _pendingCaptures.set(agentId, {
      agent,
      renderer,
      scene,
      mainCamera,
      width,
      height,
      eyeHeight,
      jpegQuality,
      fov,
      near,
      far,
      headLamp,
      preRender,
      renderFn,
      resolve,
      reject,
    });
  });
}

/**
 * Process all pending captures - called from the main render loop.
 * This renders from each agent's POV and captures the result.
 * 
 * We do a "warm-up" render first to trigger splat sorting, wait for the
 * renderer to complete, then do the actual capture render.
 */
export async function processPendingCaptures() {
  const captures = getPendingCaptures();
  if (captures.length === 0) return;
  
  for (const capture of captures) {
    try {
      const base64 = await performCaptureWithDelay(capture);
      capture.resolve(base64);
    } catch (e) {
      capture.reject(e);
    } finally {
      clearPendingCapture(capture.agent.id || "default");
    }
  }
}

/**
 * Perform capture with a delay to allow SparkRenderer to sort splats.
 * 1. First render triggers splat sorting for agent's viewpoint
 * 2. Wait 500ms for GPU sorting to complete
 * 3. Second render captures with properly sorted splats
 */
async function performCaptureWithDelay(captureParams) {
  const {
    agent,
    renderer,
    scene,
    mainCamera,
    width,
    height,
    eyeHeight,
    fov,
    near,
    far,
    headLamp,
    preRender,
    jpegQuality,
  } = captureParams;

  if (!agent || !renderer || !scene || !mainCamera) return null;

  let lamp = null;

  // Calculate agent's eye position and direction
  const [ax, ay, az] = agent.getPosition?.() || [0, 0, 0];
  const yaw = agent.group?.rotation?.y ?? 0;
  const pitch = typeof agent.pitch === "number" ? agent.pitch : 0;
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const forward = new THREE.Vector3(Math.sin(yaw) * cp, sp, Math.cos(yaw) * cp);
  const eyeY = ay + eyeHeight;

  // Use a dedicated offscreen camera so user view never switches.
  const captureCamera = new THREE.PerspectiveCamera(fov, width / height, near, far);
  captureCamera.position.set(ax, eyeY, az);
  captureCamera.lookAt(ax + forward.x, eyeY + forward.y, az + forward.z);
  captureCamera.updateProjectionMatrix();
  captureCamera.updateMatrixWorld(true);

  const prevTarget = renderer.getRenderTarget?.() || null;
  const captureTarget = new THREE.WebGLRenderTarget(width, height, {
    minFilter: THREE.LinearFilter,
    magFilter: THREE.LinearFilter,
    format: THREE.RGBAFormat,
    depthBuffer: true,
    stencilBuffer: false,
  });

  // Optional capture-only fill light. Keep null for strict view parity.
  if (headLamp && typeof headLamp === "object") {
    lamp = _lampByAgent.get(agent);
    if (!lamp) {
      lamp = new THREE.PointLight(0xffffff, headLamp.intensity, headLamp.distance, headLamp.decay);
      _lampByAgent.set(agent, lamp);
    }
    const fwdN = forward.clone().normalize();
    const up = new THREE.Vector3(0, 1, 0);
    const off = headLamp.offset || { x: 0, y: 1.0, z: 0.6 };
    const right = new THREE.Vector3().crossVectors(fwdN, up).normalize();
    const lampPos = new THREE.Vector3(ax, eyeY, az)
      .addScaledVector(right, off.x)
      .addScaledVector(up, off.y)
      .addScaledVector(fwdN, off.z);
    lamp.position.copy(lampPos);
    scene.add(lamp);
  }

  // Call preRender callback
  let cleanup = null;
  if (typeof preRender === "function") {
    try {
      cleanup = preRender(captureCamera) || null;
    } catch {
      // ignore
    }
  }

  const { renderFn } = captureParams;
  const doRender = () => {
    renderer.setRenderTarget(captureTarget);
    if (typeof renderFn === "function") {
      // Intentionally pass captureCamera; renderFn should avoid mutating player camera.
      renderFn(renderer, scene, captureCamera);
    } else {
      renderer.render(scene, captureCamera);
    }
    renderer.setRenderTarget(null);
  };

  // FIRST RENDER: Trigger splat sorting for agent's viewpoint
  doRender();

  // Give Spark sorting a short moment to settle.
  await new Promise(resolve => setTimeout(resolve, 180));

  // SECOND RENDER: Capture with properly sorted splats
  doRender();

  // Read pixels from offscreen target and encode JPEG.
  const raw = new Uint8Array(width * height * 4);
  renderer.readRenderTargetPixels(captureTarget, 0, 0, width, height, raw);
  const flipped = new Uint8ClampedArray(width * height * 4);
  const rowBytes = width * 4;
  for (let y = 0; y < height; y++) {
    const srcY = height - 1 - y;
    const srcOff = srcY * rowBytes;
    const dstOff = y * rowBytes;
    // Match on-screen output transform: convert linear RT pixels to sRGB.
    for (let i = 0; i < rowBytes; i += 4) {
      const r = raw[srcOff + i + 0] / 255;
      const g = raw[srcOff + i + 1] / 255;
      const b = raw[srcOff + i + 2] / 255;
      const a = raw[srcOff + i + 3];
      const toSrgb = (x) => (x <= 0.0031308 ? 12.92 * x : 1.055 * Math.pow(x, 1 / 2.4) - 0.055);
      flipped[dstOff + i + 0] = Math.max(0, Math.min(255, Math.round(toSrgb(r) * 255)));
      flipped[dstOff + i + 1] = Math.max(0, Math.min(255, Math.round(toSrgb(g) * 255)));
      flipped[dstOff + i + 2] = Math.max(0, Math.min(255, Math.round(toSrgb(b) * 255)));
      flipped[dstOff + i + 3] = a;
    }
  }
  const cvs = document.createElement("canvas");
  cvs.width = width;
  cvs.height = height;
  const ctx = cvs.getContext("2d");
  if (!ctx) return null;
  ctx.putImageData(new ImageData(flipped, width, height), 0, 0);
  const dataUrl = cvs.toDataURL("image/jpeg", jpegQuality);

  // Remove optional capture lamp
  if (lamp) scene.remove(lamp);

  // Call cleanup
  if (typeof cleanup === "function") {
    try {
      cleanup();
    } catch {
      // ignore
    }
  }

  renderer.setRenderTarget(prevTarget);
  captureTarget.dispose();

  const idx = dataUrl.indexOf("base64,");
  return idx !== -1 ? dataUrl.slice(idx + "base64,".length) : null;
}

// Legacy function for backwards compatibility - now just calls requestAgentCapture
export async function captureAgentPovBase64(params) {
  return requestAgentCapture(params);
}
