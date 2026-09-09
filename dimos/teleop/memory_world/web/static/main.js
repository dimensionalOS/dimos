// Memory World browser entry point:
//   - opens /ws_memory_world
//   - hands payloads to WorldScene (point cloud + Street-View markers)
//   - routes gestures from InputAdapter directly to the scene (locomotion
//     is client-side state; server doesn't need a copy)

import { InputAdapter } from '/static_mw/input_adapter.js';
import {
    MSG_IMAGE_POSES,
    MSG_IMAGE_THUMBNAIL,
    MSG_ODOM_TRAIL,
    MSG_POINT_CLOUD,
    MSG_QUERY_IMAGE,
    MSG_TOP_DOWN_MAP,
    decodeBinary,
    decodeText,
    encodeText,
} from '/static_mw/protocol.js';

const statusEl = document.getElementById('status');
const connectBtn = document.getElementById('connectBtn');
const disconnectBtn = document.getElementById('disconnectBtn');
const micBtn = document.getElementById('micBtn');
const logEl = document.getElementById('log');
const backgroundMode = document.body.dataset.backgroundMode || 'black';

let ws = null;
let xrSession = null;
let xrRefSpace = null;
let scene = null;
let input = null;
let WorldScene = null;
let pendingQueryResult = null;
let lastViewerPoseSent = 0;
let perfReadoutTimer = null;
let micStream = null;
let recorder = null;
const pendingDiag = [];

// The page is served at the module's client_route, so /voice hangs off it.
const voiceUrl = `${window.location.pathname.replace(/\/$/, '')}/voice`;

function log(msg) {
    if (!logEl) return;
    const line = `[${new Date().toLocaleTimeString()}] ${msg}\n`;
    logEl.textContent = (logEl.textContent + line).split('\n').slice(-12).join('\n');
}

function diag(event, fields = {}) {
    const line = `[diag] ${event} ${JSON.stringify(fields)}`;
    console.log(line);
    log(line.slice(0, 100));
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(encodeText('diag', { event, ...fields }));
    } else {
        pendingDiag.push({ event, fields });
    }
}

function flushPendingDiag() {
    while (pendingDiag.length && ws && ws.readyState === WebSocket.OPEN) {
        const { event, fields } = pendingDiag.shift();
        ws.send(encodeText('diag', { event, ...fields }));
    }
}

diag('module_load');

try {
    const mod = await import('/static_mw/scene.js');
    WorldScene = mod.WorldScene;
    diag('scene_module_loaded');
} catch (err) {
    diag('scene_module_failed', { error: String(err && err.message || err) });
}

function setStatus(msg) {
    statusEl.textContent = msg;
}

window.onerror = (msg, url, line, col, err) => {
    console.error(`[err] ${msg} at ${url}:${line}:${col}`, err);
    setStatus(`Error: ${msg}`);
    diag('window_error', { msg: String(msg), url: String(url), line, col });
};
window.addEventListener('unhandledrejection', (e) => {
    diag('unhandled_rejection', { reason: String(e.reason && e.reason.message || e.reason) });
});

// ---- WebSocket -------------------------------------------------------------

function setupWebSocket() {
    return new Promise((resolve, reject) => {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws_memory_world`;
        setStatus('Connecting to server…');
        ws = new WebSocket(wsUrl);
        ws.binaryType = 'arraybuffer';

        ws.onopen = () => {
            setStatus('Server connected — starting VR…');
            flushPendingDiag();
            diag('ws_open');
            resolve();
        };
        ws.onerror = (e) => {
            console.error('[ws] error', e);
            setStatus('WebSocket error');
            reject(e);
        };
        ws.onclose = () => {
            log('ws closed');
            setStatus('Disconnected');
        };
        ws.onmessage = (event) => {
            if (typeof event.data === 'string') {
                handleControl(decodeText(event.data));
            } else {
                handleBinary(event.data);
            }
        };
    });
}

const pendingSceneMsgs = [];

function applySceneMsg(m) {
    if (!scene) return;
    if (m.kind === 'point_cloud') scene.setPointCloud(m.header, m.payload);
    else if (m.kind === 'image_poses') scene.setImagePoses(m.header, m.payload);
    else if (m.kind === 'odom_trail') scene.setOdomTrail(m.header, m.payload);
    else if (m.kind === 'top_down_map') scene.setTopDownMap(m.header, m.payload);
    else if (m.kind === 'image_thumbnail') scene.addImageThumbnail(m.header.index, m.payload);
    else if (m.kind === 'query_image') scene.addQueryImage(m.header, m.payload);
}

function flushSceneMsgs() {
    if (!scene) return;
    while (pendingSceneMsgs.length) applySceneMsg(pendingSceneMsgs.shift());
}

function handleBinary(buffer) {
    const { msgType, header, payload } = decodeBinary(buffer);
    let kind;
    if (msgType === MSG_POINT_CLOUD) kind = 'point_cloud';
    else if (msgType === MSG_IMAGE_POSES) kind = 'image_poses';
    else if (msgType === MSG_ODOM_TRAIL) kind = 'odom_trail';
    else if (msgType === MSG_TOP_DOWN_MAP) kind = 'top_down_map';
    else if (msgType === MSG_IMAGE_THUMBNAIL) kind = 'image_thumbnail';
    else if (msgType === MSG_QUERY_IMAGE) kind = 'query_image';
    else { log(`unknown bin type ${msgType}`); return; }

    if (scene) applySceneMsg({ kind, header, payload });
    else pendingSceneMsgs.push({ kind, header, payload });
}

function handleControl(msg) {
    if (!msg) return;
    switch (msg.type) {
        case 'world_summary':
            log(`world n=${msg.n}${msg.has_colors ? ' rgb' : ''}`);
            break;
        case 'ready':
            setStatus('World loaded — left stick walks, pinch both hands to scale');
            diag('server_ready');
            break;
        case 'query_result':
            if (scene) scene.setQueryResult(msg);
            else pendingQueryResult = msg;
            setStatus(msg.answer || 'Memory result highlighted');
            break;
        case 'voice_transcript':
            setStatus(`Heard: “${msg.text}” — searching…`);
            if (scene) scene.setHeardText(msg.text);
            break;
        case 'error':
            setStatus(`Server error: ${msg.message || 'unknown'}`);
            break;
        case 'pong':
            break;
        default:
            log(`unknown control ${msg.type}`);
    }
}

// All locomotion gestures go straight to the scene. We still echo them to the
// server as diag so the host terminal can see what the headset is doing.
function dispatchGesture(g) {
    if (!scene) return;
    switch (g.type) {
        case 'locomote': scene.applyLocomote(g); break;
        case 'yaw': scene.applyYaw(g); break;
        case 'teleport_aim': scene.setTeleportAim(g); break;
        case 'teleport_commit': scene.applyTeleportCommit(g); break;
        case 'teleport_cancel': scene.clearTeleportAim(); break;
        case 'scale_delta': scene.applyScale(g); break;
        case 'reset_view': scene.resetView(); break;
        case 'toggle_images': scene.toggleImages(); break;
        case 'toggle_cloud': scene.toggleCloud(); break;
        case 'voice_start': startRecording(); break;
        case 'voice_stop': stopRecording(); break;
        default: break;
    }
    // Lightweight diag — only on discrete events, not continuous.
    if (g.type !== 'locomote' && g.type !== 'yaw' && g.type !== 'teleport_aim') {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(encodeText(g.type, g));
        }
    }
}

// ---- viewer ----------------------------------------------------------------

function buildScene() {
    if (!WorldScene) {
        throw new Error('Scene module failed to load — Three.js import did not resolve');
    }

    if (scene) scene.dispose();
    try {
        scene = new WorldScene(diag, backgroundMode);
        diag('scene_constructed');
        flushSceneMsgs();
        if (pendingQueryResult) {
            scene.setQueryResult(pendingQueryResult);
            pendingQueryResult = null;
        }
        diag('scene_msgs_flushed');
    } catch (e) {
        diag('scene_construct_failed', { error: String(e.message || e) });
        throw e;
    }
}

const perfEl = document.getElementById('perf');

function startPerfReadout() {
    if (perfReadoutTimer) clearInterval(perfReadoutTimer);
    perfEl.style.display = 'block';
    let sinceDiag = 0;
    perfReadoutTimer = setInterval(() => {
        if (!scene) return;
        const s = scene.getPerfStats();
        perfEl.textContent = [
            `${s.fps.toFixed(1)} fps  (${s.median_ms.toFixed(1)} ms med, ${s.p95_ms.toFixed(1)} p95)`,
            `${s.draw_calls} draws  ${(s.triangles / 1000).toFixed(0)}k tris`,
            `${s.textures} textures  ${s.live_quads} quads`,
            `quality ${s.quality}${s.quality_auto ? ' auto' : ' pinned'}  ${(s.voxels_drawn / 1000).toFixed(0)}k/${(s.voxels_total / 1000).toFixed(0)}k voxels`,
            `images ${s.images_visible ? 'on' : 'off'}  cloud ${s.cloud_visible ? 'on' : 'off'}`,
        ].join('\n');
        if (++sinceDiag >= 20) {
            sinceDiag = 0;
            diag('perf', s);
        }
    }, 250);
}

// Per-frame busy-wait for testing the quality governor on a machine too fast
// to trigger it on its own (window.app.simulateLoad(ms)).
let simulatedLoadMs = 0;

function sendViewerPose() {
    if (simulatedLoadMs > 0) {
        const until = performance.now() + simulatedLoadMs;
        while (performance.now() < until) { /* burn */ }
    }
    const now = performance.now();
    if (ws && ws.readyState === WebSocket.OPEN && now - lastViewerPoseSent >= 500) {
        lastViewerPoseSent = now;
        ws.send(encodeText('viewer_pose', { position: scene.getViewerRobotPosition() }));
    }
}

/** Enter VR when a headset is present, otherwise fall back to the flat viewer. */
async function startViewer() {
    buildScene();
    startPerfReadout();
    // ?flat skips WebXR even where a headset is present: plain WebGL in a window.
    const wantFlat = new URLSearchParams(window.location.search).has('flat');
    if (navigator.xr && !wantFlat) {
        try {
            await startVR();
            return;
        } catch (e) {
            diag('vr_unavailable_using_desktop', { error: String(e.message || e) });
        }
    } else {
        diag('vr_unavailable_using_desktop', { error: 'navigator.xr missing' });
    }
    document.body.classList.add('desktop-view');
    scene.startDesktop(sendViewerPose);
    setStatus('Desktop view — click to look, WASD to walk');
}

async function startVR() {
    let session;
    let mode;
    if (backgroundMode === 'passthrough') {
        mode = 'immersive-ar';
        session = await navigator.xr.requestSession(mode, {
            requiredFeatures: ['local-floor'],
            optionalFeatures: ['hand-tracking'],
        });
    } else {
        mode = 'immersive-vr';
        try {
            session = await navigator.xr.requestSession(mode, {
                requiredFeatures: ['local-floor'],
                optionalFeatures: ['hand-tracking'],
            });
        } catch (e) {
            diag('vr_failed', { error: String(e.message || e) });
            mode = 'immersive-ar';
            session = await navigator.xr.requestSession(mode, {
                requiredFeatures: ['local-floor'],
                optionalFeatures: ['hand-tracking'],
            });
        }
    }
    diag('xr_session_started', { mode, backgroundMode, blendMode: session.environmentBlendMode });
    xrSession = session;

    input = new InputAdapter(dispatchGesture);

    xrRefSpace = await session.requestReferenceSpace('local-floor');
    diag('ref_space_ready');

    session.addEventListener('end', () => {
        diag('xr_session_ended');
        xrSession = null;
        disconnect();
    });

    let frameCount = 0;
    await scene.setSession(session, (frame) => {
        frameCount++;
        if (frameCount === 1) diag('first_frame');
        if (input && frame) input.onFrame(frame, xrRefSpace, performance.now(), scene);
        sendViewerPose();
    });
    diag('animation_loop_set');

    setStatus(`VR active (${mode})`);
}

// ---- voice query -----------------------------------------------------------

// Quest Browser has no webkitSpeechRecognition, so the audio is recorded here
// and transcribed server-side. The permission prompt cannot be answered from
// inside an immersive session, so the stream is acquired before VR starts.
async function acquireMic() {
    if (micStream) return micStream;
    if (!navigator.mediaDevices || !window.MediaRecorder) {
        diag('voice_unsupported', { secure: window.isSecureContext });
        return null;
    }
    try {
        micStream = await navigator.mediaDevices.getUserMedia({ audio: true });
        diag('mic_ready');
    } catch (e) {
        diag('mic_denied', { error: String(e.message || e) });
        micStream = null;
    }
    return micStream;
}

async function startRecording() {
    if (recorder) return;
    const stream = await acquireMic();
    if (!stream) {
        setStatus('Microphone unavailable');
        return;
    }
    const chunks = [];
    recorder = new MediaRecorder(stream);
    recorder.ondataavailable = (e) => e.data.size && chunks.push(e.data);
    recorder.onstop = () => {
        recorder = null;
        sendRecording(new Blob(chunks, { type: chunks[0]?.type || 'audio/webm' }));
    };
    recorder.start();
    micBtn.classList.add('recording');
    setStatus('Listening…');
    diag('voice_recording_started');
}

function stopRecording() {
    micBtn.classList.remove('recording');
    if (recorder && recorder.state !== 'inactive') recorder.stop();
}

async function sendRecording(blob) {
    if (!blob.size) return;
    setStatus('Transcribing…');
    diag('voice_recording_sent', { bytes: blob.size, type: blob.type });
    const body = new FormData();
    body.append('audio', blob, 'query.webm');
    try {
        const response = await fetch(voiceUrl, { method: 'POST', body });
        const result = await response.json();
        diag('voice_answer', { transcript: result.transcript, success: result.success });
        setStatus(result.answer || result.detail || 'No answer');
    } catch (e) {
        diag('voice_failed', { error: String(e.message || e) });
        setStatus(`Voice query failed: ${e.message || e}`);
    }
}

// ---- touch controls ---------------------------------------------------------

const stickEl = document.getElementById('stick');
const stickKnob = document.getElementById('stickKnob');
const STICK_RADIUS_PX = 40;

function stickFrom(touch) {
    const rect = stickEl.getBoundingClientRect();
    const dx = touch.clientX - (rect.left + rect.width / 2);
    const dy = touch.clientY - (rect.top + rect.height / 2);
    const len = Math.hypot(dx, dy);
    const clamp = len > STICK_RADIUS_PX ? STICK_RADIUS_PX / len : 1;
    return { x: dx * clamp / STICK_RADIUS_PX, y: dy * clamp / STICK_RADIUS_PX };
}

function moveStick(event) {
    event.preventDefault();
    const touch = event.touches[0];
    if (!touch || !scene) return;
    const { x, y } = stickFrom(touch);
    stickKnob.style.transform = `translate(${x * STICK_RADIUS_PX}px, ${y * STICK_RADIUS_PX}px)`;
    scene.setTouchStick(x, y); // knob up (negative y) walks forward
}

function releaseStick() {
    stickKnob.style.transform = '';
    if (scene) scene.setTouchStick(0, 0);
}

stickEl.addEventListener('touchstart', moveStick, { passive: false });
stickEl.addEventListener('touchmove', moveStick, { passive: false });
stickEl.addEventListener('touchend', releaseStick);
stickEl.addEventListener('touchcancel', releaseStick);

document.getElementById('mapBtn').addEventListener('click', () => scene && scene.toggleMinimap());
document.getElementById('answerBtn').addEventListener('click', () => window.app.jumpTo(0));
document.getElementById('cameraBtn').addEventListener('click', () => {
    if (!scene || !scene._queryImages.length) return;
    scene.viewFrom((scene._queryImageCursor + 1) % scene._queryImages.length);
});

micBtn.addEventListener('pointerdown', startRecording);
micBtn.addEventListener('pointerup', stopRecording);
micBtn.addEventListener('pointerleave', stopRecording);

// ---- UI handlers -----------------------------------------------------------

async function connect() {
    try {
        connectBtn.disabled = true;
        await setupWebSocket();
        // Ask for the mic now, before VR starts (an immersive session cannot
        // show the prompt), but never make the world wait on the answer: an
        // unanswered prompt would otherwise leave the canvas black forever.
        void acquireMic();
        await startViewer();
        connectBtn.classList.add('hidden');
        disconnectBtn.classList.remove('hidden');
        micBtn.classList.remove('hidden');
    } catch (e) {
        console.error(e);
        setStatus(`Connection failed: ${e.message || e}`);
        connectBtn.disabled = false;
    }
}

async function disconnect() {
    setStatus('Disconnecting…');
    if (xrSession) {
        try { await xrSession.end(); } catch (_) { /* already ending */ }
        xrSession = null;
    }
    if (ws) {
        try { ws.close(); } catch (_) { /* ignore */ }
        ws = null;
    }
    if (perfReadoutTimer) {
        clearInterval(perfReadoutTimer);
        perfReadoutTimer = null;
    }
    perfEl.style.display = 'none';
    document.body.classList.remove('desktop-view');
    connectBtn.classList.remove('hidden');
    connectBtn.disabled = false;
    disconnectBtn.classList.add('hidden');
    micBtn.classList.add('hidden');
    setStatus('Disconnected');
}

window.app = {
    connect,
    disconnect,
    diag,
    perf: () => (scene ? scene.getPerfStats() : null),
    resetPerf: () => scene && scene.resetPerf(),
    benchmark: (frames) => (scene ? scene.benchmarkRender(frames) : null),
    // Bring the i-th answer of the last result in front of the viewer (also key J).
    jumpTo: (index = 0) => scene && scene._lastResultPoints.length > index
        && !scene.viewFrom(index) && scene.focusOn(scene._lastResultPoints[index].position),
    minimap: () => scene && scene.toggleMinimap(),
    // Stand where the camera behind the i-th answer stood (also key P, cycling).
    viewFrom: (index = 0) => scene && scene.viewFrom(index),
    // Pin a quality level (0 = everything, 4 = least) or null for automatic.
    quality: (level = null) => scene && scene.setQuality(level),
    simulateLoad: (ms = 0) => { simulatedLoadMs = Math.max(0, ms); return simulatedLoadMs; },
    // View state for automated checks: where the desktop camera looks and the world scale.
    viewState: () => scene && {
        yaw: scene._desktopYaw, pitch: scene._desktopPitch, scale: scene._worldGroup.scale.x,
        world: [scene._worldGroup.position.x, scene._worldGroup.position.y, scene._worldGroup.position.z].map((v) => +v.toFixed(3)),
    },
    queryImages: () => scene && scene._queryImages.map((h) => h && { index: h.index, position: h.position.map((v) => +v.toFixed(2)), forward: h.forward.map((v) => +v.toFixed(3)), up: h.up.map((v) => +v.toFixed(3)) }),
    // Marker orientations for automated checks: the quad normal of the first few capture poses, robot frame.
    markerNormals: (count = 6) => scene && scene._imagePoseMeta.slice(0, count).map((m) => {
        const { x, y, z, w } = m.quadQuat;
        // rotate (0, 0, 1) by q
        return [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)].map((v) => +v.toFixed(3));
    }),
};

// H pins the desktop menu and perf readout, which otherwise fade out once
// the world is up and only return on hover.
window.addEventListener('keydown', (event) => {
    if (event.code === 'KeyH' && document.body.classList.contains('desktop-view')) {
        document.body.classList.toggle('hud-visible');
    }
});

// The flat viewer runs anywhere, so a missing headset only changes the
// status line; Connect stays enabled.
window.addEventListener('load', async () => {
    const wantFlat = new URLSearchParams(window.location.search).has('flat');
    if (!navigator.xr || wantFlat) {
        setStatus(wantFlat ? 'Flat view — Connect to load the world' : 'No WebXR here — flat view on Connect');
        return;
    }
    try {
        const vr = await navigator.xr.isSessionSupported('immersive-vr').catch(() => false);
        const ar = await navigator.xr.isSessionSupported('immersive-ar').catch(() => false);
        const supported = backgroundMode === 'passthrough' ? ar : (vr || ar);
        if (!supported) {
            const requested = backgroundMode === 'passthrough' ? 'Passthrough AR' : 'VR/AR';
            setStatus(`${requested} not on this device — flat view on Connect`);
        }
    } catch (e) {
        log(`xr check failed: ${e.message || e}`);
    }
});
