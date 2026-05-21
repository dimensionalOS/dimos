// Memory World entry point. Same shape as memory_browser/main.js but:
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
    MSG_TOP_DOWN_MAP,
    decodeBinary,
    decodeText,
    encodeText,
} from '/static_mw/protocol.js';

const statusEl = document.getElementById('status');
const connectBtn = document.getElementById('connectBtn');
const disconnectBtn = document.getElementById('disconnectBtn');
const logEl = document.getElementById('log');

let ws = null;
let xrSession = null;
let xrRefSpace = null;
let scene = null;
let input = null;
let WorldScene = null;
const pendingDiag = [];

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
        default: break;
    }
    // Lightweight diag — only on discrete events, not continuous.
    if (g.type !== 'locomote' && g.type !== 'yaw' && g.type !== 'teleport_aim') {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(encodeText(g.type, g));
        }
    }
}

// ---- VR --------------------------------------------------------------------

async function startVR() {
    if (!WorldScene) {
        throw new Error('Scene module failed to load — Three.js import did not resolve');
    }

    try {
        scene = new WorldScene(diag);
        diag('scene_constructed');
        flushSceneMsgs();
        diag('scene_msgs_flushed');
    } catch (e) {
        diag('scene_construct_failed', { error: String(e.message || e) });
        throw e;
    }

    let session;
    let mode = 'immersive-vr';
    // Memory-world is opaque (we draw the cloud as the world), so prefer VR.
    try {
        session = await navigator.xr.requestSession('immersive-vr', {
            requiredFeatures: ['local-floor'],
            optionalFeatures: ['hand-tracking'],
        });
    } catch (e) {
        diag('vr_failed', { error: String(e.message || e) });
        mode = 'immersive-ar';
        session = await navigator.xr.requestSession('immersive-ar', {
            requiredFeatures: ['local-floor'],
            optionalFeatures: ['hand-tracking'],
        });
    }
    diag('xr_session_started', { mode });
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
        if (frameCount % 240 === 0) diag('frame_tick', { count: frameCount });
        if (input && frame) input.onFrame(frame, xrRefSpace, performance.now(), scene);
    });
    diag('animation_loop_set');

    setStatus(`VR active (${mode})`);
}

// ---- UI handlers -----------------------------------------------------------

async function connect() {
    try {
        connectBtn.disabled = true;
        if (!navigator.xr) throw new Error('WebXR unavailable. Use the Quest browser.');
        await setupWebSocket();
        await startVR();
        connectBtn.classList.add('hidden');
        disconnectBtn.classList.remove('hidden');
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
    connectBtn.classList.remove('hidden');
    connectBtn.disabled = false;
    disconnectBtn.classList.add('hidden');
    setStatus('Disconnected');
}

window.app = { connect, disconnect, diag };

window.addEventListener('load', async () => {
    if (!navigator.xr) {
        setStatus('WebXR not available in this browser');
        connectBtn.disabled = true;
        return;
    }
    try {
        const vr = await navigator.xr.isSessionSupported('immersive-vr').catch(() => false);
        const ar = await navigator.xr.isSessionSupported('immersive-ar').catch(() => false);
        if (!vr && !ar) {
            setStatus('VR/AR not supported on this device');
            connectBtn.disabled = true;
        }
    } catch (e) {
        log(`xr check failed: ${e.message || e}`);
    }
});
