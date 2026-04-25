// Memory Browser entry point. Wires together:
//   - WebSocket transport (binary thumbnails + text control)
//   - InputAdapter (controller-driven gesture primitives — Phase 1)
//   - RibbonScene (Three.js timeline ribbon + active-frame highlight)

import { InputAdapter } from '/static_mb/input_adapter.js';
import {
    MSG_GLOBAL_MAP,
    MSG_THUMBNAIL,
    decodeBinary,
    decodeText,
    encodeText,
} from '/static_mb/protocol.js';

const statusEl = document.getElementById('status');
const connectBtn = document.getElementById('connectBtn');
const disconnectBtn = document.getElementById('disconnectBtn');
const logEl = document.getElementById('log');
const canvas = document.getElementById('canvas');

let ws = null;
let xrSession = null;
let xrRefSpace = null;
let scene = null;
let input = null;
let RibbonScene = null;
// Buffered diag events emitted before the ws is open.
const pendingDiag = [];

function log(msg) {
    if (!logEl) return;
    const line = `[${new Date().toLocaleTimeString()}] ${msg}\n`;
    logEl.textContent = (logEl.textContent + line).split('\n').slice(-12).join('\n');
}

// Send a tagged event to the server so we can read the headset's progress in
// the host terminal. Buffers if the WS isn't open yet so the early lifecycle
// (script import, scene module load) is still visible after connect.
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

// Import RibbonScene dynamically so a Three.js load failure doesn't kill main.js.
try {
    const mod = await import('/static_mb/scene.js');
    RibbonScene = mod.RibbonScene;
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
        const wsUrl = `${protocol}//${window.location.host}/ws_memory`;
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

// Messages that arrive before the scene is constructed (WS opens before
// startVR() finishes) get queued and replayed once the scene exists.
const pendingSceneMsgs = [];

function applySceneMsg(m) {
    if (m.kind === 'binary') {
        scene.addThumbnail(m.header.index, m.payload, m.header.ts);
    } else if (m.kind === 'global_map') {
        scene.setGlobalMap(m.payload, m.header);
    } else if (m.kind === 'summary') {
        scene.setExpectedSize(m.n);
    } else if (m.kind === 'frame_meta_batch') {
        scene.setFrameMetaBatch(m.entries);
    } else if (m.kind === 'active_index') {
        scene.setActiveIndex(m.index);
    } else if (m.kind === 'clear_focus') {
        scene.clearFocus();
    }
}

function flushSceneMsgs() {
    if (!scene) return;
    while (pendingSceneMsgs.length) applySceneMsg(pendingSceneMsgs.shift());
}

function handleBinary(buffer) {
    const { msgType, header, payload } = decodeBinary(buffer);
    if (msgType === MSG_THUMBNAIL) {
        if (scene) scene.addThumbnail(header.index, payload, header.ts);
        else pendingSceneMsgs.push({ kind: 'binary', header, payload });
    } else if (msgType === MSG_GLOBAL_MAP) {
        if (scene) scene.setGlobalMap(payload, header);
        else pendingSceneMsgs.push({ kind: 'global_map', header, payload });
    } else {
        log(`unknown bin type ${msgType}`);
    }
}

function handleControl(msg) {
    if (!msg) return;
    switch (msg.type) {
        case 'timeline_summary':
            log(`timeline ${msg.n} frames span ${(msg.t_end - msg.t_start).toFixed(1)}s`);
            if (scene) scene.setExpectedSize(msg.n);
            else pendingSceneMsgs.push({ kind: 'summary', n: msg.n });
            break;
        case 'frame_meta_batch':
            if (scene) scene.setFrameMetaBatch(msg.entries);
            else pendingSceneMsgs.push({ kind: 'frame_meta_batch', entries: msg.entries });
            break;
        case 'active_index':
            if (scene) scene.setActiveIndex(msg.index);
            else pendingSceneMsgs.push({ kind: 'active_index', index: msg.index });
            break;
        case 'clear_focus':
            if (scene) scene.clearFocus();
            else pendingSceneMsgs.push({ kind: 'clear_focus' });
            break;
        case 'ready':
            setStatus('Ribbon loaded — engage right grip to scrub');
            diag('server_ready', { pending: pendingSceneMsgs.length });
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

function sendGesture(g) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(encodeText(g.type, g));
    }
}

// ---- VR --------------------------------------------------------------------

async function startVR() {
    if (!RibbonScene) {
        throw new Error('Scene module failed to load — Three.js import did not resolve');
    }

    // Construct scene first so it owns its canvas/context. Then request the
    // XR session, then hand the session to the scene.
    try {
        scene = new RibbonScene(diag);
        diag('scene_constructed');
        flushSceneMsgs();
        diag('scene_msgs_flushed');
    } catch (e) {
        diag('scene_construct_failed', { error: String(e.message || e) });
        throw e;
    }

    let session;
    let mode = 'immersive-ar';
    try {
        session = await navigator.xr.requestSession('immersive-ar', {
            requiredFeatures: ['local-floor'],
            optionalFeatures: ['hand-tracking'],
        });
    } catch (e) {
        diag('ar_failed', { error: String(e.message || e) });
        mode = 'immersive-vr';
        session = await navigator.xr.requestSession('immersive-vr', {
            requiredFeatures: ['local-floor'],
            optionalFeatures: ['hand-tracking'],
        });
    }
    diag('xr_session_started', { mode });
    xrSession = session;

    input = new InputAdapter(sendGesture);

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
        if (frameCount % 120 === 0) diag('frame_tick', { count: frameCount });
        if (input && frame) input.onFrame(frame, xrRefSpace, performance.now());
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

// ---- Capability check on load ---------------------------------------------

window.addEventListener('load', async () => {
    if (!navigator.xr) {
        setStatus('WebXR not available in this browser');
        connectBtn.disabled = true;
        return;
    }
    try {
        const ar = await navigator.xr.isSessionSupported('immersive-ar').catch(() => false);
        const vr = await navigator.xr.isSessionSupported('immersive-vr').catch(() => false);
        if (!ar && !vr) {
            setStatus('VR/AR not supported on this device');
            connectBtn.disabled = true;
        }
    } catch (e) {
        log(`xr check failed: ${e.message || e}`);
    }
});
