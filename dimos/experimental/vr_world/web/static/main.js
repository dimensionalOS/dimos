// VR World live entry point. Opens /ws_vr_world, feeds live messages to
// WorldScene, and routes input gestures: left stick -> robot drive (to server),
// view nav -> scene (client-side).

import { InputAdapter } from '/static_vw/input_adapter.js';
import {
    MSG_CAMERA,
    MSG_VOXEL_MAP,
    decodeBinary,
    decodeText,
    encodeText,
} from '/static_vw/protocol.js';

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
    console.log(`[diag] ${event}`, fields);
    log(`[diag] ${event}`);
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(encodeText('diag', { event, ...fields }));
    else pendingDiag.push({ event, fields });
}

function flushPendingDiag() {
    while (pendingDiag.length && ws && ws.readyState === WebSocket.OPEN) {
        const { event, fields } = pendingDiag.shift();
        ws.send(encodeText('diag', { event, ...fields }));
    }
}

diag('module_load');

try {
    const mod = await import('/static_vw/scene.js');
    WorldScene = mod.WorldScene;
    diag('scene_module_loaded');
} catch (err) {
    diag('scene_module_failed', { error: String(err && err.message || err) });
}

function setStatus(msg) { statusEl.textContent = msg; }

window.onerror = (msg, url, line, col) => {
    setStatus(`Error: ${msg}`);
    diag('window_error', { msg: String(msg), line, col });
};
window.addEventListener('unhandledrejection', (e) => {
    diag('unhandled_rejection', { reason: String(e.reason && e.reason.message || e.reason) });
});

// ---- WebSocket -------------------------------------------------------------

function setupWebSocket() {
    return new Promise((resolve, reject) => {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws_vr_world`;
        setStatus('Connecting to server…');
        ws = new WebSocket(wsUrl);
        ws.binaryType = 'arraybuffer';
        ws.onopen = () => { setStatus('Server connected — starting VR…'); flushPendingDiag(); diag('ws_open'); resolve(); };
        ws.onerror = (e) => { setStatus('WebSocket error'); reject(e); };
        ws.onclose = () => { log('ws closed'); setStatus('Disconnected'); };
        ws.onmessage = (event) => {
            if (typeof event.data === 'string') handleControl(decodeText(event.data));
            else handleBinary(event.data);
        };
    });
}

function handleBinary(buffer) {
    if (!scene) return;
    const { msgType, header, payload } = decodeBinary(buffer);
    if (msgType === MSG_VOXEL_MAP) scene.setVoxelMap(header, payload);
    else if (msgType === MSG_CAMERA) scene.setCameraFrame(payload);
}

function handleControl(msg) {
    if (!msg) return;
    switch (msg.type) {
        case 'robot_pose': if (scene) scene.setRobotPose(msg.pose); break;
        case 'pong': break;
        default: log(`ctrl ${msg.type}`);
    }
}

// Input dispatch: drive goes to server; everything else is client-side view nav.
function dispatchGesture(g) {
    switch (g.type) {
        case 'drive':
            if (ws && ws.readyState === WebSocket.OPEN) ws.send(encodeText('drive', { x: g.x, yaw: g.yaw }));
            break;
        case 'yaw': if (scene) scene.applyYaw(g); break;
        case 'teleport_aim': if (scene) scene.setTeleportAim(g); break;
        case 'teleport_commit': if (scene) scene.applyTeleportCommit(g); break;
        case 'teleport_cancel': if (scene) scene.clearTeleportAim(); break;
        case 'scale_delta': if (scene) scene.applyScale(g); break;
        case 'reset_view': if (scene) scene.resetView(); break;
        case 'toggle_render': if (scene) scene.toggleRenderMode(); break;
        default: break;
    }
    if (g.type !== 'drive' && g.type !== 'yaw' && g.type !== 'teleport_aim') {
        if (ws && ws.readyState === WebSocket.OPEN) ws.send(encodeText(g.type, g));
    }
}

// ---- VR --------------------------------------------------------------------

async function startVR() {
    if (!WorldScene) throw new Error('Scene module failed to load');
    scene = new WorldScene(diag);
    diag('scene_constructed');

    let session;
    let mode = 'immersive-vr';
    try {
        session = await navigator.xr.requestSession('immersive-vr', {
            requiredFeatures: ['local-floor'], optionalFeatures: ['hand-tracking'],
        });
    } catch (e) {
        mode = 'immersive-ar';
        session = await navigator.xr.requestSession('immersive-ar', {
            requiredFeatures: ['local-floor'], optionalFeatures: ['hand-tracking'],
        });
    }
    diag('xr_session_started', { mode });
    xrSession = session;
    input = new InputAdapter(dispatchGesture);
    xrRefSpace = await session.requestReferenceSpace('local-floor');
    session.addEventListener('end', () => { diag('xr_session_ended'); xrSession = null; disconnect(); });

    let frameCount = 0;
    await scene.setSession(session, (frame) => {
        frameCount++;
        if (frameCount === 1) diag('first_frame');
        if (frameCount % 240 === 0) diag('frame_tick', { count: frameCount });
        if (input && frame) input.onFrame(frame, xrRefSpace, performance.now(), scene);
    });
    setStatus(`VR active (${mode}) — left stick drives the robot`);
}

async function connect() {
    try {
        connectBtn.disabled = true;
        if (!navigator.xr) throw new Error('WebXR unavailable. Use the Quest browser.');
        await setupWebSocket();
        await startVR();
        connectBtn.classList.add('hidden');
        disconnectBtn.classList.remove('hidden');
    } catch (e) {
        setStatus(`Connection failed: ${e.message || e}`);
        connectBtn.disabled = false;
    }
}

async function disconnect() {
    setStatus('Disconnecting…');
    if (xrSession) { try { await xrSession.end(); } catch (_) {} xrSession = null; }
    if (ws) { try { ws.close(); } catch (_) {} ws = null; }
    connectBtn.classList.remove('hidden');
    connectBtn.disabled = false;
    disconnectBtn.classList.add('hidden');
    setStatus('Disconnected');
}

window.app = { connect, disconnect, diag };

window.addEventListener('load', async () => {
    if (!navigator.xr) { setStatus('WebXR not available'); connectBtn.disabled = true; return; }
    const vr = await navigator.xr.isSessionSupported('immersive-vr').catch(() => false);
    const ar = await navigator.xr.isSessionSupported('immersive-ar').catch(() => false);
    if (!vr && !ar) { setStatus('VR/AR not supported'); connectBtn.disabled = true; }
});
