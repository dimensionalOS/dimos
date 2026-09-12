// Memory World browser entry point:
//   - opens <client route>/ws
//   - hands payloads to WorldScene (point cloud + Street-View markers)
//   - routes gestures from InputAdapter directly to the scene (locomotion
//     is client-side state; server doesn't need a copy)

import { InputAdapter } from '/static_mw/input_adapter.js';
import {
    MSG_IMAGE_POSES,
    MSG_IMAGE_THUMBNAIL,
    MSG_ODOM_TRAIL,
    MSG_POINT_CLOUD,
    MSG_HEATMAP,
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
const orbitBtn = document.getElementById('orbitBtn');
const embedBtn = document.getElementById('embedBtn');
const prepareBtn = document.getElementById('prepareBtn');
const searchNote = document.getElementById('searchNote');
const askForm = document.getElementById('askBar');
const askInput = document.getElementById('askInput');
const askBtn = document.getElementById('askBtn');
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
const embeddingsUrl = `${window.location.pathname.replace(/\/$/, '')}/embeddings`;
const baseUrl = window.location.pathname.replace(/\/$/, '');

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

// The page loads this module with a ?v= stamp; the scene gets the same one so
// the two can never come from different versions of the cache.
const assetVersion = new URL(import.meta.url).search;

let ReplayController = null;
let replay = null;
let HeatmapLayer = null;
let PyramidLayer = null;
let Flight = null;
let ResultsNav = null;
let Tour = null;
let heatmap = null;
let pyramids = null;
let flight = null;
let results = null;
let tour = null;
let pendingPyramids = null;
let voxelStyle = null;
// Per-frame work hung off the scene's tick: flights, replay, the tour.
let tickers = [];

try {
    const mod = await import(`/static_mw/scene.js${assetVersion}`);
    WorldScene = mod.WorldScene;
    ReplayController = (await import(`/static_mw/replay.js${assetVersion}`)).ReplayController;
    HeatmapLayer = (await import(`/static_mw/heatmap.js${assetVersion}`)).HeatmapLayer;
    PyramidLayer = (await import(`/static_mw/pyramids.js${assetVersion}`)).PyramidLayer;
    Flight = (await import(`/static_mw/flight.js${assetVersion}`)).Flight;
    ResultsNav = (await import(`/static_mw/results.js${assetVersion}`)).ResultsNav;
    Tour = (await import(`/static_mw/tour.js${assetVersion}`)).Tour;
    voxelStyle = (await import('/static_mw/voxel_sprites.js')).voxelStyle;  // same instance as scene.js
    diag('scene_module_loaded');
} catch (err) {
    diag('scene_module_failed', { error: String(err && err.message || err) });
}

function setStatus(msg, isError = false) {
    statusEl.textContent = msg;
    statusEl.classList.toggle('error', Boolean(isError));
}

window.onerror = (msg, url, line, col, err) => {
    console.error(`[err] ${msg} at ${url}:${line}:${col}`, err);
    setStatus(`Error: ${msg}`, true);
    diag('window_error', { msg: String(msg), url: String(url), line, col });
};
window.addEventListener('unhandledrejection', (e) => {
    diag('unhandled_rejection', { reason: String(e.reason && e.reason.message || e.reason) });
});

// ---- WebSocket -------------------------------------------------------------

function setupWebSocket() {
    return new Promise((resolve, reject) => {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        // Derived from the page's own path, not a literal: the server registers the
        // socket under the client route, so a moved viewer keeps working. A separate
        // ws_route setting used to exist on the Python side and this file ignored it,
        // which made it a knob that could only break things.
        const wsUrl = `${protocol}//${window.location.host}${baseUrl}/ws`;
        setStatus('Connecting to server…');
        const socket = new WebSocket(wsUrl);
        ws = socket;
        ws.binaryType = 'arraybuffer';

        // Settled once: a socket that closes after it opened must not reject the
        // promise connect() already moved past, and one that closes without ever
        // opening must not leave connect() awaiting a promise nothing will settle.
        let settled = false;
        ws.onopen = () => {
            setStatus('Server connected — starting VR…');
            flushPendingDiag();
            diag('ws_open');
            settled = true;
            resolve(socket);   // the caller cleans up THIS socket, never whatever `ws` is by then
        };
        ws.onerror = (e) => {
            if (ws !== socket) return;  // a stale socket's error must not touch the live status
            console.error('[ws] error', e);
            setStatus('WebSocket error');
            settled = true;
            reject(e);
        };
        ws.onclose = () => {
            log('ws closed');
            if (!settled) {
                // Closed before it ever opened. A failed handshake fires error first, so
                // this is normally already settled; without it a close that arrives alone
                // would hang connect() forever with the button disabled.
                settled = true;
                reject(new Error('the websocket closed before it opened'));
                return;
            }
            if (ws === socket) void disconnect();  // dropped by the server: same teardown
        };
        ws.onmessage = (event) => {
            if (ws !== socket) return;  // a frame in flight when the session was torn down
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
    else if (m.kind === 'heatmap' && heatmap) heatmap.set(m.header, m.payload);
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
    else if (msgType === MSG_HEATMAP) kind = 'heatmap';
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
            syncLayerBoxes();  // the cloud, photos and minimap exist now
            setStatus('World loaded — left stick walks, pinch both hands to scale');
            diag('server_ready');
            break;
        case 'query_result':
            if (scene) scene.setQueryResult(msg);
            else pendingQueryResult = msg;
            if (results) results.setResult(msg);
            // The answer has its own element now (`#answerText`, set by scene.onAnswerText),
            // which persists; the status line is transient and gets overwritten by the next
            // thing that happens. Printing the answer in both showed it twice on screen.
            setStatus('Answer ready — places highlighted');
            askBtn.disabled = false;
            break;
        case 'query_pyramids':
            if (pyramids) pyramids.set(msg.pyramids);
            else pendingPyramids = msg.pyramids;
            break;
        case 'route':
            if (results) results.setRoute(msg);
            break;
        case 'search_status':
            applySearchStatus(msg);
            break;
        case 'voice_transcript':
            setStatus(`Heard: “${msg.text}” — searching…`);
            if (scene) scene.setHeardText(msg.text);
            break;
        case 'index_status':
            applyIndexStatus(msg);
            break;
        case 'status':
            setStatus(msg.message);
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
        tickers = [];
        scene.onTick = (dt) => { for (const tick of tickers) tick(dt); };
        scene.onJump = () => jumpToAnswer();  // the J key; the Answer button calls it directly
        // The answer as real text on the page. `null` means an immersive XR session is
        // showing the drawn panel instead, because the DOM is not composited into one.
        scene.onAnswerText = (text) => {
            const el = document.getElementById('answerText');
            if (!el) return;
            el.textContent = text || '';
            el.hidden = !text;
        };
        heatmap = HeatmapLayer ? new HeatmapLayer(scene._frameRotate) : null;
        pyramids = PyramidLayer ? new PyramidLayer(scene._frameRotate) : null;
        flight = Flight ? new Flight(scene) : null;
        if (flight) tickers.push((dt) => flight.tick(dt));
        if (heatmap) tickers.push((dt) => heatmap.tick(dt));
        syncLayerBoxes();  // the layers are new objects; the boxes kept their state
        results = ResultsNav ? new ResultsNav({
            scene, heatmap, pyramids, flight, baseUrl, diag,
            ui: {
                bar: document.getElementById('results'),
                prevBtn: document.getElementById('resultsPrev'),
                nextBtn: document.getElementById('resultsNext'),
                counter: document.getElementById('resultsCounter'),
                label: document.getElementById('resultsLabel'),
                orbitBtn: document.getElementById('resultsOrbit'),
                navigateBtn: document.getElementById('resultsNavigate'),
                closeBtn: document.getElementById('resultsClose'),
                status: statusEl,
            },
        }) : null;
        // Either orbit button may be the one pressed; both say the same thing after.
        if (results) results.onOrbitChange = () => syncOrbitLabels();
        scene.onOrbitChange = () => syncOrbitLabels();  // the O key and camera flights too
        tour = Tour ? new Tour({
            scene, heatmap, pyramids, flight, results, baseUrl, diag,
            replay: () => replay,
            ask: (text) => ask(text),
            ui: {
                panel: document.getElementById('tour'),
                title: document.getElementById('tourTitle'),
                body: document.getElementById('tourBody'),
                kicker: document.getElementById('tourKicker'),
                dots: document.getElementById('tourDots'),
                prevBtn: document.getElementById('tourPrev'),
                nextBtn: document.getElementById('tourNext'),
                exitBtn: document.getElementById('tourExit'),
            },
        }) : null;
        if (tour) tickers.push((dt) => tour.tick(dt));
        flushSceneMsgs();
        if (pendingQueryResult) {
            scene.setQueryResult(pendingQueryResult);
            if (results) results.setResult(pendingQueryResult);
            pendingQueryResult = null;
        }
        if (pendingPyramids && pyramids) {
            pyramids.set(pendingPyramids);
            pendingPyramids = null;
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

// The replay index is served once the server has built (or found) the diff
// streams in the recording; until then it answers 503, so keep asking.
async function startReplay() {
    if (!ReplayController || !scene) return;
    replay = new ReplayController({
        scene,
        baseUrl: window.location.pathname.replace(/\/$/, ''),
        diag,
        ui: {
            bar: document.getElementById('timeline'),
            scrub: document.getElementById('scrub'),
            playBtn: document.getElementById('playBtn'),
            timeLabel: document.getElementById('timeLabel'),
            exitBtn: document.getElementById('exitReplayBtn'),
        },
    });
    const owner = scene;
    const mine = replay;
    tickers.push(() => { if (replay === mine) mine.tick(); });
    scene.onQualityChange = () => replay && replay.refill();
    scene.onLayerChange = syncBoxesFromScene;
    // Polled while the server builds the replay streams (up to half an hour on a long
    // recording); a build the server remembers as failed will not change, so stop then.
    // "not started" usually will not change -- with build_replay_on_start off, or the
    // voxel streams deleted, nothing ever asks for a build -- and polling it every five
    // seconds left a 503 in the console for as long as the page stayed open. So it backs
    // OFF rather than giving up: "not started" is also what a worker blocked on the store
    // lock reports before it reaches "building", and a recording whose first scan takes a
    // minute to reach would have had its timeline declared missing while it was on its way.
    const IDLE_ATTEMPTS = 12;   // ~1 min of five-second tries before slowing down
    const SLOW_MS = 60000;
    let idle = 0;
    for (let attempt = 0; scene === owner; attempt++) {
        try {
            const index = await replay.load();
            const orbit = index.orbit;
            if (orbit && orbit.positions && orbit.positions.length) {
                orbitBtn.textContent = `Orbit ${orbit.frame}`;
                scene.setOrbitTarget(orbit.positions[orbit.positions.length - 1]);
                replay.onScan = (scan) => scene.setOrbitTarget(orbit.positions[scan]);
            }
            return;
        } catch (e) {
            const reason = String(e.message || e);
            if (attempt === 0) diag('replay_waiting', { error: reason });
            if (reason.startsWith('replay build failed')) {
                setStatus(`Timeline unavailable: ${reason}`);
                return;
            }
            // A settled answer, not progress: this recording is not going to get one.
            if (reason.includes('turned off')) {
                diag('replay_turned_off', { error: reason });
                setStatus('This recording has no timeline');
                return;
            }
            idle = reason.includes('not started') ? idle + 1 : 0;
            const slow = idle >= IDLE_ATTEMPTS;
            if (slow && idle === IDLE_ATTEMPTS) {   // say it once, on the way down
                diag('replay_backing_off', { error: reason, attempts: attempt + 1 });
                setStatus('No timeline for this recording yet; still checking');
            }
            await new Promise((resolve) => setTimeout(resolve, slow ? SLOW_MS : 5000));
        }
    }
}

/** Enter VR when a headset is present, otherwise fall back to the flat viewer. */
async function startViewer() {
    buildScene();
    startPerfReadout();
    void startReplay();
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
    // `connected` gates the phone's chrome: the status line is worth reading while the
    // world is still coming up and noise once it is there.
    document.body.classList.add('connected');
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
        setStatus(result.detail || 'Answer ready');  // #answerText carries the answer
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
    // targetTouches: another finger may be looking around on the canvas.
    const touch = event.targetTouches[0];
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

const hudBtn = document.getElementById('hudBtn');
hudBtn.addEventListener('click', () => {
    if (scene) hudBtn.textContent = scene.toggleHud() ? 'Hide map' : 'Show map';
});
// The minimap starts hidden, so the button starts as the way to get it back.
hudBtn.textContent = 'Show map';
// Take me to the answer: the best photo of the place BEING BROWSED, else that place's
// marker, else the focus point. The J key and this button both used to ask for index 0 of
// the UNFILTERED photo list and fall back to _lastResultPoints[0], so after stepping to
// another place they walked you toward place 0 while place 0's evidence was hidden -- and
// on an answer carrying a focus point but no points they did nothing at all.
function jumpToAnswer() {
    if (!scene) return false;
    const here = scene.queryImagesHere();
    if (here.length && scene.viewFrom(here[0])) return true;
    const points = scene._lastResultPoints || [];
    const at = scene.clusterFilter >= 0 ? scene.clusterFilter : 0;
    const point = points[at] || points[0];
    if (point) { scene.focusOn(point.position); return true; }
    if (scene._focusPoint) { scene.focusOn(scene._focusPoint); return true; }
    return false;
}
document.getElementById('answerBtn').addEventListener('click', () => jumpToAnswer());

// ---- typed questions ---------------------------------------------------------

async function ask(text) {
    text = (text || '').trim();
    if (!text) return null;
    const session = ws;  // the answer belongs to this connection only, and there must be one
    if (!session) return null;
    askBtn.disabled = true;
    // The previous answer stops being the answer the moment another question is asked --
    // not when a new result arrives, because a question that FAILS never brings one and
    // its predecessor stayed on screen. Through the nav, which owns the heat map, the
    // pyramids and the route as well as the scene: clearing only the scene left those.
    if (results) results.clear();
    setStatus(`Asking: ${text}`);
    diag('ask', { text });
    try {
        const response = await fetch(`${baseUrl}/ask`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ text }),
        });
        const body = await response.json();
        if (ws !== session) return null;  // answered after a disconnect: not our status line
        if (!response.ok) throw new Error(body.detail || response.status);
        setStatus(body.answer ? 'Answer ready' : 'No answer');
        return body;
    } catch (e) {
        if (ws === session) setStatus(`Question failed: ${e.message || e}`);
        return null;
    } finally {
        if (ws === session) askBtn.disabled = false;
    }
}

askForm.addEventListener('submit', (event) => {
    event.preventDefault();
    void ask(askInput.value);
    askInput.blur();
});

// ---- search status (Hyperspace) --------------------------------------------

let searchStatus = { engine: null, ready: false, memory_db_present: false, prepare: { state: 'idle', progress: '' } };
let preparePoll = null;
let lastPrepareFailure = null;

function applySearchStatus(status) {
    searchStatus = status || searchStatus;
    const connected = !!ws;
    const ready = !!searchStatus.ready;
    const preparing = searchStatus.prepare && searchStatus.prepare.state === 'running';
    const failed = searchStatus.prepare && searchStatus.prepare.state === 'failed';
    if (ready) {
        searchNote.textContent = `Search: Hyperspace · ${searchStatus.keyframes} keyframes`
            + (searchStatus.segments ? ` · ${searchStatus.segments} segments` : '');
    } else if (preparing) {
        searchNote.textContent = `Preparing search… ${(searchStatus.prepare.progress || '').slice(0, 80)}`;
    } else if (failed) {
        // The server works out why (a refused claim, a missing stream); saying so beats
        // a bare "retry" the user can only repeat.
        searchNote.textContent = `Prepare search failed: ${(searchStatus.prepare.progress || 'no reason given').slice(0, 120)}`;
    } else if (searchStatus.error) {
        // A db that exists and failed to load is not one still loading. Saying
        // "loading Hyperspace…" for ever hides a reason the server already worked out
        // (a model mismatch, a corrupt db) and leaves the user nothing to act on.
        searchNote.textContent = `Search unavailable: ${String(searchStatus.error).slice(0, 160)}`;
    } else if (searchStatus.memory_db_present) {
        searchNote.textContent = 'Search: loading Hyperspace…';
    } else {
        searchNote.textContent = searchStatus.error
            ? `Search unavailable: ${searchStatus.error}`
            : 'This recording has no Hyperspace embeddings yet.';
    }
    // A db that failed to load still needs rebuilding, so the button that rebuilds it
    // has to be reachable: hiding it leaves the reason on screen and no way to act.
    const usable = searchStatus.memory_db_present && !searchStatus.error;
    prepareBtn.classList.toggle('hidden', !connected || ready || usable);
    prepareBtn.disabled = preparing;
    prepareBtn.textContent = preparing ? 'Preparing…' : (failed ? 'Prepare search failed — retry' : 'Prepare search (embed this recording)');
    if (failed && searchStatus.prepare.progress !== lastPrepareFailure) {
        lastPrepareFailure = searchStatus.prepare.progress;  // said once, not on every poll
        setStatus(`Prepare search failed: ${searchStatus.prepare.progress}`);
    }
    if (!failed) lastPrepareFailure = null;
    if (connected && preparing && !preparePoll) preparePoll = setInterval(pollSearchStatus, 3000);
    if (!preparing && preparePoll) { clearInterval(preparePoll); preparePoll = null; }
    applyAskAvailability();
}

/** Whether you can ask at all, which depends on BOTH statuses, so both handlers end here.
 *
 *  Hyperspace being ready is one way; a SigLIP frame index present is the other. Only the
 *  search handler used to set this, so finishing "Add embeddings" -- which arrives as an
 *  index status and nothing else -- showed the microphone and left the ask box disabled
 *  for the rest of the session.
 */
function applyAskAvailability() {
    const connected = !!ws;
    const ready = !!searchStatus.ready;
    const canAsk = ready || !!indexStatus.present;
    askInput.disabled = !connected || !canAsk;
    askInput.placeholder = ready ? 'Ask the recording, e.g. where did I see a chair'
        : (indexStatus.present ? 'Ask (SigLIP frame search)' : 'Search not ready — see the menu');
    micBtn.classList.toggle('hidden', !connected || !canAsk);
}

async function pollSearchStatus() {
    try {
        const response = await fetch(`${baseUrl}/search/status`);
        if (response.ok) applySearchStatus(await response.json());
    } catch (e) {
        log(`search status failed: ${e.message || e}`);
    }
}

prepareBtn.addEventListener('click', async () => {
    prepareBtn.disabled = true;
    setStatus('Embedding the recording for Hyperspace — minutes on a long recording');
    try {
        const response = await fetch(`${baseUrl}/search/prepare`, { method: 'POST' });
        if (!response.ok) throw new Error(`${response.status} ${await response.text()}`);
        applySearchStatus(await response.json());
    } catch (e) {
        setStatus(`Could not start: ${e.message || e}`);
        prepareBtn.disabled = false;
    }
});

// ---- menu --------------------------------------------------------------------

const menuEl = document.getElementById('menu');
const menuBtn = document.getElementById('menuBtn');
menuBtn.addEventListener('click', () => menuEl.classList.toggle('open'));
document.getElementById('tourBtn').addEventListener('click', () => { menuEl.classList.remove('open'); tour && tour.start(); });
document.getElementById('menuPrevBtn').addEventListener('click', () => results && results.prev());
document.getElementById('menuNextBtn').addEventListener('click', () => results && results.next());
document.getElementById('menuOrbitResultBtn').addEventListener('click', () => results && results.orbitCurrent());
document.getElementById('menuNavigateBtn').addEventListener('click', () => results && results.navigate());
document.getElementById('menuOrbitBtn').addEventListener('click', () => setOrbit(!scene?.isOrbiting()));
// The phone hides the on-screen Disconnect, so the menu has to carry one or there is no
// way back from a connected session on a device with no keyboard.
document.getElementById('menuDisconnectBtn').addEventListener('click', () => {
    document.body.classList.remove('connected');
    window.app.disconnect();
});
const layerBoxes = {
    heat: document.getElementById('layerHeat'),
    pyramids: document.getElementById('layerPyramids'),
    voxels: document.getElementById('layerVoxels'),
    photos: document.getElementById('layerPhotos'),
    hud: document.getElementById('layerHud'),
};
layerBoxes.heat.addEventListener('change', () => heatmap && heatmap.setVisible(layerBoxes.heat.checked));
layerBoxes.pyramids.addEventListener('change', () => pyramids && pyramids.setVisible(layerBoxes.pyramids.checked));
layerBoxes.voxels.addEventListener('change', () => scene && scene._cloudWanted !== layerBoxes.voxels.checked && scene.toggleCloud());
layerBoxes.photos.addEventListener('change', () => scene && scene._imageQuadGroup.visible !== layerBoxes.photos.checked && scene.toggleImages());
layerBoxes.hud.addEventListener('change', () => {
    if (scene && scene._hudPanel.visible !== layerBoxes.hud.checked) hudBtn.textContent = scene.toggleHud() ? 'Hide map' : 'Show map';
});

/** The scene changed a layer itself (a key, the tour): the boxes and the map button follow. */
function syncBoxesFromScene() {
    if (!scene) return;
    layerBoxes.voxels.checked = scene._cloudWanted;
    layerBoxes.photos.checked = scene._imageQuadGroup.visible;
    layerBoxes.hud.checked = scene._hudPanel.visible;
    hudBtn.textContent = scene._hudPanel.visible ? 'Hide map' : 'Show map';
}

/** Apply the boxes to the current scene: they keep their state across a reconnect, the scene does not. */
function syncLayerBoxes() {
    const wanted = { heat: layerBoxes.heat.checked, pyramids: layerBoxes.pyramids.checked,
        voxels: layerBoxes.voxels.checked, photos: layerBoxes.photos.checked, hud: layerBoxes.hud.checked };
    if (heatmap) heatmap.setVisible(wanted.heat);
    if (pyramids) pyramids.setVisible(wanted.pyramids);
    if (!scene) return;
    // Each toggle writes the boxes back; the snapshot keeps the later ones honest.
    if (scene._cloudWanted !== wanted.voxels) scene.toggleCloud();
    if (scene._imageQuadGroup && scene._imageQuadGroup.visible !== wanted.photos) scene.toggleImages();
    // The boxes survive a reconnect and the scene does not, so the user's choice has to be
    // restored whether or not the toggle above ran. It does not run when the box already
    // matches a fresh scene's default, which is exactly the "Photos off" case, and an
    // answer would then have switched them back on again after every reconnect.
    scene._photosPinnedOff = !wanted.photos;
    scene._hudOff = !wanted.hud;  // same reason: the toggle below does not always run
    if (scene._hudPanel && scene._hudPanel.visible !== wanted.hud) {
        hudBtn.textContent = scene.toggleHud() ? 'Hide map' : 'Show map';
    }
}
const cubesBox = document.getElementById('layerCubes');
function setVoxelStyle(cubes) {
    if (voxelStyle) voxelStyle.value = cubes ? 1 : 0;
    cubesBox.checked = !!cubes;
    try { localStorage.setItem('memworld.cubes', cubes ? '1' : '0'); } catch (_) { /* private mode */ }
    diag('voxel_style', { cubes: !!cubes });
}
cubesBox.addEventListener('change', () => setVoxelStyle(cubesBox.checked));
try { if (localStorage.getItem('memworld.cubes') === '1') setVoxelStyle(true); } catch (_) { /* ignore */ }

// Orbit any tf frame: the server lists them and gives a frame's position per replay scan.
const orbitFrameSel = document.getElementById('orbitFrameSel');
let orbitPositions = null;

async function loadFrames() {
    try {
        const response = await fetch(`${baseUrl}/frames`);
        if (!response.ok) return;
        const body = await response.json();
        orbitFrameSel.innerHTML = '';
        for (const name of body.frames || []) {
            const option = document.createElement('option');
            option.value = name;
            option.textContent = name;
            option.selected = name === body.default;
            orbitFrameSel.appendChild(option);
        }
    } catch (e) {
        log(`frames failed: ${e.message || e}`);
    }
}

async function setOrbitFrame(frame) {
    try {
        const response = await fetch(`${baseUrl}/orbit?frame=${encodeURIComponent(frame)}`);
        if (!response.ok) throw new Error(`${response.status}`);
        const body = await response.json();
        orbitPositions = body.positions || [];
        if (orbitPositions.length && scene) {
            // `??` guarded the wrong sentinel: ReplayController starts at scan = -1, not
            // null, so this asked for orbitPositions[-1], got undefined, and setOrbitTarget
            // returned without doing anything -- picking an orbit frame before touching the
            // timeline was a silent no-op. Before the first scan, orbit the last pose.
            const shown = replay && replay.scan >= 0 ? replay.scan : orbitPositions.length - 1;
            const at = Math.min(shown, orbitPositions.length - 1);
            scene.setOrbitTarget(orbitPositions[at]);
            if (replay) replay.onScan = (scan) => scene.setOrbitTarget(orbitPositions[Math.min(scan, orbitPositions.length - 1)]);
        }
        orbitBtn.textContent = scene?.isOrbiting() ? 'Stop orbit' : `Orbit ${frame}`;
        diag('orbit_frame', { frame, positions: orbitPositions.length });
        return orbitPositions;
    } catch (e) {
        setStatus(`No positions for ${frame}: ${e.message || e}`);
        return null;
    }
}
orbitFrameSel.addEventListener('change', () => setOrbitFrame(orbitFrameSel.value));

// Keys: arrows step through places, N routes, T starts the tour, Esc closes things.
window.addEventListener('keydown', (event) => {
    const typing = event.target && (event.target.tagName === 'INPUT' || event.target.tagName === 'SELECT' || event.target.tagName === 'TEXTAREA');
    if (event.code === 'Escape') {
        if (tour && tour.active) tour.exit();
        menuEl.classList.remove('open');
        if (typing) event.target.blur();
        return;
    }
    if (typing) return;
    if (event.code === 'Slash') { event.preventDefault(); askInput.focus(); return; }
    if (tour && tour.active) {
        // Space always advances the tour. The arrows do too -- EXCEPT on a hands-on
        // station, whose whole text is "the controls are back": it prints
        // "arrows places . N route . O orbit . P camera" and then this branch swallowed
        // every one of them. On the last station, which is the hands-on one, pressing the
        // right arrow to "step to the next place" ran tour.next() off the end and quietly
        // exited the tour instead. On those stations everything falls through to the
        // normal handlers below.
        const station = tour.stations && tour.stations[tour.index];
        if (event.code === 'Space') { event.preventDefault(); tour.next(); return; }
        if (!(station && station.hands_on)) {
            if (event.code === 'ArrowRight') { event.preventDefault(); tour.next(); }
            if (event.code === 'ArrowLeft') { event.preventDefault(); tour.prev(); }
            return;
        }
    }
    if (event.code === 'ArrowRight') { event.preventDefault(); results && results.next(); }
    else if (event.code === 'ArrowLeft') { event.preventDefault(); results && results.prev(); }
    else if (event.code === 'KeyN') results && results.navigate();
    else if (event.code === 'KeyT') tour && tour.start();
});

// ---- embeddings ------------------------------------------------------------

// A recording with no SigLIP vectors cannot be searched. Instead of a dead
// "Hold to ask" the viewer offers to add them: the server runs siglipify over
// the recording and swaps the buttons when the index is up.
let indexStatus = { present: false, embedding: 'idle', progress: '' };
let embedPoll = null;
let lastIndexNote = null;  // said once, not on every three-second poll

function applyIndexStatus(status) {
    indexStatus = status || indexStatus;
    const connected = !!ws;
    const running = indexStatus.embedding === 'running';
    applyAskAvailability();
    embedBtn.classList.toggle('hidden', !connected || indexStatus.present || searchStatus.ready);
    embedBtn.disabled = running;
    if (running) {
        embedBtn.textContent = `Embedding… ${(indexStatus.progress || '').slice(0, 60)}`;
        if (connected && !embedPoll) embedPoll = setInterval(pollEmbeddings, 3000);
    } else if (/^(not started|building|loading)/.test(indexStatus.index || '')) {
        // The server builds the frame index on startup and never pushes its progress, so
        // this has to keep asking. "not started" is the window before the build begins:
        // offering "Add embeddings" there invites a second job doing the same work.
        embedBtn.textContent = `Indexing… ${indexStatus.index.slice(0, 60)}`;
        embedBtn.disabled = true;
        if (connected && !embedPoll) embedPoll = setInterval(pollEmbeddings, 3000);
    } else {
        embedBtn.textContent = indexStatus.embedding === 'failed' ? 'Embedding failed — retry' : 'Add embeddings';
        if (embedPoll) { clearInterval(embedPoll); embedPoll = null; }
        if (indexStatus.embedding === 'failed') setStatus(`Embedding failed: ${indexStatus.progress}`);
        if (indexStatus.embedding === 'done') setStatus('Embeddings added — hold to ask');
        // An index the server refused (another model, camera or world frame) reads as
        // "no embeddings" otherwise, which is a different situation with the same button
        // and sends the user to a long job that lands in the same place.
        const why = indexStatus.index || '';
        if (why && !/^(not started|no embeddings|ready|not needed)/.test(why) && why !== lastIndexNote) {
            lastIndexNote = why;
            setStatus(`Frame index: ${why.slice(0, 140)}`);
        }
        if (!why) lastIndexNote = null;
    }
}

async function pollEmbeddings() {
    try {
        const response = await fetch(embeddingsUrl);
        if (response.ok) applyIndexStatus(await response.json());
    } catch (e) {
        log(`embeddings poll failed: ${e.message || e}`);
    }
}

embedBtn.addEventListener('click', async () => {
    embedBtn.disabled = true;
    setStatus('Adding embeddings with siglipify — this takes a while on a long recording');
    try {
        const response = await fetch(embeddingsUrl, { method: 'POST' });
        if (!response.ok) throw new Error(`${response.status} ${await response.text()}`);
        applyIndexStatus(await response.json());
    } catch (e) {
        setStatus(`Could not start embedding: ${e.message || e}`);
        embedBtn.disabled = false;
    }
});

// Orbit mode circles a frame of the robot (base_link by default) and follows it
// along the timeline. The server names the frame and gives its position per scan.
/** Make every orbit button say what pressing it will do. Called for ANY change of
 *  orbit, including ones this file did not ask for -- a camera flight turns orbit off
 *  on its way to a place, and the buttons used to go on claiming it was still on. */
function syncOrbitLabels() {
    if (!scene) return;
    const on = scene.isOrbiting();
    // The frame the user picked, not the recording's default: OrbitControl keeps
    // orbiting their choice, so the button has to name it.
    const frame = orbitFrameSel.value || replay?.index?.orbit?.frame || 'frame';
    orbitBtn.textContent = on ? 'Stop orbit' : `Orbit ${frame}`;
    document.getElementById('orbitTouchBtn').textContent = on ? 'Walk' : 'Orbit';
    if (results && results._syncOrbitLabel) results._syncOrbitLabel();  // label only, no loop
}
function setOrbit(enabled) {
    if (!scene) return;
    scene.setOrbit(enabled);   // which calls back into syncOrbitLabels
}
orbitBtn.addEventListener('click', () => setOrbit(!scene?.isOrbiting()));
document.getElementById('orbitTouchBtn').addEventListener('click', () => setOrbit(!scene?.isOrbiting()));
document.getElementById('cameraBtn').addEventListener('click', () => {
    if (scene) scene.stepQueryImage();  // the same filtered step the P key takes
});

micBtn.addEventListener('pointerdown', startRecording);
micBtn.addEventListener('pointerup', stopRecording);
micBtn.addEventListener('pointerleave', stopRecording);

// ---- UI handlers -----------------------------------------------------------

async function connect() {
    let socket = null;
    try {
        connectBtn.disabled = true;
        socket = await setupWebSocket();
        // Ask for the mic now, before VR starts (an immersive session cannot
        // show the prompt), but never make the world wait on the answer: an
        // unanswered prompt would otherwise leave the canvas black forever.
        void acquireMic();
        await startViewer();
        connectBtn.classList.add('hidden');
        disconnectBtn.classList.remove('hidden');
        applyIndexStatus(indexStatus);
        applySearchStatus(searchStatus);
        void pollSearchStatus();
        void loadFrames();
        if (document.body.classList.contains('desktop-view')) orbitBtn.classList.remove('hidden');
    } catch (e) {
        // The socket may already be open: setupWebSocket resolves before startViewer
        // runs, so anything startViewer throws (no WebXR, a scene that will not build)
        // used to leave it open and registered on the server, and the next Connect
        // opened a second one beside it.
        //
        // Close OUR socket, not whatever the module-level `ws` happens to be now. A
        // dropped connection auto-disconnects and re-offers the Connect button, so a
        // second connect() can be well underway while this one is still suspended in
        // startViewer(); closing `ws` there would kill the healthy new socket and null
        // the handle out from under it, leaving the UI "connected" with nothing behind
        // it. `ws` is only cleared when it is still the one we opened.
        if (socket) {
            try { socket.close(); } catch (_) { /* ignore */ }
            if (ws === socket) ws = null;
        }
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
    if (micStream) {
        for (const track of micStream.getTracks()) track.stop();
        micStream = null;
    }
    if (scene) {
        scene.dispose();  // stops the render loop; the next connect builds a fresh one
        scene = null;
    }
    if (perfReadoutTimer) {
        clearInterval(perfReadoutTimer);
        perfReadoutTimer = null;
    }
    // The status polls run over plain HTTP; they would outlive the socket otherwise.
    if (preparePoll) { clearInterval(preparePoll); preparePoll = null; }
    if (embedPoll) { clearInterval(embedPoll); embedPoll = null; }
    pendingQueryResult = null;
    pendingPyramids = null;
    pendingSceneMsgs.length = 0;
    perfEl.style.display = 'none';
    const timeline = document.getElementById('timeline');
    timeline.hidden = true;
    timeline.classList.remove('loading', 'replaying');
    orbitBtn.textContent = 'Orbit frame';  // the next world names its frame again
    hudBtn.textContent = 'Show map';       // a fresh scene starts with the minimap hidden
    askBtn.disabled = false;               // a question in flight stops owning it
    if (replay) replay.dispose();
    replay = null;
    document.body.classList.remove('desktop-view');
    connectBtn.classList.remove('hidden');
    connectBtn.disabled = false;
    disconnectBtn.classList.add('hidden');
    applyAskAvailability();  // ws is null by now: the ask box goes dead with the mic
    embedBtn.classList.add('hidden');
    prepareBtn.classList.add('hidden');
    orbitBtn.classList.add('hidden');
    menuEl.classList.remove('open');
    if (tour && tour.active) tour.exit();
    if (tour) tour.dispose();
    if (results) results.dispose();
    // The next connect builds fresh ones; a key or a late message must not reach these.
    results = tour = heatmap = pyramids = flight = null;
    document.getElementById('results').hidden = true;
    setStatus('Disconnected');
}

window.app = {
    connect,
    disconnect,
    diag,
    // Typed question; resolves with the server's answer summary.
    ask,
    // The answer's clusters: step, orbit, route.
    results: () => results,
    go: (index) => results && results.go(index),
    next: () => results && results.next(),
    prev: () => results && results.prev(),
    navigate: () => results && results.navigate(),
    heatmap: (on = null) => { if (heatmap && on !== null) heatmap.setVisible(on); return heatmap && heatmap.visible; },
    // Draw voxels as cubes (true) or spheres (false); also in the menu, remembered per browser.
    cubes: (on = null) => { if (on !== null) setVoxelStyle(on); return !!(voxelStyle && voxelStyle.value); },
    pyramids: (on = null) => { if (pyramids && on !== null) pyramids.setVisible(on); return pyramids && pyramids.visible; },
    orbitFrame: (frame) => setOrbitFrame(frame),
    searchStatus: () => searchStatus,
    // The museum tour: start, step, exit; state for automated checks.
    tour: () => tour,
    tourStart: (station = 0) => tour && tour.start(station),
    tourState: () => tour && tour.state(),
    flying: () => !!(flight && flight.flying),
    perf: () => (scene ? scene.getPerfStats() : null),
    resetPerf: () => scene && scene.resetPerf(),
    benchmark: (frames) => (scene ? scene.benchmarkRender(frames) : null),
    // Bring the i-th answer of the last result in front of the viewer (also key J).
    jumpTo: (index = null) => (index === null
        ? jumpToAnswer()
        : scene && scene._lastResultPoints.length > index
            && !scene.viewFrom(index) && scene.focusOn(scene._lastResultPoints[index].position)),
    hud: () => scene && scene.toggleHud(),
    // Search readiness as the server last reported it, and the embed job's state.
    indexStatus: () => indexStatus,
    // Orbit the robot's frame (also key O / the Orbit button); null toggles.
    orbit: (enabled = null) => { setOrbit(enabled === null ? !scene?.isOrbiting() : enabled); return scene?.isOrbiting(); },
    // Timeline replay: seek to an absolute time / scan index, read the state, play.
    replay: () => replay,
    seek: (ts) => replay && replay.seek(ts),
    seekScan: (scan) => replay && replay.seekScan(scan),
    replayState: () => replay && replay.state(),
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
    // What is actually on screen, for checking that a layer toggle covers everything it
    // names. "Photos" means both the capture-pose markers and an answer's evidence.
    photos: () => {
        if (!scene) return null;
        const counts = { markers: 0, evidence: 0, matchRings: 0, matchLines: 0 };
        const walk = (node, shown) => {
            const visible = shown && node.visible !== false;
            if (visible && node.material && node.material.map) {
                counts[node.parent === scene._imageQuadGroup ? 'markers' : 'evidence']++;
            }
            if (visible && node.geometry && node.geometry.type === 'RingGeometry') counts.matchRings++;
            if (visible && node.type === 'Line') counts.matchLines++;
            (node.children || []).forEach((child) => walk(child, visible));
        };
        walk(scene._frameRotate, true);
        return counts;
    },
};

// H pins the desktop menu and perf readout, which otherwise fade out once
// the world is up and only return on hover.
window.addEventListener('keydown', (event) => {
    const el = event.target;
    if (el && (el.tagName === 'INPUT' || el.tagName === 'SELECT' || el.tagName === 'TEXTAREA' || el.isContentEditable)) return;
    if (event.code === 'KeyH' && document.body.classList.contains('desktop-view')) {
        document.body.classList.toggle('hud-visible');
    }
});

// The flat viewer runs anywhere, so a missing headset only changes the
// status line; Connect stays enabled. Runs now rather than on `load`: the
// top-level await above means `load` has usually fired by this point.
(async () => {
    if (navigator.maxTouchPoints > 0) document.body.classList.add('touch');
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
})();
