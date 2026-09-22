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
    MSG_QUERY_IMAGE,
    MSG_TOP_DOWN_MAP,
    decodeBinary,
    decodeText,
    encodeText,
} from '/static_mw/protocol.js';

const statusEl = document.getElementById('status');
const connectBtn = document.getElementById('connectBtn');
const orbitBtn = document.getElementById('orbitBtn');
const embedBtn = document.getElementById('embedBtn');
const searchNote = document.getElementById('searchNote');
const chatEl = document.getElementById('chat');
const chatLogEl = document.getElementById('chatLog');
const chatStateEl = document.getElementById('chatState');
const chatForm = document.getElementById('chatForm');
const chatInput = document.getElementById('chatInput');
const chatBtn = document.getElementById('chatBtn');
const chatMic = document.getElementById('chatMic');
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
// WHICH press we are on. `startRecording` awaits the permission prompt, so a press and a
// release can both happen before it returns, and so can a second press: a flag was not
// enough -- two overlapping presses both saw "still held", both built a recorder, the
// second overwrote the only reference to the first, and `stopRecording` stopped one of
// them while the other kept the microphone live. A press that is no longer the current
// one belongs to nobody. `stopRecording` moves it on, so a pending start is orphaned by
// the release as well as by the next press.
let micPress = 0;
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
let Flight = null;
let ResultsNav = null;
let Tour = null;
let flight = null;
let results = null;
let tour = null;
let voxelStyle = null;
let heightBand = null;
// Per-frame work hung off the scene's tick: flights, replay, the tour.
let tickers = [];

try {
    const mod = await import(`/static_mw/scene.js${assetVersion}`);
    WorldScene = mod.WorldScene;
    ReplayController = (await import(`/static_mw/replay.js${assetVersion}`)).ReplayController;
    Flight = (await import(`/static_mw/flight.js${assetVersion}`)).Flight;
    ResultsNav = (await import(`/static_mw/results.js${assetVersion}`)).ResultsNav;
    Tour = (await import(`/static_mw/tour.js${assetVersion}`)).Tour;
    const sprites = await import('/static_mw/voxel_sprites.js');  // same instances as scene.js
    voxelStyle = sprites.voxelStyle;
    heightBand = sprites.heightBand;
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
            syncLayerBoxes();  // the cloud, photos and minimap exist now
            syncHeightRange();  // and the cloud's own z range is known
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
            break;
        case 'route':
            if (results) results.setRoute(msg);
            break;
        case 'voice_transcript':
            setStatus(`Heard: “${msg.text}” — searching…`);
            if (scene) scene.setHeardText(msg.text);
            break;
        case 'chat':
            appendChat(msg);
            break;
        case 'chat_history':
            chatLogEl.textContent = '';
            for (const entry of msg.entries || []) appendChat(entry);
            break;
        case 'agent_idle':
            setAgentIdle(Boolean(msg.idle));
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
        // Where the scene fetches a marker's sharp frame from. Set here rather than read
        // off `window.location` inside the scene, so the one place that already computes
        // this route prefix stays the only place that computes it.
        scene.baseUrl = baseUrl;
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
        flight = Flight ? new Flight(scene) : null;
        if (flight) tickers.push((dt) => flight.tick(dt));
        syncLayerBoxes();  // the layers are new objects; the boxes kept their state
        results = ResultsNav ? new ResultsNav({
            scene, flight, baseUrl, diag,
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
            scene, flight, results, baseUrl, diag,
            replay: () => replay,
            ask: (text, span) => ask(text, span),
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
    // With a keyboard the conversation gets its own panel; a phone has no room for it,
    // and the menu's box is what says whether it is wanted at all.
    showChat(chatWanted);
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
    const press = ++micPress;
    const stream = await acquireMic();
    if (!stream) {
        setStatus('Microphone unavailable');
        return;
    }
    // Let go while the prompt was up, or another press got here first: either way this
    // one is not the press that should be recording.
    if (press !== micPress || recorder) return;
    const chunks = [];
    // Held in a local as well, because `onstop` arrives LATER -- after another press may
    // already have started a recorder of its own. Clearing `recorder` unconditionally
    // there threw the new one away.
    const mine = new MediaRecorder(stream);
    recorder = mine;
    mine.ondataavailable = (e) => e.data.size && chunks.push(e.data);
    mine.onstop = () => {
        if (recorder === mine) recorder = null;
        sendRecording(new Blob(chunks, { type: chunks[0]?.type || 'audio/webm' }));
    };
    mine.start();
    chatMic.classList.add('recording');
    setStatus('Listening…');
    diag('voice_recording_started');
}

function stopRecording() {
    micPress += 1;  // no start still waiting on a prompt belongs to a held button now
    chatMic.classList.remove('recording');
    // Let go of it HERE, not when `onstop` eventually arrives. `stop()` only queues that
    // event, so a press arriving in between found `recorder` still set and turned itself
    // away -- and the stale `onstop` then cleared the reference without starting anything.
    // Measured: press, release, press, deliver onstop -> one recorder, none recording,
    // the button dark and the status still saying "Listening...".
    const stopping = recorder;
    recorder = null;
    if (stopping && stopping.state !== 'inactive') stopping.stop();
}

async function sendRecording(blob) {
    // A tap too short for the recorder to emit a single chunk. There is nothing to send,
    // but the status still says "Listening…" from `startRecording` and the mic is already
    // off, so returning quietly left the viewer watching a line that says the world is
    // hearing them when it is not -- until some unrelated action happened to overwrite it.
    if (!blob.size) {
        setStatus('Nothing recorded — hold the button while you speak');
        diag('voice_recording_empty');
        return;
    }
    setStatus('Transcribing…');
    diag('voice_recording_sent', { bytes: blob.size, type: blob.type });
    const body = new FormData();
    body.append('audio', blob, 'query.webm');
    try {
        const response = await fetch(voiceUrl, { method: 'POST', body });
        const result = await response.json();
        diag('voice_answer', { transcript: result.transcript, success: result.success });
        // `detail` is only there when the route REFUSED (an empty recording). On the way
        // through it answers `success` and `answer`, and `answer` is the only place the
        // reason lives: nothing was said, or the search found nothing. Saying "Answer
        // ready" to all three told the viewer an answer had arrived when none had, and
        // #answerText -- which carries a real answer -- had not been written either.
        setStatus(result.detail || (result.success ? 'Answer ready' : result.answer) || 'Answer ready');
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

async function ask(text, span) {
    text = (text || '').trim();
    if (!text) return null;
    const session = ws;  // the answer belongs to this connection only, and there must be one
    if (!session) return null;
    // The previous answer stops being the answer the moment another question is asked --
    // not when a new result arrives, because a question that FAILS never brings one and
    // its predecessor stayed on screen. Through the nav, which owns the pictures and the
    // route as well as the scene: clearing only the scene left those.
    if (results) results.clear();
    setStatus(`Asking: ${text}`);
    diag('ask', { text });
    try {
        const response = await fetch(`${baseUrl}/ask`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            // `span` is [from, to] as fractions of the recording, for a question about
            // part of it. Omitted entirely when absent so the server keeps its defaults.
            body: JSON.stringify(span ? { text, from_fraction: span[0], to_fraction: span[1] } : { text }),
        });
        const body = await response.json();
        if (ws !== session) return null;  // answered after a disconnect: not our status line
        if (!response.ok) throw new Error(body.detail || response.status);
        // A refusal comes back 200 with `success: false` and its reason in `answer` -- an
        // index that holds no frames, a question that reduces to nothing. Reading only
        // `answer` put "Answer ready" on the status line for a search that never ran.
        setStatus(body.success === false ? (body.answer || 'No answer') : 'Answer ready');
        return body;
    } catch (e) {
        if (ws === session) setStatus(`Question failed: ${e.message || e}`);
        return null;
    }
}

// ---- the agent's conversation -------------------------------------------------
//
// From andrew/feat/vr_demo. The server forwards the rows the human CLI prints: what was
// asked, each tool call and its result, and the reply. Typing here does NOT go through
// `ask()` above -- that one blocks on an HTTP round trip and returns only the
// conclusion, which is what the old one-line ask bar could show. This publishes the
// question to the agent and lets every step come back on its own.

const TOOL_ARGS_CHARS = 160;

function escapeHtml(text) {
    return String(text).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

function inlineMarkdown(text) {
    return escapeHtml(text)
        .replace(/`([^`]+)`/g, '<code>$1</code>')
        .replace(/\*\*(.+?)\*\*/g, '<strong>$1</strong>')
        .replace(/(^|[^*\w])\*(?!\*)([^*]+?)\*(?![*\w])/g, '$1<em>$2</em>');
}

// The markdown a model writes, as HTML: bold, italics, code, and lists. Escaped first,
// so a recording that contains a `<script>` in a label cannot become one here.
function renderMarkdown(text) {
    const html = [];
    let list = null;
    for (const raw of String(text).split('\n')) {
        const line = raw.trimEnd();
        const bullet = /^\s*[-*]\s+(.*)$/.exec(line);
        const numbered = /^\s*\d+[.)]\s+(.*)$/.exec(line);
        const item = bullet || numbered;
        const kind = bullet ? 'ul' : 'ol';
        if (item && list !== kind) {
            if (list) html.push(`</${list}>`);
            html.push(`<${kind}>`);
            list = kind;
        } else if (!item && list) {
            html.push(`</${list}>`);
            list = null;
        }
        if (item) html.push(`<li>${inlineMarkdown(item[1])}</li>`);
        else if (line) html.push(`<div>${inlineMarkdown(line)}</div>`);
        else html.push('<div class="gap"></div>');
    }
    if (list) html.push(`</${list}>`);
    return html.join('');
}

function appendChat(entry) {
    const row = document.createElement('div');
    row.className = `msg ${entry.role}`;
    if (entry.role === 'tool_call') {
        const args = entry.args || '';
        const short = args.length > TOOL_ARGS_CHARS ? `${args.slice(0, TOOL_ARGS_CHARS)}…` : args;
        const argsEl = document.createElement('span');
        argsEl.className = 'args';
        argsEl.textContent = short;
        row.append(`▶ ${entry.name}(`, argsEl, ')');
        if (short !== args) {
            row.classList.add('expandable');
            row.title = 'Click to expand';
            row.addEventListener('click', () => {
                const open = row.classList.toggle('open');
                argsEl.textContent = open ? args : short;
            });
        }
    } else if (entry.role === 'tool_result') {
        row.textContent = `↳ ${entry.text}`;
        if (entry.ok === false) row.classList.add('failed');
    } else {
        const who = document.createElement('span');
        who.className = 'who';
        who.textContent = entry.role;
        const body = document.createElement('div');
        body.innerHTML = renderMarkdown(entry.text || '');
        row.append(who, body);
    }
    // Only follow the tail if the reader is already at it: scrolling back to read a tool
    // result must not be yanked away by the next row.
    const follow = chatLogEl.scrollTop + chatLogEl.clientHeight >= chatLogEl.scrollHeight - 24;
    chatLogEl.appendChild(row);
    if (follow) chatLogEl.scrollTop = chatLogEl.scrollHeight;
}

function setAgentIdle(idle) {
    chatEl.classList.toggle('thinking', !idle);
    chatStateEl.textContent = idle ? 'idle' : 'thinking…';
}

chatForm.addEventListener('submit', (event) => {
    event.preventDefault();
    const text = chatInput.value.trim();
    if (!text || !ws || ws.readyState !== WebSocket.OPEN) return;
    // The places and pictures on screen belong to the previous question.
    if (results) results.clear();
    ws.send(encodeText('ask', { text }));
    chatInput.value = '';
    setAgentIdle(false);
    diag('ask', { text });
});

/** Whether you can ask at all: the frame index has to hold vectors. */
function applyAskAvailability() {
    const connected = !!ws;
    const canAsk = !!indexStatus.present;
    chatInput.disabled = !connected || !canAsk;
    chatBtn.disabled = !connected || !canAsk;
    chatInput.placeholder = canAsk
        ? 'Ask the recording, e.g. where did I see a chair'
        : 'Search not ready — see the menu';
    chatMic.classList.toggle('hidden', !connected || !canAsk);
}

// ---- the lidar height band ----------------------------------------------------
// Two sliders over the cloud's OWN z range, so "all the way up" means the top of this
// recording rather than some number chosen for a different building. They are indices
// into that range, not metres: a range input's step is fixed at authoring time and the
// range is not known until the world arrives.
const HEIGHT_STEPS = 400;
const heightMinEl = document.getElementById('heightMin');
const heightMaxEl = document.getElementById('heightMax');
const heightMinVal = document.getElementById('heightMinVal');
const heightMaxVal = document.getElementById('heightMaxVal');
let heightRange = null;   // {low, high} metres, from the cloud header

for (const el of [heightMinEl, heightMaxEl]) {
    el.min = 0; el.max = HEIGHT_STEPS; el.step = 1;
    el.disabled = true;
    el.addEventListener('input', applyHeightBand);
}
document.getElementById('heightResetBtn').addEventListener('click', () => {
    heightMinEl.value = 0;
    heightMaxEl.value = HEIGHT_STEPS;
    applyHeightBand();
});

const heightAt = (slider) => {
    if (!heightRange) return null;
    const t = Number(slider.value) / HEIGHT_STEPS;
    return heightRange.low + t * (heightRange.high - heightRange.low);
};

function applyHeightBand() {
    if (!heightRange || !heightBand) return;
    // Either slider may be dragged past the other; the band is what lies BETWEEN them,
    // so read them as a pair rather than trusting which is which. Letting min exceed max
    // would empty the world with no way back but the reset button.
    const a = heightAt(heightMinEl);
    const b = heightAt(heightMaxEl);
    const low = Math.min(a, b);
    const high = Math.max(a, b);
    // Slack only where a slider is at its stop, so the labels stay true everywhere else.
    // The bounds came from the extreme voxels themselves, and an exact cut at a bound is
    // a float comparison against the very points that set it -- "all the way down" has to
    // mean all of them, not all but the lowest.
    const slack = (heightRange.high - heightRange.low) / HEIGHT_STEPS;
    const ends = [Number(heightMinEl.value), Number(heightMaxEl.value)];
    heightBand.value.set(
        Math.min(...ends) <= 0 ? low - slack : low,
        Math.max(...ends) >= HEIGHT_STEPS ? high + slack : high,
    );
    heightMinVal.textContent = `${low.toFixed(2)}m`;
    heightMaxVal.textContent = `${high.toFixed(2)}m`;
}

/** Scale the sliders to the cloud that just arrived, keeping them wide open. */
function syncHeightRange() {
    const bounds = scene && scene._cloudBounds;
    if (!bounds || !Number.isFinite(bounds.z_min) || !Number.isFinite(bounds.z_max)) return;
    heightRange = { low: bounds.z_min, high: bounds.z_max };
    heightMinEl.value = 0;
    heightMaxEl.value = HEIGHT_STEPS;
    heightMinEl.disabled = heightMaxEl.disabled = false;
    applyHeightBand();
}

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
    voxels: document.getElementById('layerVoxels'),
    photos: document.getElementById('layerPhotos'),
    hud: document.getElementById('layerHud'),
};
layerBoxes.voxels.addEventListener('change', () => scene && scene._cloudWanted !== layerBoxes.voxels.checked && scene.toggleCloud());
layerBoxes.photos.addEventListener('change', () => scene && scene._imageQuadGroup.visible !== layerBoxes.photos.checked && scene.toggleImages());
layerBoxes.hud.addEventListener('change', () => {
    if (scene && scene._hudPanel.visible !== layerBoxes.hud.checked) hudBtn.textContent = scene.toggleHud() ? 'Hide map' : 'Show map';
});
// Whether the conversation is wanted. Survives a reconnect, which the scene and the
// panel's contents do not. Declared before `showChat` reads it: a `let` after its first
// use is only safe by accident of call order.
let chatWanted = true;
const chatToggle = document.getElementById('chatToggle');
chatToggle.addEventListener('click', () => showChat(!document.body.classList.contains('chat-open')));

/** Show or hide the conversation. A phone never gets it, however it is asked for. */
function showChat(open) {
    const room = !document.body.classList.contains('touch');
    document.body.classList.toggle('chat-open', open && room);
    chatWanted = open;
}

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
    const wanted = { voxels: layerBoxes.voxels.checked, photos: layerBoxes.photos.checked,
        hud: layerBoxes.hud.checked };
    if (!scene) return;
    showChat(chatWanted);
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
    if (event.code === 'Slash') { event.preventDefault(); chatInput.focus(); return; }
    // C, beside the other layer keys. Here and not in `scene.js` with M/I/V because
    // the panel is DOM the scene knows nothing about.
    if (event.code === 'KeyC' && document.body.classList.contains('desktop-view')) {
        showChat(!document.body.classList.contains('chat-open'));
        return;
    }
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
    // The menu's one line about search. `index` is the server's own progress string
    // ("ready (402 frames)", "building (had 0 frames)", or why it refused one).
    searchNote.textContent = running
        ? `Adding embeddings… ${(indexStatus.progress || '').slice(0, 80)}`
        : `Frame index: ${(indexStatus.index || 'checking…').slice(0, 120)}`;
    embedBtn.classList.toggle('hidden', !connected || indexStatus.present);
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
    setStatus('Adding embeddings — this takes a while on a long recording');
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

chatMic.addEventListener('pointerdown', startRecording);
chatMic.addEventListener('pointerup', stopRecording);
chatMic.addEventListener('pointerleave', stopRecording);

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
        // A fresh connection is a fresh session: nothing on screen was asked for by the
        // person who just arrived. The server stopped replaying the last visitor's answer
        // over the wire, but the PAGE keeps its own copy -- the typed question, the
        // places, the route and the evidence photos all survive a disconnect, so a
        // reconnect used to come back to someone else's answer with no query behind it.
        chatInput.value = '';
        chatLogEl.textContent = '';
        if (results) results.clear();
        connectBtn.classList.add('hidden');
        applyIndexStatus(indexStatus);
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
    // The status poll runs over plain HTTP; it would outlive the socket otherwise.
    if (embedPoll) { clearInterval(embedPoll); embedPoll = null; }
    pendingQueryResult = null;
    pendingSceneMsgs.length = 0;
    perfEl.style.display = 'none';
    const timeline = document.getElementById('timeline');
    timeline.hidden = true;
    timeline.classList.remove('loading', 'replaying');
    orbitBtn.textContent = 'Orbit frame';  // the next world names its frame again
    hudBtn.textContent = 'Show map';       // a fresh scene starts with the minimap hidden
    if (replay) replay.dispose();
    replay = null;
    document.body.classList.remove('desktop-view', 'chat-open');
    chatLogEl.textContent = '';
    setAgentIdle(true);
    chatStateEl.textContent = 'not connected';
    connectBtn.classList.remove('hidden');
    connectBtn.disabled = false;
    applyAskAvailability();  // ws is null by now: the ask box goes dead with the mic
    embedBtn.classList.add('hidden');
    orbitBtn.classList.add('hidden');
    menuEl.classList.remove('open');
    if (tour && tour.active) tour.exit();
    if (tour) tour.dispose();
    if (results) results.dispose();
    // The next connect builds fresh ones; a key or a late message must not reach these.
    results = tour = flight = null;
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
    // Draw voxels as cubes (true) or spheres (false); also in the menu, remembered per browser.
    cubes: (on = null) => { if (on !== null) setVoxelStyle(on); return !!(voxelStyle && voxelStyle.value); },
    // The lidar height band: the slider range the world came with, and what is kept.
    heightBand: () => (heightBand ? { low: heightBand.value.x, high: heightBand.value.y, range: heightRange } : null),
    setHeightBand: (low, high) => {
        if (!heightRange) return null;
        const at = (m) => Math.round(((m - heightRange.low) / (heightRange.high - heightRange.low)) * HEIGHT_STEPS);
        heightMinEl.value = Math.max(0, Math.min(HEIGHT_STEPS, at(low)));
        heightMaxEl.value = Math.max(0, Math.min(HEIGHT_STEPS, at(high)));
        applyHeightBand();
        return { low: heightBand.value.x, high: heightBand.value.y };
    },
    orbitFrame: (frame) => setOrbitFrame(frame),
    searchStatus: () => indexStatus,
    // The museum tour: start, step, exit; state for automated checks.
    tour: () => tour,
    tourStart: (station = 0) => tour && tour.start(station),
    tourState: () => tour && tour.state(),
    flying: () => !!(flight && flight.flying),
    // Read-only handle for checking what the world is actually doing. The map button's
    // label is NOT that: it is initialised to "Show map" and only rewritten when a
    // toggle runs, so it reads the same whether the map is hidden or simply untouched.
    scene: () => scene,
    perf: () => (scene ? scene.getPerfStats() : null),
    resetPerf: () => scene && scene.resetPerf(),
    benchmark: (frames) => (scene ? scene.benchmarkRender(frames) : null),
    // Bring the i-th answer of the last result in front of the viewer (also key J).
    // Returns whether it actually took you anywhere. It used to end on either
    // `!scene.viewFrom(index)` or `scene.focusOn(...)`, which are false and undefined
    // respectively when the jump SUCCEEDS -- so the one thing this reports was wrong in
    // both directions, and an automated check reading it saw every jump fail.
    jumpTo: (index = null) => {
        if (index === null) return jumpToAnswer();
        const points = (scene && scene._lastResultPoints) || [];
        // A negative index is not "from the end" here: points[-1] is undefined and the
        // old length test let it through to a read of `.position`.
        if (!Number.isInteger(index) || index < 0 || index >= points.length) return false;
        // `index` counts PLACES, and `viewFrom` counts PHOTOGRAPHS -- there are several
        // per place. Passing it straight through took you to photograph 1, which belongs
        // to place 1, when you asked for place 2: 0.00 m from where you already were
        // instead of the 14.27 m to the place you named. Find this place's first
        // photograph; standing at it is the better answer, and focusOn is what there is
        // when the place has none.
        const photo = scene._queryImages.findIndex((h) => h && h.cluster === index);
        if (photo >= 0 && scene.viewFrom(photo)) return true;
        scene.focusOn(points[index].position);
        return true;
    },
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
