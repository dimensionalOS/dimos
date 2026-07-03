// Go2 teleop cockpit — big video left, control column right.
//
// Fully wired: posture/action commands go over state_reliable (acked on
// state_reliable_back), telemetry + battery come back over state_reliable_back,
// video is the WebRTC track, and drive (WASD/QE) reuses the keyboard loop /
// cmdChannel exactly as views/keyboard.js does.
//
// Preview with no broker:  window._teleopDev.previewGo2()

import { disconnect } from '../disconnect.js';
import { applyStampCrop, hudDetailRows, hudSummaryLine, statsHealth, transportLabel } from '../hud.js';
import { escHtml, state } from '../state.js';
import { startKeyboardLoop } from './keyboard.js';

// Command catalog — labels only; SPORT_CMD ids live robot-side.
// StandReady = standup + balance_stand (drive-ready); they always go together,
// so there's no separate Stand Up / Balance. No Recovery button either:
// Stand/Drive already ends in RecoveryStand robot-side, so it doubles as the
// recovery action — one less thing on the panel.
const POSTURE = [
    { name: 'StandReady', label: 'Stand / Drive' },
    { name: 'StandDown', label: 'Sit' },
];
// Robot actions. Hello/Stretch verified working; Pounce/Jump are acrobatic and
// UNVERIFIED on this firmware (may no-op) — and the robot leaps, so clear space.
const ACTIONS = [
    { name: 'Hello', label: 'Shake Hand' },
    { name: 'Stretch', label: 'Stretch' },
    { name: 'FrontPounce', label: 'Pounce' },
    { name: 'FrontJump', label: 'Jump Forward' },
];

// Speed bar. Normal/High = browser-side velocity scale (lin m/s-ish, ang).
// Rage = firmware Rage Mode (set_mode RPC) + full scale. mode is sent to the
// robot; scale is applied locally in buildTwist via state.speedScale.
// Camera tabs → robot composites the selected cameras into the one video track.
// cam1 = Go2, cam2 = RealSense. Toggle on/off; both = side-by-side. (B-ready:
// the same {camera_select, cams:[...]} protocol works for per-camera tracks.)
const CAMS = [
    { id: 'cam1', label: 'Cam 1' },
    { id: 'cam2', label: 'Cam 2' },
];

const SPEEDS = [
    { mode: 'normal', label: 'Normal', scale: { lin: 0.5, ang: 0.5 } },
    { mode: 'high', label: 'High', scale: { lin: 1.0, ang: 1.0 } },
    // Rage: firmware widens the envelope to ~2.5 m/s, but you only reach it by
    // pushing the stick HARDER. At lin=1.0 rage feels identical to High. The
    // working rage keyboard blueprint sends linear_speed=1.25; we go further to
    // actually exploit the wider envelope. (Note: buildTwist's Shift adds ×2 on
    // top, so effective max can exceed this.)
    { mode: 'rage', label: 'Rage', scale: { lin: 2.0, ang: 1.5 } },
];

// Local UI state. Posture/estop still placeholder; battery is wired to real
// telemetry. (Body-height shelved — firmware 3203.)
const ui = {
    posture: 'StandReady',  // robot auto-stands+balances on blueprint start
    estopped: false,
    speedMode: 'normal',      // speed bar selection
    selectedCams: ['cam1'],   // active camera tabs (default Go2)
    obstacleAvoid: true,      // onboard obstacle avoidance on/off (robot boots ON)
    light: 0,                 // head-LED brightness 0..1 (robot boots off; telemetry reconciles)
    lightDragging: false,     // don't let reconcile fight an in-progress drag
    robotVideoStalled: false, // robot-confirmed no-frames watchdog (telemetry)
    nonce: 0,                 // monotonic command id for ack matching
    pending: new Map(),       // nonce -> {el, timer}
    mainView: 'camera',       // 'camera' | 'map' — which is the big stage; other → PiP
    lastMap: null,            // latest decoded {type:map,...} for redraw between frames
    lastOdom: null,           // latest {x,y,yaw,ts} for the robot marker
};

let tickTimer = null;

// Is the operator → robot command channel up? (state_reliable)
function cmdReady() {
    return state.stateChannel && state.stateChannel.readyState === 'open' && !ui.estopped;
}

export function renderGo2(c) {
    const btn = (cmd) =>
        `<button class="cmd-btn relative" data-cmd="${cmd.name}" data-status="idle"><span>${cmd.label}</span></button>`;

    c.innerHTML = `
    <div class="h-screen flex flex-col p-3 lg:p-4 fade-in">
        <header class="flex items-center justify-between mb-3 shrink-0">
            <div class="flex items-center gap-4">
                <span class="crt-glow text-dim-500 font-bold tracking-widest text-lg">DIMENSIONAL</span>
                <span class="term-caps text-gray-600 text-xs">// GO2 TELEOP</span>
                <span class="text-gray-300 text-sm">${escHtml(state.activeRobot?.robot_name || 'go2')}</span>
            </div>
            <div class="flex items-center gap-3">
                <span id="link-pill" class="pill pill-good"><span class="dot"></span><span>LINK OK</span></span>
                <button id="disconnectBtn" class="term-caps px-3 py-1.5 text-xs text-gray-400 hover:text-white border border-[#2a2a2a] rounded">[ disconnect ]</button>
            </div>
        </header>

        <div class="flex-1 min-h-0 grid grid-cols-1 lg:grid-cols-[1fr_400px] gap-4">
            <!-- LEFT: main stage — camera OR map, swappable. The non-main one
                 shows in the floating PiP (bottom-right). state.mainView toggles.
                 SKELETON: markup + swap wired; map draw + PiP layout TBD. -->
            <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl overflow-hidden flex flex-col min-h-0">
                <!-- Slim control strip: swap + camera tabs. Kept compact (px-2
                     py-1, text-[11px]) so it's a thin bar, not a tall column. -->
                <div class="flex items-center gap-1.5 px-2 py-1 border-b border-[#2a2a2a] shrink-0">
                    <!-- Swap button — LEFT of the camera tabs (per spec). Flips
                         which of {camera, map} is the main stage vs the PiP. -->
                    <button id="view-swap" class="cmd-btn term-caps text-[11px] leading-none px-2 py-0.5" title="Swap camera / map">
                        ⇄ <span id="view-swap-label">MAP</span>
                    </button>
                    <!-- Camera tabs: toggle which cameras the robot composites into
                         the single video. At least one stays selected. -->
                    <div class="flex items-center gap-1.5" id="cam-tabs"></div>
                </div>
                <div class="relative flex-1 bg-black flex items-center justify-center min-h-0" id="stage">
                    <!-- Camera + map are BOTH always in the DOM; setMainView()
                         toggles a .is-main / .is-pip class on each so one fills
                         the stage and the other floats in the corner. The live
                         <video> is never reparented (that can drop the track). -->
                    <video id="robot-cam" autoplay muted playsinline
                        class="object-contain is-main" style="display:none;"></video>
                    <!-- Map canvas — occupancy grid + robot marker drawn on top. -->
                    <canvas id="map-canvas" class="is-pip"></canvas>
                    <!-- Centered status (Negotiating WebRTC…) + placeholder, both
                         hidden once the video track is actually playing. -->
                    <div id="video-placeholder" class="absolute inset-0 flex flex-col items-center justify-center text-center text-gray-500">
                        <div class="text-6xl mb-3">🐕</div>
                        <div id="teleop-status" class="text-lg text-gray-300 px-4 py-2 bg-bg-950/80 border border-[#2a2a2a] rounded-lg">
                            Negotiating WebRTC…
                        </div>
                    </div>
                    <div id="video-lost" class="hidden absolute inset-0 bg-black/70 flex flex-col items-center justify-center">
                        <div class="text-4xl mb-2">⚠</div>
                        <div class="term-caps text-sm text-[#f3b4b4]">video stalled — drive disabled</div>
                    </div>
                    <div class="absolute bottom-3 left-3 text-xs font-mono bg-black/40 rounded px-2 py-1 text-dim-400" id="twist-readout">
                        x 0.00 · y 0.00 · ω 0.00
                    </div>
                    <div class="absolute bottom-3 right-3">
                        <span class="pill pill-good"><span class="dot"></span><span id="posture-chip">STANDING</span></span>
                    </div>
                </div>
                <div class="border-t border-[#2a2a2a] p-3 flex items-center justify-between shrink-0">
                    <div class="flex items-center gap-3 text-xs text-gray-500">
                        <span id="kb-live" class="pill pill-good"><span class="dot"></span>KEYBOARD LIVE</span>
                        <span>Drive: <kbd class="px-1.5 py-0.5 bg-[#1f1f1f] rounded">W</kbd><kbd class="px-1.5 py-0.5 bg-[#1f1f1f] rounded">A</kbd><kbd class="px-1.5 py-0.5 bg-[#1f1f1f] rounded">S</kbd><kbd class="px-1.5 py-0.5 bg-[#1f1f1f] rounded">D</kbd></span>
                    </div>
                </div>
            </section>

            <!-- RIGHT: control column -->
            <aside class="flex flex-col gap-3 min-h-0 overflow-y-auto pr-1">
                <div id="blocked" class="hidden blocked-banner rounded-lg px-3 py-2 text-xs term-caps shrink-0"></div>

                <!-- Battery: symbol+label left, % right. No bar. -->
                <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl p-4 shrink-0 flex items-center justify-between">
                    <span class="text-sm text-gray-400">🔋 Battery</span>
                    <span id="batt-pct" class="text-sm font-semibold text-dim-400">—%</span>
                </section>

                <!-- Obstacle avoidance toggle -->
                <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl p-4 shrink-0 flex items-center justify-between">
                    <span class="text-sm text-gray-400">Obstacle avoidance</span>
                    <button id="obstacle-toggle" class="px-3 py-1 text-xs term-caps rounded border border-dim-700 text-dim-400">ON</button>
                </section>

                <!-- Head light brightness (0..1 → firmware levels 0-10) -->
                <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl p-4 shrink-0 flex items-center gap-3">
                    <span class="text-sm text-gray-400 shrink-0">💡 Light</span>
                    <input id="light-slider" type="range" min="0" max="1" step="0.1" value="0"
                        class="flex-1 accent-[#b0e1f0]">
                    <span id="light-val" class="text-xs font-mono text-dim-400 w-10 text-right">0%</span>
                </section>

                <!-- Telemetry: summary always; click to expand full detail. -->
                <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl p-4 shrink-0">
                    <button id="hud-toggle" class="w-full flex items-center justify-between mb-2">
                        <span class="term-caps text-xs text-gray-500">Telemetry <span id="hud-caret" class="text-gray-600">▸</span></span>
                        <span id="hud-health" class="pill pill-good"><span class="dot"></span><span id="hud-transport">Cloudflare</span></span>
                    </button>
                    <pre id="hud-summary" class="text-xs text-dim-400 leading-relaxed">—</pre>
                    <div id="hud-detail" class="hidden mt-2 pt-2 border-t border-[#2a2a2a] space-y-2.5"></div>
                </section>

                <!-- Posture -->
                <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl p-4 shrink-0">
                    <div class="term-caps text-xs text-gray-500 mb-3">Posture</div>
                    <div class="grid grid-cols-2 gap-2">${POSTURE.map(btn).join('')}</div>
                </section>

                <!-- Actions -->
                <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl p-4 shrink-0">
                    <div class="term-caps text-xs text-gray-500 mb-3">Actions</div>
                    <div class="grid grid-cols-2 gap-2">${ACTIONS.map(btn).join('')}</div>
                </section>

                <!-- Speed bar: Normal / High / Rage -->
                <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl p-4 shrink-0">
                    <div class="term-caps text-xs text-gray-500 mb-3">Speed</div>
                    <div class="grid grid-cols-3 gap-2" id="speed-bar"></div>
                </section>

                <!-- WASD drive indicator: lights up keys as they're pressed
                     (updateKeyVisuals() in keyboard.js toggles .pressed by id). -->
                <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl p-4 shrink-0">
                    <div class="term-caps text-xs text-gray-500 mb-3">Drive</div>
                    <div class="flex flex-col items-center gap-2">
                        <div class="flex gap-2">
                            <div id="key-q" class="kb-key kb-key-secondary">Q</div>
                            <div id="key-w" class="kb-key">W</div>
                            <div id="key-e" class="kb-key kb-key-secondary">E</div>
                        </div>
                        <div class="flex gap-2">
                            <div id="key-a" class="kb-key">A</div>
                            <div id="key-s" class="kb-key">S</div>
                            <div id="key-d" class="kb-key">D</div>
                        </div>
                    </div>
                    <div class="mt-3 text-[11px] text-gray-500 leading-relaxed">
                        <div><span class="text-gray-300">W/S</span> forward · back &nbsp; <span class="text-gray-300">A/D</span> turn left · right</div>
                        <div><span class="text-gray-300">Q/E</span> strafe left · right</div>
                        <div><span class="text-gray-300">Shift</span> 2× fast &nbsp; <span class="text-gray-300">Space</span> ½× slow</div>
                    </div>
                </section>

                <!-- E-STOP: sticks to bottom of the aside as it scrolls.
                     Original horizontal shape (full column width) preserved;
                     background content scrolls behind it. -->
                <section id="estop-dock" class="sticky bottom-0 z-10 mt-auto bg-bg-950 border border-[#2a2a2a] rounded-xl p-4 shrink-0 shadow-lg">
                    <button id="estop" class="estop">■ EMERGENCY STOP</button>
                    <button id="rearm" class="hidden mt-2 w-full py-2 text-xs term-caps text-gray-300 border border-[#2a2a2a] rounded hover:border-dim-700">
                        re-arm →
                    </button>
                </section>
            </aside>
        </div>
    </div>`;

    wireGo2();
    refreshControls();
    startTick();
    // Drive: reuse the proven keyboard loop verbatim — WASD → TwistStamped on
    // state.cmdChannel (same as views/keyboard.js). The loop's updateKeyVisuals()
    // lights up the #key-w/a/s/d Drive panel as keys are pressed.
    startKeyboardLoop();
}

// ── interaction ──────────────────────────────────────────────────────
function wireGo2() {
    document.getElementById('disconnectBtn').onclick = disconnect;

    // Telemetry expand/collapse — summary always, full detail grid on click.
    document.getElementById('hud-toggle').addEventListener('click', () => {
        const detail = document.getElementById('hud-detail');
        const collapsed = detail.classList.toggle('hidden');
        document.getElementById('hud-caret').textContent = collapsed ? '▸' : '▾';
        if (!collapsed) renderTelemetryGrid();  // populate immediately on expand
    });

    // Camera tabs: render toggles, wire selection.
    const tabs = document.getElementById('cam-tabs');
    tabs.innerHTML = CAMS.map((c) =>
        `<button data-cam="${c.id}" class="px-2 py-0.5 rounded text-[11px] leading-none border border-[#2a2a2a] text-gray-400">${c.label}</button>`
    ).join('');
    tabs.querySelectorAll('[data-cam]').forEach((b) =>
        b.addEventListener('click', () => toggleCam(b.dataset.cam)));
    renderCamTabs();

    // Speed bar: render 3 segments, select current, wire selection.
    const bar = document.getElementById('speed-bar');
    bar.innerHTML = SPEEDS.map((s) =>
        `<button class="cmd-btn" data-speed="${s.mode}" data-status="idle"><span>${s.label}</span></button>`
    ).join('');
    bar.querySelectorAll('[data-speed]').forEach((b) =>
        b.addEventListener('click', () => selectSpeed(b.dataset.speed)));

    document.getElementById('obstacle-toggle').addEventListener('click', toggleObstacleAvoid);
    renderObstacleToggle();
    wireLightSlider();

    // Video: webrtc.js sets srcObject + display:block on ontrack, but doesn't
    // know about our placeholder. Hide the dog+status overlay once frames flow
    // ('playing'); show it again if the stream drops.
    const cam = document.getElementById('robot-cam');
    const placeholder = document.getElementById('video-placeholder');
    const showPlaceholder = (on) => placeholder && placeholder.classList.toggle('hidden', !on);
    cam.addEventListener('playing', () => {
        cam.style.display = 'block';
        showPlaceholder(false);
    });
    cam.addEventListener('emptied', () => showPlaceholder(true)); // stream cleared on disconnect

    document.querySelectorAll('.cmd-btn[data-cmd]').forEach((b) =>
        b.addEventListener('click', () => sendCommand(b.dataset.cmd, b)));

    document.getElementById('estop').addEventListener('click', () => {
        ui.estopped = true;
        // Dedicated estop type: new robots latch (move() refuses twists,
        // commands rejected until estop_clear) AND Damp urgently. The legacy
        // sport_cmd Damp follows for older robots that don't know estop —
        // on new ones it's an idempotent second Damp (urgent path, harmless).
        // Fire-and-forget — don't gate the local latch on an ack.
        if (state.stateChannel && state.stateChannel.readyState === 'open') {
            state.stateChannel.send(JSON.stringify({ type: 'estop', nonce: ++ui.nonce }));
            state.stateChannel.send(JSON.stringify({ type: 'sport_cmd', name: 'Damp', nonce: ++ui.nonce }));
        }
        document.querySelectorAll('.cmd-btn').forEach((b) => (b.dataset.status = 'idle'));
        ui.posture = 'Damp';
        refreshControls();
    });
    document.getElementById('rearm').addEventListener('click', () => {
        ui.estopped = false;  // re-arm; operator must Stand/Drive-ready to resume
        // Clear the robot-side latch too (older robots ignore unknown types).
        if (state.stateChannel && state.stateChannel.readyState === 'open') {
            state.stateChannel.send(JSON.stringify({ type: 'estop_clear', nonce: ++ui.nonce }));
        }
        refreshControls();
    });

    // Body-height slider shelved for v1 (firmware 3203 unknown-api). The
    // command send/ack infra below stays — posture buttons will use it next.

    // Resolve command acks coming back on state_reliable_back (via webrtc.js).
    state.onCmdAck = onCmdAck;
    // Reconcile controls from robot-authoritative telemetry state (3Hz).
    state.onRobotState = onRobotState;
    // Minimap: occupancy grid (slow) + robot pose (fast), both on
    // state_reliable_back. SKELETON — handlers below.
    state.onMap = onMap;
    state.onOdom = onOdom;
    document.getElementById('view-swap').addEventListener('click', () => setMainView());
    // Click the floating PiP to swap too. Both media elements are always in the
    // DOM; only the one with .is-pip is visually the PiP, so a click on either
    // (guarded to the PiP one) flips the view.
    for (const id of ['robot-cam', 'map-canvas']) {
        document.getElementById(id).addEventListener('click', (e) => {
            if (e.currentTarget.classList.contains('is-pip')) setMainView();
        });
    }
    setMainView('camera');  // default: camera main, map floating (per spec)

    selectSpeed(ui.speedMode, /*sendToRobot=*/ false);  // reflect default selection
}

// ── minimap: map + robot marker (SKELETON) ───────────────────────────
// Camera stays the WebRTC <video>; the map is a <canvas> we draw ourselves.
// The two swap between the big stage and the floating PiP.

// Swap which of {camera, map} is the big stage vs the floating PiP. With no
// arg, flips the current view. Never reparents the <video> (would drop the
// track) — just toggles .is-main / .is-pip so CSS repositions each element.
function setMainView(view) {
    ui.mainView = view || (ui.mainView === 'camera' ? 'map' : 'camera');
    const cam = document.getElementById('robot-cam');
    const map = document.getElementById('map-canvas');
    const camMain = ui.mainView === 'camera';
    cam.classList.toggle('is-main', camMain);
    cam.classList.toggle('is-pip', !camMain);
    map.classList.toggle('is-main', !camMain);
    map.classList.toggle('is-pip', camMain);
    // Button/label name what a click switches TO (the other view).
    const label = document.getElementById('view-swap-label');
    if (label) label.textContent = camMain ? 'MAP' : 'CAM';
    // Canvas backing-store size changed (stage <-> PiP) → redraw at new size.
    drawMap();
}

// Occupancy grid (~2Hz). Decode the PNG once here (off the fast odom path),
// cache it, then redraw. The robot bakes the rerun palette into the PNG
// (violet-blue free / orange occupied / red lethal / transparent unknown), so
// there's no colormap here — drawMap just blits the image.
function onMap(msg) {
    if (!msg || !msg.png_b64) return;
    const img = new Image();
    img.onload = () => {
        ui.lastMap = { ...msg, img };
        drawMap();
    };
    img.onerror = () => {};  // ignore a corrupt frame; the next one redraws
    img.src = 'data:image/png;base64,' + msg.png_b64;
}

// Robot pose (~15Hz). Cache + redraw so the marker moves smoothly between the
// slower map frames. No PNG work here.
function onOdom(msg) {
    if (!msg) return;
    ui.lastOdom = msg;
    drawMap();
}

// Draw the cached map (scaled to fill the canvas, nearest-neighbour so cells
// stay crisp) then the robot glyph, placed via the grid origin + resolution:
//   col = (odom.x - origin[0]) / res ;  row = (odom.y - origin[1]) / res
// The grid is row-major from origin; y grows up in world but down in canvas,
// so the row is flipped. Yaw rotates the glyph (0 = +x world = canvas right).
function drawMap() {
    const canvas = document.getElementById('map-canvas');
    if (!canvas) return;
    const m = ui.lastMap;
    // Size the backing store to the element's box (avoids blur on resize/swap).
    const rect = canvas.getBoundingClientRect();
    const cw = Math.max(1, Math.round(rect.width));
    const ch = Math.max(1, Math.round(rect.height));
    if (canvas.width !== cw) canvas.width = cw;
    if (canvas.height !== ch) canvas.height = ch;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, cw, ch);
    if (!m || !m.img) {
        ctx.fillStyle = '#0b0d0d';
        ctx.fillRect(0, 0, cw, ch);
        ctx.fillStyle = '#3a4a4a';
        ctx.font = '12px monospace';
        ctx.fillText('awaiting map…', 10, 20);
        return;
    }
    // Fit the grid image into the canvas preserving aspect; letterbox the rest.
    const scale = Math.min(cw / m.w, ch / m.h);
    const dw = m.w * scale, dh = m.h * scale;
    const dx = (cw - dw) / 2, dy = (ch - dh) / 2;
    ctx.imageSmoothingEnabled = false;

    // The grid is row-major from the bottom-left origin: row 0 = min world y
    // (south), col 0 = min world x (west). Canvas y grows DOWN, so we flip the
    // image vertically to put world-north at the top — matching DimOS's own
    // renderer (OccupancyGrid.to_rerun does grid[::-1] for the same reason).
    // Everything below is drawn in this flipped frame so the marker agrees.
    ctx.save();
    ctx.translate(dx, dy + dh);
    ctx.scale(1, -1);                 // y-up within [0..dh] → world-north = top
    ctx.drawImage(m.img, 0, 0, dw, dh);

    // Robot footprint, in the same flipped frame. Drawn as a to-scale box:
    // the Go2 body is 0.70 m long (+x / heading) × 0.31 m wide (+y). Convert
    // metres → cells → px via the map scale; a min-size floor keeps it visible
    // on wide/zoomed-out maps where true scale would be a few px.
    const o = ui.lastOdom;
    if (o && m.res > 0) {
        const col = (o.x - m.origin[0]) / m.res;   // cells east of origin
        const row = (o.y - m.origin[1]) / m.res;    // cells north of origin
        const px = col * scale;
        const py = row * scale;                     // y-up here (frame flipped)
        if (px >= 0 && px <= dw && py >= 0 && py <= dh) {
            const pxPerM = scale / m.res;           // map px per world metre
            const MIN_LEN_PX = 14;                  // visibility floor (long side)
            let lenPx = GO2_LEN_M * pxPerM;         // along heading (+x)
            let widPx = GO2_WID_M * pxPerM;         // across (+y)
            if (lenPx < MIN_LEN_PX) {               // scale up together, keep ratio
                widPx *= MIN_LEN_PX / lenPx;
                lenPx = MIN_LEN_PX;
            }
            ctx.save();
            ctx.translate(px, py);
            // y-up frame: CCW world yaw = CCW canvas rotation (no sign flip).
            // Box local x = heading, local y = left. Center it on the robot.
            ctx.rotate(o.yaw || 0);
            ctx.fillStyle = 'rgba(176,225,240,0.35)';
            ctx.strokeStyle = '#b0e1f0';
            ctx.lineWidth = 1.5;
            ctx.fillRect(-lenPx / 2, -widPx / 2, lenPx, widPx);
            ctx.strokeRect(-lenPx / 2, -widPx / 2, lenPx, widPx);
            // Heading tick: short line from centre to the front (+x) edge so the
            // facing direction is unambiguous on the rectangle.
            ctx.beginPath();
            ctx.moveTo(0, 0);
            ctx.lineTo(lenPx / 2, 0);
            ctx.strokeStyle = '#0d0e0e';
            ctx.lineWidth = 2;
            ctx.stroke();
            ctx.restore();
        }
    }
    ctx.restore();
}

// Go2 body footprint (URDF base_link box): 0.70 m long × 0.31 m wide.
const GO2_LEN_M = 0.70;
const GO2_WID_M = 0.31;

// ── camera tabs ──────────────────────────────────────────────────────
function toggleCam(id) {
    const sel = new Set(ui.selectedCams);
    if (sel.has(id)) {
        if (sel.size === 1) return;  // keep at least one camera on
        sel.delete(id);
    } else {
        sel.add(id);
    }
    // Preserve CAMS order so side-by-side order is stable.
    ui.selectedCams = CAMS.map((c) => c.id).filter((id) => sel.has(id));
    renderCamTabs();
    sendCameraSelect();
}

function renderCamTabs() {
    document.querySelectorAll('#cam-tabs [data-cam]').forEach((b) => {
        const on = ui.selectedCams.includes(b.dataset.cam);
        b.className = 'px-2 py-0.5 rounded text-[11px] leading-none border ' +
            (on ? 'bg-dim-500 text-bg-950 border-dim-500' : 'border-[#2a2a2a] text-gray-400');
    });
}

function sendCameraSelect() {
    if (state.stateChannel && state.stateChannel.readyState === 'open') {
        state.stateChannel.send(JSON.stringify({ type: 'camera_select', cams: ui.selectedCams }));
    }
}

// ── speed bar ────────────────────────────────────────────────────────
function selectSpeed(mode, sendToRobot = true) {
    const spec = SPEEDS.find((s) => s.mode === mode);
    if (!spec) return;
    ui.speedMode = mode;
    // Apply the velocity scale locally NOW (buildTwist reads state.speedScale).
    state.speedScale = spec.scale;
    // Highlight the active segment.
    document.querySelectorAll('#speed-bar [data-speed]').forEach((b) =>
        b.classList.toggle('is-active', b.dataset.speed === mode));
    // Only rage crossing changes the robot FSM; the robot ignores normal<->high
    // (browser scale handles those). Send set_mode regardless — robot no-ops if
    // already in the right FSM, and acks.
    if (sendToRobot && state.stateChannel && state.stateChannel.readyState === 'open') {
        state.stateChannel.send(JSON.stringify({ type: 'set_mode', mode, nonce: ++ui.nonce }));
    }
}

function toggleObstacleAvoid() {
    if (!state.stateChannel || state.stateChannel.readyState !== 'open') return;
    ui.obstacleAvoid = !ui.obstacleAvoid;
    renderObstacleToggle();
    state.stateChannel.send(JSON.stringify(
        { type: 'obstacle_avoidance', enabled: ui.obstacleAvoid, nonce: ++ui.nonce }));
}

function renderObstacleToggle() {
    const b = document.getElementById('obstacle-toggle');
    if (!b) return;
    const on = ui.obstacleAvoid;
    b.textContent = on ? 'ON' : 'OFF';
    b.classList.toggle('text-dim-400', on);
    b.classList.toggle('border-dim-700', on);
    b.classList.toggle('text-gray-500', !on);
}

// ── head light (brightness slider, 0..1) ─────────────────────────────
// Live label while dragging; send on release only ({type:'light',
// brightness}). Ack feedback uses the cmd-sending/ok/err range classes.
function wireLightSlider() {
    const s = document.getElementById('light-slider');
    if (!s) return;
    const drag = (on) => { ui.lightDragging = on; };
    s.addEventListener('pointerdown', () => drag(true));
    s.addEventListener('pointerup', () => drag(false));
    s.addEventListener('touchstart', () => drag(true), { passive: true });
    s.addEventListener('touchend', () => drag(false));
    s.addEventListener('input', () => renderLightValue(parseFloat(s.value)));
    s.addEventListener('change', () => sendLight(parseFloat(s.value)));
    renderLightSlider();
}

function sendLight(brightness) {
    if (!state.stateChannel || state.stateChannel.readyState !== 'open') return;
    ui.light = brightness;
    const s = document.getElementById('light-slider');
    if (s) { s.classList.remove('cmd-ok', 'cmd-err'); s.classList.add('cmd-sending'); }
    const nonce = ++ui.nonce;
    state.stateChannel.send(JSON.stringify({ type: 'light', brightness, nonce }));
    const timer = setTimeout(() => resolveAck(nonce, false), 3000);
    ui.pending.set(nonce, { el: s, name: 'light', timer });
}

function renderLightValue(v) {
    const val = document.getElementById('light-val');
    if (val) val.textContent = `${Math.round(v * 100)}%`;
}

function renderLightSlider() {
    const s = document.getElementById('light-slider');
    if (s && !ui.lightDragging) s.value = String(ui.light);
    renderLightValue(ui.light);
}

// ── command ack (state_reliable_back) — shared by all nonce'd commands ──
function onCmdAck(msg) {
    resolveAck(msg.nonce, !!msg.ok);
}

// ── robot-state reconcile (robot_telemetry.state, 3Hz) ──────────────
// The robot is authoritative: on (re)connect the cockpit's optimistic
// defaults (StandReady, OA on, cams [cam1], no rage) get corrected to
// reality. Skipped while a command is pending — the robot still reports
// the old state until the ack lands, and flip-flopping the UI mid-click
// reads as a glitch.
function onRobotState(s) {
    if (ui.pending.size > 0) return;
    let dirty = false;
    if (typeof s.posture === 'string' && s.posture !== ui.posture) {
        ui.posture = s.posture;
        dirty = true;
    }
    if (typeof s.estopped === 'boolean' && s.estopped !== ui.estopped) {
        ui.estopped = s.estopped;
        dirty = true;
    }
    if (typeof s.video_stalled === 'boolean') {
        ui.robotVideoStalled = s.video_stalled;  // robot-confirmed camera stall
    }
    if (typeof s.obstacle_avoidance === 'boolean' && s.obstacle_avoidance !== ui.obstacleAvoid) {
        ui.obstacleAvoid = s.obstacle_avoidance;
        renderObstacleToggle();
    }
    if (typeof s.light === 'number' && !ui.lightDragging && Math.abs(s.light - ui.light) > 0.01) {
        ui.light = s.light;
        renderLightSlider();
    }
    if (Array.isArray(s.cams) && s.cams.join() !== ui.selectedCams.join()) {
        ui.selectedCams = s.cams.filter((c) => CAMS.some((k) => k.id === c));
        if (!ui.selectedCams.length) ui.selectedCams = ['cam1'];
        renderCamTabs();
    }
    // Rage is firmware truth; normal-vs-high is browser-only, so only the
    // rage boundary is reconcilable. Don't send set_mode back — this IS the
    // robot's state.
    if (typeof s.rage === 'boolean') {
        const uiRage = ui.speedMode === 'rage';
        if (s.rage !== uiRage) selectSpeed(s.rage ? 'rage' : 'normal', /*sendToRobot=*/ false);
    }
    if (dirty) refreshControls();
}

function resolveAck(nonce, ok) {
    const p = ui.pending.get(nonce);
    if (!p) return;
    clearTimeout(p.timer);
    ui.pending.delete(nonce);
    const btn = p.el;
    // Track posture optimistically on a confirmed posture command. StandReady
    // is an action (stand+balance), not a latched state — map it to standing.
    const POSTURE_STATE = { StandReady: 'StandReady', StandDown: 'StandDown', RecoveryStand: 'RecoveryStand', Sit: 'Sit' };
    if (ok && POSTURE_STATE[p.name]) ui.posture = POSTURE_STATE[p.name];
    // Range inputs (light slider) flash via the cmd-* classes; buttons via
    // data-status. 700ms flash → idle; bail if the cockpit unmounted.
    const isRange = btn && btn.tagName === 'INPUT';
    if (isRange) {
        btn.classList.remove('cmd-sending');
        btn.classList.add(ok ? 'cmd-ok' : 'cmd-err');
    } else if (btn) {
        btn.dataset.status = ok ? 'done' : 'error';
    }
    setTimeout(() => {
        if (!document.getElementById('hud-summary')) return;
        if (isRange) btn.classList.remove('cmd-ok', 'cmd-err');
        else if (btn) btn.dataset.status = 'idle';
        refreshControls();
    }, 700);
    refreshControls();
}

// Posture/gesture button → {type:sport_cmd, name, nonce} on state_reliable.
// Robot allow-lists + dispatches, then acks on state_reliable_back (→ resolveAck).
// Acrobatic actions make the robot leap — confirm before firing so a stray
// click doesn't launch it. (Matches the robot-side allow-list entries.)
const CONFIRM_ACTIONS = new Set(['FrontPounce', 'FrontJump']);

function sendCommand(name, btn) {
    if (!cmdReady()) return;
    if (CONFIRM_ACTIONS.has(name) &&
        !confirm(`${name} makes the robot leap — clear the area. Continue?`)) {
        return;
    }
    const nonce = ++ui.nonce;
    btn.dataset.status = 'pending';
    state.stateChannel.send(JSON.stringify({ type: 'sport_cmd', name, nonce }));
    // Ack watchdog. StandReady is a robot-side combo (standup → recovery →
    // balance → joystick) with ~3.6s of settling sleeps — 3s would mark it
    // failed while the robot is mid-stand.
    const timeoutMs = name === 'StandReady' ? 9000 : 3000;
    const timer = setTimeout(() => resolveAck(nonce, false), timeoutMs);
    ui.pending.set(nonce, { el: btn, name, timer });
}

function refreshControls() {
    const reason = ui.estopped ? 'E-STOPPED — re-arm to resume' : null;
    const banner = document.getElementById('blocked');
    if (!banner) return;
    banner.textContent = reason || '';
    banner.classList.toggle('hidden', !reason);

    document.querySelectorAll('.cmd-btn').forEach((b) => {
        const active = b.dataset.cmd === ui.posture;
        b.classList.toggle('is-active', active);
        // Disable the active posture so you can't re-fire it — EXCEPT StandReady,
        // which you may want to re-press to re-arm drive after sitting.
        const lockActive = active && b.dataset.cmd !== 'StandReady';
        b.disabled = !!reason || (lockActive && b.dataset.status === 'idle');
    });

    document.getElementById('estop').classList.toggle('latched', ui.estopped);
    document.getElementById('rearm').classList.toggle('hidden', !ui.estopped);

    // Drive is live only in Stand/Drive (StandReady) and not e-stopped. Other
    // postures (Recovery/Sit/StandDown) change pose but don't accept WASD —
    // press Stand/Drive to start moving. Gates the keyboard loop's send.
    state.driveEnabled = ui.posture === 'StandReady' && !ui.estopped;

    const kb = document.getElementById('kb-live');
    const stalled = state.videoStall.stalled;
    kb.className = 'pill ' + (state.driveEnabled && !stalled ? 'pill-good' : 'pill-bad');
    kb.querySelector('.dot').nextSibling.textContent =
        stalled ? 'DRIVE OFF — video stalled'
        : state.driveEnabled ? 'DRIVE LIVE' : 'DRIVE OFF — press Stand/Drive';

    document.getElementById('posture-chip').textContent =
        ({ StandReady: 'STANDING', StandDown: 'SITTING', RecoveryStand: 'RECOVERY', Damp: 'STOPPED' }[ui.posture]) ||
        ui.posture;

    renderBattery();
}

function renderBattery() {
    const pct = document.getElementById('batt-pct');
    if (!pct) return;
    // Real SOC from robot_telemetry (state_reliable_back); null until first push.
    const soc = state.liveStats?.soc;
    if (soc == null) {
        pct.textContent = '—%';
        pct.style.color = '#6b7280';
        return;
    }
    const p = Math.max(0, Math.min(100, soc));
    pct.textContent = `${p}%`;
    pct.style.color = p > 40 ? '#c4e7f3' : p > 15 ? '#eab308' : '#f3b4b4';
}

// Telemetry tick (1Hz): samples the operator's own send rate (cmdHz), then
// renders the summary + full detail from the shared hud.js formatters, drives
// the health pill, and updates battery. Reuses hud.js so the cockpit and the
// keyboard HUD stay in sync.
// Telemetry value tint by per-metric health (matches the .pill palette).
const HEALTH_TINT = { good: 'text-[#b0e1f0]', warn: 'text-[#eab308]', bad: 'text-[#f3b4b4]' };

function renderTelemetryGrid() {
    const el = document.getElementById('hud-detail');
    if (!el || el.classList.contains('hidden')) return;  // skip work when collapsed
    el.innerHTML = hudDetailRows().map((g) => `
        <div>
            <div class="term-caps text-[10px] text-gray-600 mb-1">${g.group}</div>
            <div class="grid grid-cols-2 gap-x-3 gap-y-1">
                ${g.rows.map((r) => `
                    <span class="text-xs text-gray-500">${r.label}</span>
                    <span class="text-xs text-right font-mono ${HEALTH_TINT[r.health] || 'text-gray-300'}">${r.value}</span>
                `).join('')}
            </div>
        </div>`).join('');
}

let _lastHudSample = 0;
let _noVideoSinceMs = 0;  // connected-but-never-saw-a-frame escalation timer
function startTick() {
    stopTick();
    _lastHudSample = performance.now();
    _noVideoSinceMs = 0;
    tickTimer = setInterval(() => {
        // Sample command send rate (cmdSendCount incremented in the drive loop).
        const now = performance.now();
        const dt = (now - _lastHudSample) / 1000;
        if (dt > 0) state.liveStats.cmdHz = state.cmdSendCount / dt;
        state.cmdSendCount = 0;
        _lastHudSample = now;

        // Bail if the cockpit DOM is gone (failed connect / view teardown) —
        // the interval can outlive the elements for a tick.
        const summary = document.getElementById('hud-summary');
        if (!summary) return;

        // Hide the benchmark timestamp strip from the video (display-only).
        applyStampCrop();

        // Never-got-video escalation: connected but no first frame → after 8s
        // stop saying "Negotiating…" and name the likely culprit. The robot's
        // no-frames watchdog flag (telemetry) upgrades it to a confirmed
        // diagnosis. Both clear naturally once frames arrive (placeholder
        // hides on 'playing').
        const chOpen = state.cmdChannel && state.cmdChannel.readyState === 'open';
        const statusEl = document.getElementById('teleop-status');
        if (statusEl && chOpen && !state.videoStall.armed) {
            if (!_noVideoSinceMs) _noVideoSinceMs = now;
            if (ui.robotVideoStalled) {
                statusEl.textContent =
                    'Robot reports its camera is stalled — power-cycle the robot';
            } else if (now - _noVideoSinceMs > 8000) {
                statusEl.textContent =
                    'Connected — no video from robot (power-cycle it if this persists)';
            }
        } else {
            _noVideoSinceMs = 0;
        }

        // Video-freshness lockout (keyboard loop drives state.videoStall):
        // stalled → overlay + drive pill off; the loop already blocks sends.
        const lost = document.getElementById('video-lost');
        const stalled = state.videoStall.stalled;
        if (lost && lost.classList.contains('hidden') !== !stalled) {
            lost.classList.toggle('hidden', !stalled);
            refreshControls();  // re-render the DRIVE pill on stall transitions
        }
        // Mid-stream stall: enrich the overlay when the robot CONFIRMS its
        // camera died (vs a plain network freeze).
        if (lost && stalled) {
            const label = lost.querySelector('.term-caps');
            if (label) label.textContent = ui.robotVideoStalled
                ? 'robot camera stalled — power-cycle the robot · drive disabled'
                : 'video stalled — drive disabled';
        }

        // Summary always; detail grid rendered (hidden until expanded).
        summary.textContent = hudSummaryLine();
        renderTelemetryGrid();

        // Health pills (good/warn/bad) + transport label. Both the telemetry
        // header pill and the video-overlay LINK pill track signal health.
        const health = statsHealth();
        const pill = document.getElementById('hud-health');
        if (pill) pill.className = 'pill pill-' + health;
        const transport = document.getElementById('hud-transport');
        if (transport) transport.textContent = transportLabel();
        const linkPill = document.getElementById('link-pill');
        if (linkPill) {
            linkPill.className = 'pill pill-' + health;
            linkPill.querySelector('span:last-child').textContent =
                { good: 'LINK OK', warn: 'LINK WEAK', bad: 'LINK BAD' }[health];
        }

        renderBattery();
    }, 1000);
}

export function stopTick() {
    if (tickTimer) {
        clearInterval(tickTimer);
        tickTimer = null;
    }
    // Drop the ack/state hooks + pending watchdogs so they don't leak to the next view.
    if (state.onCmdAck === onCmdAck) state.onCmdAck = null;
    if (state.onRobotState === onRobotState) state.onRobotState = null;
    ui.pending.forEach((p) => clearTimeout(p.timer));
    ui.pending.clear();
}
