// Go2 teleop cockpit — big video left, control column right.
//
// Fully wired: posture/action commands go over state_reliable (acked on
// state_reliable_back), telemetry + battery come back over state_reliable_back,
// video is the WebRTC track, and drive (WASD/QE) reuses the keyboard loop /
// cmdChannel exactly as views/keyboard.js does.
//
// Preview with no broker:  window._teleopDev.previewGo2()

import { disconnect } from '../disconnect.js';
import { hudDetailRows, hudSummaryLine, statsHealth, transportLabel } from '../hud.js';
import { escHtml, state } from '../state.js';
import { startKeyboardLoop, stopKeyboardLoop } from './keyboard.js';

// Command catalog — labels only; SPORT_CMD ids live robot-side.
// StandReady = standup + balance_stand (drive-ready); they always go together,
// so there's no separate Stand Up / Balance.
const POSTURE = [
    { name: 'StandReady', label: 'Stand / Drive' },
    { name: 'StandDown', label: 'Sit' },
    { name: 'RecoveryStand', label: 'Recovery' },
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
    { id: 'cam1', label: 'Cam 1 · Go2' },
    { id: 'cam2', label: 'Cam 2 · RealSense' },
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
    nonce: 0,                 // monotonic command id for ack matching
    pending: new Map(),       // nonce -> {el, timer}
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
            <!-- LEFT: video -->
            <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl overflow-hidden flex flex-col min-h-0">
                <!-- Camera tabs: toggle which cameras the robot composites into the
                     single video. cam1 (Go2) default; cam2 (RealSense) optional;
                     both → side-by-side. At least one stays selected. -->
                <div class="flex items-center gap-2 p-2 border-b border-[#2a2a2a] shrink-0" id="cam-tabs"></div>
                <div class="relative flex-1 bg-black flex items-center justify-center min-h-0">
                    <video id="robot-cam" autoplay muted playsinline
                        class="w-full h-full object-contain" style="display:none;"></video>
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
        `<button data-cam="${c.id}" class="px-3 py-1 rounded text-xs border border-[#2a2a2a] text-gray-400">${c.label}</button>`
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
        // E-STOP = Damp: robot goes limp immediately. Send on state_reliable
        // (reliable plane). Fire-and-forget — don't gate the latch on an ack.
        if (state.stateChannel && state.stateChannel.readyState === 'open') {
            state.stateChannel.send(JSON.stringify({ type: 'sport_cmd', name: 'Damp', nonce: ++ui.nonce }));
        }
        document.querySelectorAll('.cmd-btn').forEach((b) => (b.dataset.status = 'idle'));
        ui.posture = 'Damp';
        refreshControls();
    });
    document.getElementById('rearm').addEventListener('click', () => {
        ui.estopped = false;  // re-arm; operator must Stand/Drive-ready to resume
        refreshControls();
    });

    // Body-height slider shelved for v1 (firmware 3203 unknown-api). The
    // command send/ack infra below stays — posture buttons will use it next.

    // Resolve command acks coming back on state_reliable_back (via webrtc.js).
    state.onCmdAck = onCmdAck;

    selectSpeed(ui.speedMode, /*sendToRobot=*/ false);  // reflect default selection
}

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
        b.className = 'px-3 py-1 rounded text-xs border ' +
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

// ── command ack (state_reliable_back) — shared by all nonce'd commands ──
function onCmdAck(msg) {
    resolveAck(msg.nonce, !!msg.ok);
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
    btn.dataset.status = ok ? 'done' : 'error';
    // 700ms done/error flash → idle. Bail if the cockpit unmounted in between.
    setTimeout(() => {
        if (!document.getElementById('hud-summary')) return;
        btn.dataset.status = 'idle';
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
    // Watchdog: if no ack in 3s, mark error and clear pending.
    const timer = setTimeout(() => resolveAck(nonce, false), 3000);
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
function startTick() {
    stopTick();
    _lastHudSample = performance.now();
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

        // Video-freshness lockout (keyboard loop drives state.videoStall):
        // stalled → overlay + drive pill off; the loop already blocks sends.
        const lost = document.getElementById('video-lost');
        const stalled = state.videoStall.stalled;
        if (lost && lost.classList.contains('hidden') !== !stalled) {
            lost.classList.toggle('hidden', !stalled);
            refreshControls();  // re-render the DRIVE pill on stall transitions
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
    // Drop the ack hook + pending watchdogs so they don't leak to the next view.
    if (state.onCmdAck === onCmdAck) state.onCmdAck = null;
    ui.pending.forEach((p) => clearTimeout(p.timer));
    ui.pending.clear();
}
