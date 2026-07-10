// Desktop browser cockpit for the hosted xArm — keyboard EE-jog, in parallel
// with the Quest VR cockpit (both drive the same ControlCoordinator; the robot
// arbitrates, VR overriding keyboard when engaged). Mirrors views/go2.js.
//
//   WASD/QE            → EE linear X/Y/Z    (m/s)
//   Shift + WASD/QE    → EE angular R/P/Y   (rad/s)
//   Space              → gripper toggle open/close
//   E-STOP button      → latch/clear
//
// Twist → cmd_unreliable (state.cmdChannel) as TwistStamped frame_id
// "eef_twist_arm"; gripper + estop → state_reliable JSON. Shares transport with
// vrarm.js/xarmcmd.js against the same ArmHostedConnection.

import { disconnect } from '../disconnect.js';
import { hudDetailRows, hudSummaryLine, sampleCmdHz, statsHealth, transportLabel } from '../hud.js';
import { createStallGate, videoMediaTime } from '../stall.js';
import { escHtml, sendInterval, state } from '../state.js';
import {
    buildEEFTwist, sendCameraSelect, sendEstop, sendEstopClear, sendGripper,
} from '../xarmcmd.js';

// Jog speeds. Modest so the arm is controllable near singularities.
const LINEAR_SPEED = 0.12;   // m/s
const ANGULAR_SPEED = 0.8;   // rad/s

// key → (vector, axis, sign). Same keys serve linear (no Shift) and angular
// (with Shift); the loop picks which vector based on the Shift state.
const AXIS_KEYS = {
    w: ['x', +1], s: ['x', -1],   // forward / back
    a: ['y', +1], d: ['y', -1],   // left / right
    q: ['z', +1], e: ['z', -1],   // up / down
};

const _held = new Set();
let _estopped = false;
let _gripperClosed = false;
let _estopNonce = 0;
let _cams = ['cam1', 'cam2'];  // default: both (robot muxes them side-by-side)
let _camsRequested = false;
let _wasSending = false;  // true while actively jogging (for one-shot stop on release)

function trackedKey(e) {
    const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
    if (k in AXIS_KEYS || k === 'Shift' || k === ' ') return k === ' ' ? 'Space' : k;
    return null;
}

function onKeyDown(e) {
    const k = trackedKey(e);
    if (k === null) return;
    e.preventDefault();
    if (k === 'Space') {
        // Edge-triggered gripper toggle (once per press, not per repeat).
        if (!e.repeat) {
            _gripperClosed = !_gripperClosed;
            sendGripper(state.stateChannel, _gripperClosed);
            paintStatus();
        }
        return;
    }
    _held.add(k);
}
function onKeyUp(e) {
    const k = trackedKey(e);
    if (k === null) return;
    e.preventDefault();
    _held.delete(k);
}
function clearHeld() { _held.clear(); }

function buildTwist() {
    const linear = { x: 0, y: 0, z: 0 };
    const angular = { x: 0, y: 0, z: 0 };
    const rot = _held.has('Shift');
    const vec = rot ? angular : linear;
    const speed = rot ? ANGULAR_SPEED : LINEAR_SPEED;
    for (const key of _held) {
        const b = AXIS_KEYS[key];
        if (b) vec[b[0]] += b[1] * speed;
    }
    return { linear, angular };
}

function triggerEstop() {
    if (_estopped) { _estopped = false; sendEstopClear(state.stateChannel, () => ++_estopNonce); }
    else { _estopped = true; sendEstop(state.stateChannel, () => ++_estopNonce); }
    paintStatus();
}

function paintStatus() {
    const grip = document.getElementById('arm-grip');
    if (grip) {
        grip.textContent = _gripperClosed ? 'CLOSED' : 'OPEN';
        grip.className = _gripperClosed ? 'text-amber-400 font-bold' : 'text-green-400 font-bold';
    }
    const es = document.getElementById('arm-estop-btn');
    if (es) {
        es.textContent = _estopped ? 'E-STOP LATCHED — CLEAR' : 'E-STOP';
        es.className = _estopped
            ? 'w-full px-4 py-4 bg-red-900 border border-red-500 text-red-100 font-bold rounded-lg'
            : 'w-full px-4 py-4 bg-red-600 hover:bg-red-500 text-white font-bold rounded-lg';
    }
    // Highlight the active camera selection.
    const active = _cams.length === 2 ? 'both' : _cams[0];
    document.querySelectorAll('.cam-btn').forEach((b) => {
        const on = b.dataset.cam === active;
        b.className = 'cam-btn flex-1 px-2 py-2 rounded text-sm '
            + (on ? 'bg-dim-500 text-bg-950 font-medium' : 'bg-[#1f1f1f] text-gray-300 hover:bg-[#2a2a2a]');
    });
}

export function renderArm(c) {
    c.innerHTML = `
    <div class="min-h-screen flex flex-col md:flex-row gap-4 p-4 fade-in">
        <!-- Video -->
        <div class="flex-1 flex flex-col">
            <div class="flex items-center justify-between mb-2">
                <h1 class="text-2xl font-bold text-white">${escHtml(state.activeRobot?.robot_name || 'xArm teleop')}</h1>
                <button id="disconnectBtn" class="term-caps px-3 py-1.5 text-xs text-gray-400 hover:text-white border border-[#2a2a2a] rounded">[ disconnect ]</button>
            </div>
            <!-- setStatus() targets #teleop-status -->
            <div id="teleop-status" class="text-sm text-gray-300 px-3 py-2 bg-bg-950 border border-[#2a2a2a] rounded-lg mb-3">Negotiating…</div>
            <!-- Stage: relative + flex-1 + min-h-0 so the video actually gets
                 height in the flex column (mirrors the go2 view's #stage). The
                 <video> fills it absolutely; object-contain letterboxes. -->
            <div class="relative flex-1 min-h-0 bg-black rounded-lg border border-[#2a2a2a] overflow-hidden" style="min-height:320px;">
                <!-- Starts display:none; webrtc.js flips it to block when the track
                     arrives (that relayout is what forces the first paint — the
                     go2/keyboard views rely on the same toggle). -->
                <video id="robot-cam" autoplay muted playsinline
                    class="absolute inset-0 w-full h-full object-contain" style="display:none;"></video>
            </div>
        </div>
        <!-- Right control panel -->
        <div class="w-full md:w-72 flex flex-col gap-3">
            <!-- Telemetry: summary always; click to expand full detail. -->
            <section class="bg-bg-950 border border-[#2a2a2a] rounded-lg p-3">
                <button id="hud-toggle" class="w-full flex items-center justify-between mb-2">
                    <span class="term-caps text-xs text-gray-500">Telemetry <span id="hud-caret" class="text-gray-600">▸</span></span>
                    <span id="hud-health" class="pill pill-good"><span class="dot"></span><span id="hud-transport">—</span></span>
                </button>
                <pre id="hud-summary" class="text-xs text-dim-400 leading-relaxed">—</pre>
                <div id="hud-detail" class="hidden mt-2 pt-2 border-t border-[#2a2a2a] space-y-2.5"></div>
            </section>
            <div class="bg-bg-950 border border-[#2a2a2a] rounded-lg p-3">
                <div class="text-gray-400 text-xs term-caps mb-2">Camera</div>
                <div id="arm-cams" class="flex gap-2">
                    <button data-cam="cam1" class="cam-btn flex-1 px-2 py-2 rounded text-sm">Cam 1</button>
                    <button data-cam="cam2" class="cam-btn flex-1 px-2 py-2 rounded text-sm">Cam 2</button>
                    <button data-cam="both" class="cam-btn flex-1 px-2 py-2 rounded text-sm">Both</button>
                </div>
            </div>
            <div class="bg-bg-950 border border-[#2a2a2a] rounded-lg p-3">
                <div class="text-gray-400 text-xs term-caps mb-2">Gripper</div>
                <div class="text-2xl"><span id="arm-grip" class="text-green-400 font-bold">OPEN</span></div>
                <div class="text-gray-500 text-xs mt-1">Space to toggle</div>
            </div>
            <div class="bg-bg-950 border border-[#2a2a2a] rounded-lg p-3 text-sm text-gray-300 leading-relaxed">
                <div class="text-gray-400 text-xs term-caps mb-2">Controls</div>
                <div><b class="text-white">W/S</b> ± X &nbsp; <b class="text-white">A/D</b> ± Y &nbsp; <b class="text-white">Q/E</b> ± Z</div>
                <div class="mt-1"><b class="text-white">Shift</b> + keys → roll/pitch/yaw</div>
                <div class="mt-1"><b class="text-white">Space</b> → gripper</div>
                <pre id="arm-readout" class="text-gray-500 text-xs mt-3 font-mono"></pre>
            </div>
            <button id="arm-estop-btn"
                class="w-full px-4 py-4 bg-red-600 hover:bg-red-500 text-white font-bold rounded-lg">E-STOP</button>
        </div>
    </div>`;

    document.getElementById('disconnectBtn').onclick = disconnect;
    document.getElementById('arm-estop-btn').onclick = triggerEstop;
    document.querySelectorAll('.cam-btn').forEach((b) => {
        b.onclick = () => selectCam(b.dataset.cam);
    });
    // Telemetry expand/collapse — summary always, full detail grid on expand.
    document.getElementById('hud-toggle').addEventListener('click', () => {
        const detail = document.getElementById('hud-detail');
        const collapsed = detail.classList.toggle('hidden');
        document.getElementById('hud-caret').textContent = collapsed ? '▸' : '▾';
        if (!collapsed) renderTelemetryGrid();
    });
    paintStatus();
}

const HEALTH_TINT = { good: 'text-[#b0e1f0]', warn: 'text-[#eab308]', bad: 'text-[#f3b4b4]' };

function renderTelemetryGrid() {
    const el = document.getElementById('hud-detail');
    if (!el || el.classList.contains('hidden')) return;
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

// 1Hz telemetry tick: sample the operator send rate, refresh the always-on
// summary + health pill (+ detail grid when expanded) via the shared hud.js
// formatters, so this cockpit's HUD matches Go2's.
function startHudTick() {
    stopHudTick();
    let last = performance.now();
    state.armHudTimer = setInterval(() => {
        const now = performance.now();
        sampleCmdHz((now - last) / 1000);
        last = now;
        const summary = document.getElementById('hud-summary');
        if (!summary) return;
        summary.textContent = hudSummaryLine();
        const health = statsHealth();
        const pill = document.getElementById('hud-health');
        if (pill) pill.className = `pill pill-${health}`;
        const tl = document.getElementById('hud-transport');
        if (tl) tl.textContent = transportLabel();
        renderTelemetryGrid();
    }, 1000);
}

function stopHudTick() {
    if (state.armHudTimer) { clearInterval(state.armHudTimer); state.armHudTimer = null; }
}

function selectCam(which) {
    _cams = which === 'both' ? ['cam1', 'cam2'] : [which];
    sendCameraSelect(state.stateChannel, _cams);
    paintStatus();
}

export function startArmLoop() {
    stopArmLoop();
    _held.clear();
    _estopped = false;
    _gripperClosed = false;
    _cams = ['cam1', 'cam2'];
    _camsRequested = false;
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
    window.addEventListener('blur', clearHeld);
    startHudTick();  // inline telemetry panel (summary + expand)

    const stallGate = createStallGate();
    state.videoStall = { stalled: false, blocked: false, armed: false };

    state.kbInterval = setInterval(() => {
        // Once the reliable channel opens, ask the robot for both cameras (the
        // mux defaults to cam1 only). One-shot.
        if (!_camsRequested && state.stateChannel && state.stateChannel.readyState === 'open') {
            sendCameraSelect(state.stateChannel, _cams);
            _camsRequested = true;
            paintStatus();
        }

        const chan = state.cmdChannel;
        if (!chan || chan.readyState !== 'open') return;

        const held = _held.size > 0;
        const gate = stallGate.sample(
            videoMediaTime(document.getElementById('robot-cam')), performance.now(), held,
        );
        state.videoStall = gate;

        // Send only while jogging (or one zero-twist on release / block / estop),
        // NOT every tick — flooding the datachannel with idle zero-twists competes
        // with the video and adds latency. The robot's eef_twist holds when idle.
        const shouldStop = gate.blocked || _estopped || !held;
        if (shouldStop) {
            if (_wasSending) {
                const nowMs = Date.now() + state.clockOffsetMs;
                chan.send(buildEEFTwist({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, nowMs).encode());
                _wasSending = false;
            }
            return;
        }

        const { linear, angular } = buildTwist();
        const nowMs = Date.now() + state.clockOffsetMs;
        chan.send(buildEEFTwist(linear, angular, nowMs).encode());
        state.cmdSendCount++;
        _wasSending = true;

        const out = document.getElementById('arm-readout');
        if (out) out.textContent =
            `lin  ${linear.x.toFixed(2)} ${linear.y.toFixed(2)} ${linear.z.toFixed(2)}\n` +
            `ang  ${angular.x.toFixed(2)} ${angular.y.toFixed(2)} ${angular.z.toFixed(2)}`;
    }, sendInterval);
}

export function stopArmLoop() {
    window.removeEventListener('keydown', onKeyDown);
    window.removeEventListener('keyup', onKeyUp);
    window.removeEventListener('blur', clearHeld);
    if (state.kbInterval) { clearInterval(state.kbInterval); state.kbInterval = null; }
    stopHudTick();
    _held.clear();
}
