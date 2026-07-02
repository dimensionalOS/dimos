// Desktop / phone view: WASD + on-screen touch keys → TwistStamped on
// cmd_unreliable. Same channel + cadence as VR.

import { geometry_msgs, std_msgs } from 'https://esm.sh/jsr/@dimos/msgs@0.1.4';
import { disconnect } from '../disconnect.js';
import { createStallGate, videoMediaTime } from '../stall.js';
import { escHtml, sendInterval, state } from '../state.js';

export function renderKeyboard(c) {
    c.innerHTML = `
    <div class="min-h-screen flex flex-col items-center justify-center p-6 fade-in select-none">
        <div class="w-full max-w-xl text-center">
            <h1 class="text-3xl font-bold text-white mb-2">${escHtml(state.activeRobot?.robot_name || 'Keyboard teleop')}</h1>
            <div id="teleop-status" class="text-lg text-gray-300 px-4 py-3 bg-bg-950 border border-[#2a2a2a] rounded-lg my-4">
                Negotiating...
            </div>

            <video id="robot-cam" autoplay muted playsinline
                class="w-full rounded-lg border border-[#2a2a2a] bg-black my-4"
                style="display:none; max-height: 360px; object-fit: contain;"></video>

            <div class="grid grid-cols-2 gap-4 my-6 text-left">
                <div class="bg-bg-950 border border-[#2a2a2a] rounded-lg p-4">
                    <div class="text-xs text-gray-500 mb-2">Controls</div>
                    <div class="text-sm text-gray-300 space-y-1">
                        <div><kbd class="px-2 py-0.5 bg-[#1f1f1f] rounded font-mono">W</kbd> / <kbd class="px-2 py-0.5 bg-[#1f1f1f] rounded font-mono">S</kbd> — forward / back</div>
                        <div><kbd class="px-2 py-0.5 bg-[#1f1f1f] rounded font-mono">A</kbd> / <kbd class="px-2 py-0.5 bg-[#1f1f1f] rounded font-mono">D</kbd> — turn left / right</div>
                        <div><kbd class="px-2 py-0.5 bg-[#1f1f1f] rounded font-mono">Q</kbd> / <kbd class="px-2 py-0.5 bg-[#1f1f1f] rounded font-mono">E</kbd> — strafe left / right</div>
                        <div><kbd class="px-2 py-0.5 bg-[#1f1f1f] rounded font-mono">Shift</kbd> — 2× faster · <kbd class="px-2 py-0.5 bg-[#1f1f1f] rounded font-mono">Ctrl</kbd> — ½× slow</div>
                    </div>
                </div>
                <div class="bg-bg-950 border border-[#2a2a2a] rounded-lg p-4">
                    <div class="text-xs text-gray-500 mb-2">Live twist</div>
                    <pre id="twist-readout" class="text-sm text-green-300 font-mono">linear.x  = 0
linear.y  = 0
linear.z  = 0
angular.z = 0</pre>
                </div>
            </div>

            <div class="bg-bg-950 border border-[#2a2a2a] rounded-lg p-6 my-6">
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
                    <div class="flex gap-2 mt-3">
                        <div id="key-shift" class="kb-key wide">Shift</div>
                        <div id="key-ctrl" class="kb-key wide">Ctrl</div>
                    </div>
                </div>
            </div>

            <button id="disconnectBtn" class="mt-4 px-6 py-2.5 bg-[#2a2a2a] hover:bg-[#3a3a3a] text-white text-sm font-medium rounded-lg transition-colors">
                Disconnect
            </button>
        </div>
    </div>`;
    document.getElementById('disconnectBtn').onclick = disconnect;
}

function trackedKey(e) {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return null;
    const k = e.key.toLowerCase();
    if ('wasdqe'.includes(k)) return k;  // drive: WASD + Q/E strafe
    if (e.key === 'Shift' || e.key === 'Control') return e.key;  // 2× / 0.5×
    return null;
}
function onKeyDown(e) {
    const k = trackedKey(e);
    if (k === null) return;
    state.kbKeys.add(k);
    e.preventDefault();
}
function onKeyUp(e) {
    const k = trackedKey(e);
    if (k === null) return;
    state.kbKeys.delete(k);
}

// On-screen keys drive the same kbKeys set as the physical keyboard, so
// buildTwist() is unchanged. Press-and-hold to move; release/leave to stop.
const TOUCH_KEYS = {
    'key-w': 'w', 'key-a': 'a', 'key-s': 's', 'key-d': 'd',
    'key-q': 'q', 'key-e': 'e',
    'key-shift': 'Shift', 'key-ctrl': 'Control',
};
function bindTouchKeys() {
    for (const [id, key] of Object.entries(TOUCH_KEYS)) {
        const el = document.getElementById(id);
        if (!el) continue;
        const down = (e) => { e.preventDefault(); state.kbKeys.add(key); };
        const up = (e) => { e.preventDefault(); state.kbKeys.delete(key); };
        el.addEventListener('touchstart', down, { passive: false });
        el.addEventListener('touchend', up);
        el.addEventListener('touchcancel', up);
        el.addEventListener('mousedown', down);
        el.addEventListener('mouseup', up);
        el.addEventListener('mouseleave', up);  // drag-off = release
    }
}

function buildTwist() {
    // W/S forward-back, A/D turn, Q/E strafe. Shift = 2×, Ctrl = 0.5× (slow).
    const kb = state.kbKeys;
    const shift = kb.has('Shift') && !kb.has('Control');
    const ctrl  = kb.has('Control') && !kb.has('Shift');
    const fwd    = (kb.has('w') ? 1 : 0) - (kb.has('s') ? 1 : 0);
    const turn   = (kb.has('a') ? 1 : 0) - (kb.has('d') ? 1 : 0);
    const strafe = (kb.has('q') ? 1 : 0) - (kb.has('e') ? 1 : 0);

    const scale = shift ? 2.0 : (ctrl ? 0.5 : 1.0);
    // Speed-bar multiplier: state.js initializes {lin:0.5, ang:0.5} (Normal),
    // so the standalone keyboard view also drives at the safe Normal scale;
    // the go2 speed bar overrides it. (The || fallback only covers undefined.)
    const sp = state.speedScale || { lin: 0.5, ang: 0.5 };
    return {
        linear_x:  fwd * scale * sp.lin,
        linear_y:  strafe * scale * sp.lin,
        linear_z:  0,
        angular_z: turn * scale * sp.ang,
    };
}

function updateKeyVisuals() {
    const map = { 'w': 'key-w', 's': 'key-s', 'a': 'key-a', 'd': 'key-d',
                  'q': 'key-q', 'e': 'key-e', 'Shift': 'key-shift', 'Control': 'key-ctrl' };
    for (const [k, id] of Object.entries(map)) {
        const el = document.getElementById(id);
        if (el) el.classList.toggle('pressed', state.kbKeys.has(k));
    }
}

const DRIVE_KEYS = ['w', 'a', 's', 'd', 'q', 'e'];

export function startKeyboardLoop() {
    // Idempotent — callers re-render and would otherwise stack listeners.
    stopKeyboardLoop();
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
    bindTouchKeys();  // on-screen keys → same kbKeys set (phone/mouse)
    let twistSeq = 0;
    // Fresh gate per session — stall state must not leak across connects.
    const stallGate = createStallGate();
    state.videoStall = { stalled: false, blocked: false, armed: false };

    const sendTwist = (t) => {
        // Stamp in the robot's clock frame (clockOffsetMs is 0 until the first
        // pong lands; falls back gracefully on old brokers).
        const nowMs = Date.now() + state.clockOffsetMs;
        const ts = new std_msgs.Time({
            sec: Math.floor(nowMs / 1000),
            nsec: (nowMs % 1000) * 1_000_000,
        });
        twistSeq = (twistSeq + 1) & 0x7fffffff;
        const twist = new geometry_msgs.TwistStamped({
            header: new std_msgs.Header({ stamp: ts, frame_id: 'keyboard', seq: twistSeq }),
            twist: new geometry_msgs.Twist({
                linear: new geometry_msgs.Vector3({ x: t.linear_x, y: t.linear_y, z: t.linear_z }),
                angular: new geometry_msgs.Vector3({ x: 0, y: 0, z: t.angular_z }),
            }),
        });
        state.cmdChannel.send(twist.encode());
        state.cmdSendCount++;  // for cmdHz (operator send rate); sampled once/sec
    };

    state.kbInterval = setInterval(() => {
        // Always update key visuals; only send/readout when channel is up.
        updateKeyVisuals();
        // Video-freshness gate: driving blind on a frozen frame is the failure
        // mode. Auto-resumes when frames return, but only unblocks after all
        // drive keys are released (neutral gate) so a held W can't lunge the
        // robot the instant the picture unfreezes.
        const keysHeld = DRIVE_KEYS.some((k) => state.kbKeys.has(k));
        const wasBlocked = state.videoStall.blocked;
        state.videoStall = stallGate.sample(
            videoMediaTime(document.getElementById('robot-cam')),
            performance.now(),
            keysHeld,
        );
        if (!state.cmdChannel || state.cmdChannel.readyState !== 'open') return;
        if (state.videoStall.blocked) {
            // One explicit zero-twist on the transition; after that the robot's
            // 200ms cmd_vel deadman keeps it stopped while we stay silent.
            if (!wasBlocked) sendTwist({ linear_x: 0, linear_y: 0, linear_z: 0, angular_z: 0 });
            return;
        }
        // Cockpit gates drive on posture: WASD moves only after Stand/Drive.
        // Standalone keyboard view leaves driveEnabled=true, so unaffected.
        if (!state.driveEnabled) return;
        const t = buildTwist();
        sendTwist(t);
        const out = document.getElementById('twist-readout');
        if (out) out.textContent =
            `linear.x  = ${t.linear_x.toFixed(2)}\n` +
            `linear.y  = ${t.linear_y.toFixed(2)}\n` +
            `linear.z  = ${t.linear_z.toFixed(2)}\n` +
            `angular.z = ${t.angular_z.toFixed(2)}`;
    }, sendInterval);
}

export function stopKeyboardLoop() {
    window.removeEventListener('keydown', onKeyDown);
    window.removeEventListener('keyup', onKeyUp);
    if (state.kbInterval) { clearInterval(state.kbInterval); state.kbInterval = null; }
    state.kbKeys.clear();
}
