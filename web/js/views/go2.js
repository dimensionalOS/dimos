// Go2 teleop cockpit — big video left, control column right.
//
// LAYOUT ONLY for now. The posture/action/body-height controls and the
// telemetry/battery readouts are wired to a local placeholder state so the
// view is demonstrable, but nothing talks to the robot yet — functionality
// gets added one piece at a time (commands over state_reliable, real telemetry
// over state_reliable_back, video track). Drive (WASD) reuses the existing
// keyboard loop / cmdChannel exactly as views/keyboard.js does.
//
// Preview with no broker:  window._teleopDev.previewGo2()

import { disconnect } from '../disconnect.js';
import { escHtml, state } from '../state.js';
import { startKeyboardLoop, stopKeyboardLoop } from './keyboard.js';

// Command catalog — labels only; SPORT_CMD ids live robot-side.
const POSTURE = [
    { name: 'StandUp', label: 'Stand Up' },
    { name: 'StandDown', label: 'Sit / Down' },
    { name: 'BalanceStand', label: 'Balance' },
    { name: 'RecoveryStand', label: 'Recovery' },
];
const ACTIONS = [
    { name: 'Hello', label: 'Hello' },
    { name: 'Stretch', label: 'Stretch' },
    { name: 'Sit', label: 'Sit' },
    { name: 'Pose', label: 'Pose' },
];

// Local placeholder UI state (replaced by real telemetry when wired).
const ui = {
    posture: 'StandUp',
    estopped: false,
    bodyHeight: 0,
    // demo telemetry until state_reliable_back is parsed here
    soc: 82,
};

let tickTimer = null;

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
                <div class="relative flex-1 bg-black flex items-center justify-center min-h-0">
                    <video id="robot-cam" autoplay muted playsinline
                        class="w-full h-full object-contain" style="display:none;"></video>
                    <div id="video-placeholder" class="text-center text-gray-600">
                        <div class="text-6xl mb-2">🐕</div>
                        <div class="term-caps text-xs">robot camera</div>
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

                <!-- Battery -->
                <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl p-4 shrink-0">
                    <div class="flex items-center justify-between mb-2">
                        <span class="term-caps text-xs text-gray-500">Battery</span>
                        <span id="batt-pct" class="text-sm font-semibold text-dim-400">—%</span>
                    </div>
                    <div class="h-2.5 w-full bg-[#1f1f1f] rounded-full overflow-hidden">
                        <div id="batt-bar" class="h-full rounded-full transition-all duration-500" style="width:0%;background:#b0e1f0;"></div>
                    </div>
                </section>

                <!-- Telemetry -->
                <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl p-4 shrink-0">
                    <div class="flex items-center justify-between mb-3">
                        <span class="term-caps text-xs text-gray-500">Telemetry</span>
                        <span id="hud-health" class="pill pill-good"><span class="dot"></span><span>Cloudflare</span></span>
                    </div>
                    <pre id="hud-panel" class="text-xs text-gray-400 leading-relaxed">—</pre>
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

                <!-- Body height -->
                <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl p-4 shrink-0">
                    <label class="flex items-center justify-between text-xs text-gray-400 mb-1">
                        <span class="term-caps text-gray-500">Body height</span><span id="bh-val" class="text-dim-400">0.00</span>
                    </label>
                    <input id="body-height" type="range" min="-0.18" max="0.03" step="0.01" value="0" class="w-full accent-dim-500">
                </section>

                <!-- E-STOP -->
                <section class="mt-auto bg-bg-950 border border-[#2a2a2a] rounded-xl p-4 shrink-0">
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
    // state.cmdChannel (same as views/keyboard.js). Writes #twist-readout; the
    // #key-* visual updates no-op here (elements absent, guarded).
    startKeyboardLoop();
}

// ── interaction (placeholder — no robot calls yet) ──────────────────
function wireGo2() {
    document.getElementById('disconnectBtn').onclick = disconnect;

    document.querySelectorAll('.cmd-btn[data-cmd]').forEach((b) =>
        b.addEventListener('click', () => sendCommand(b.dataset.cmd, b)));

    document.getElementById('estop').addEventListener('click', () => {
        ui.estopped = true; // TODO send {type:'estop'} on state_reliable
        document.querySelectorAll('.cmd-btn').forEach((b) => (b.dataset.status = 'idle'));
        refreshControls();
    });
    document.getElementById('rearm').addEventListener('click', () => {
        ui.estopped = false; // TODO real two-step re-arm
        refreshControls();
    });

    const bh = document.getElementById('body-height');
    bh.addEventListener('input', () => {
        ui.bodyHeight = +bh.value;
        document.getElementById('bh-val').textContent = ui.bodyHeight.toFixed(2);
    });
    // TODO on 'change': send {type:'sport_cmd', name:'BodyHeight', value:ui.bodyHeight}
}

// Placeholder command: locally flips status so the UI is demonstrable. The real
// version sends a nonce envelope on state_reliable and resolves on cmd_ack.
function sendCommand(name, btn) {
    if (ui.estopped) return;
    btn.dataset.status = 'pending';
    setTimeout(() => {
        if (POSTURE.some((p) => p.name === name)) ui.posture = name;
        btn.dataset.status = 'done';
        setTimeout(() => {
            btn.dataset.status = 'idle';
            refreshControls();
        }, 700);
        refreshControls();
    }, 350);
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
        b.disabled = !!reason || (active && b.dataset.status === 'idle');
    });

    document.getElementById('estop').classList.toggle('latched', ui.estopped);
    document.getElementById('rearm').classList.toggle('hidden', !ui.estopped);

    const kb = document.getElementById('kb-live');
    kb.className = 'pill ' + (ui.estopped ? 'pill-bad' : 'pill-good');
    kb.querySelector('.dot').nextSibling.textContent = ui.estopped ? 'KEYBOARD OFF' : 'KEYBOARD LIVE';

    document.getElementById('posture-chip').textContent =
        ({ StandUp: 'STANDING', StandDown: 'SITTING', BalanceStand: 'BALANCE', RecoveryStand: 'RECOVERY' }[ui.posture]) ||
        ui.posture;

    renderBattery();
}

function renderBattery() {
    const p = Math.max(0, Math.min(100, ui.soc));
    const pct = document.getElementById('batt-pct');
    const bar = document.getElementById('batt-bar');
    if (!pct || !bar) return;
    pct.textContent = `${p}%`;
    bar.style.width = `${p}%`;
    bar.style.background = p > 40 ? '#b0e1f0' : p > 15 ? '#eab308' : '#d97777';
    pct.style.color = p > 40 ? '#c4e7f3' : p > 15 ? '#eab308' : '#f3b4b4';
}

// Telemetry tick — for now reuses state.liveStats if present (real HUD data),
// else shows placeholder. Battery SOC is placeholder until robot_telemetry
// carries bms_state.soc.
function startTick() {
    stopTick();
    tickTimer = setInterval(() => {
        const v = state.liveStats?.video;
        const c = state.liveStats?.cmd;
        const rtt = state.liveStats?.rttMs;
        document.getElementById('hud-panel').textContent = [
            `Video  ${v ? (v.fps ?? 0).toFixed(0) : '—'}fps  ${v ? ((v.kbps ?? 0) / 1000).toFixed(1) : '—'}mbps`,
            `Cmd    ${c ? `lat ${(c.latency_ms ?? 0).toFixed(0)}ms  loss ${(c.loss_pct ?? 0).toFixed(1)}%` : '—'}`,
            `Clock  RTT ${rtt != null ? rtt.toFixed(0) : '—'}ms`,
        ].join('\n');
        renderBattery();
    }, 1000);
}

export function stopTick() {
    if (tickTimer) {
        clearInterval(tickTimer);
        tickTimer = null;
    }
}
