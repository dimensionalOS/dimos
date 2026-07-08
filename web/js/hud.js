// Live-metrics surface — browser DOM pill + in-headset WebGL stats quad. Both
// read state.liveStats and share the health classifier + line formatters.

import { state } from './state.js';

// Brand-aligned: warn → DimOS yellow, bad softened so it doesn't dominate.
// Green kept universal for "good" (traffic-light semantics).
const HUD_GOOD = '#34d399', HUD_WARN = '#ffcc00', HUD_BAD = '#ff5252';

// Drives the color dot in both surfaces. Command-plane (latency/loss) is the
// safety-relevant axis — a laggy command link is worse than a degraded picture.
// Also the signal a future stale-video drive-lockout will reuse.
export function statsHealth() {
    // A stalled feed means driving blind — worst state regardless of numbers.
    if (state.videoStall?.stalled) return 'bad';
    const v = state.liveStats.video;
    const c = state.liveStats.cmd;
    if (!v) return 'warn';
    const fps = v.fps ?? 0;
    const rtt = state.liveStats.rttMs ?? Infinity;
    const cmdLat = c ? (c.latency_ms ?? 0) : 0;
    // Go2 source is ~14–15fps, so 15 is a healthy link (not the 30 a webcam
    // gives). Bands scaled to that: bad < 8, warn < 12, good ≥ 12.
    if (fps < 6 || rtt > 200 || cmdLat > 200) return 'bad';
    if (fps < 12 || rtt > 100 || (v.loss_pct ?? 0) > 3 || cmdLat > 100) return 'warn';
    return 'good';
}
export function healthColor() {
    return { good: HUD_GOOD, warn: HUD_WARN, bad: HUD_BAD }[statsHealth()];
}

// Roll the shared send counter into cmdHz over `dtSec` and reset it. Called
// from each view's ~1Hz tick (DOM HUD, go2 cockpit, VR frame loop).
export function sampleCmdHz(dtSec) {
    if (dtSec > 0) state.liveStats.cmdHz = state.cmdSendCount / dtSec;
    state.cmdSendCount = 0;
}

// SOC → health level (good/warn/bad) — shared cutoffs; each view maps to its
// own colours (DOM hex vs VR palette).
export function socHealth(soc) {
    if (soc == null) return null;
    return soc > 40 ? 'good' : soc > 15 ? 'warn' : 'bad';
}

// SFU the operator is connected through.
export function transportLabel() {
    return state.activeRobot?.transport === 'livekit' ? 'LiveKit' : 'Cloudflare';
}

// ICE path the media/data actually traverses, once getStats() has a selected
// pair: direct (host) / STUN (srflx) / TURN (relay). '' until known.
export function iceTypeLabel() {
    const t = state.liveStats.iceType;
    return t === 'turn' ? 'TURN' : t === 'stun' ? 'STUN' : t === 'direct' ? 'direct' : '';
}

export function hudSummaryLine() {
    const v = state.liveStats.video;
    const c = state.liveStats.cmd;
    const fps = v ? `${(v.fps ?? 0).toFixed(0)}fps` : '—fps';
    const mbps = v ? `${(((v.kbps ?? 0) / 1000)).toFixed(1)}mbps` : '—mbps';
    const rtt = state.liveStats.rttMs != null ? `RTT ${state.liveStats.rttMs.toFixed(0)}ms` : 'RTT —';
    // Robot-measured command latency (what actually arrived) — the
    // safety-relevant number, so it earns a spot in the always-on line.
    const cmd = c && c.latency_ms != null ? `cmd ${c.latency_ms.toFixed(0)}ms` : 'cmd —';
    return `${fps}  ${mbps}  ${rtt}  ${cmd}`;
}

// Fuller detail lines (expand panel + VR quad body).
export function hudDetailLines() {
    const v = state.liveStats.video || {};
    const c = state.liveStats.cmd;
    const sentHz = state.liveStats.cmdHz ?? 0;  // operator's own send rate
    const cmdLine = c
        ? `lat ${(c.latency_ms ?? 0).toFixed(0)}ms  jit ${(c.jitter_ms ?? 0).toFixed(0)}ms`
        : '—';
    const rateLine = `sent ${sentHz.toFixed(0)}Hz` +
        (c ? ` → recv ${(c.rate_hz ?? 0).toFixed(0)}Hz` : '');
    const ice = iceTypeLabel();
    return [
        `Link   ${transportLabel()}${ice ? `  ·  ${ice}` : ''}`,
        `Video  ${(v.fps ?? 0).toFixed(0)}fps  ${(((v.kbps ?? 0) / 1000)).toFixed(1)}mbps  ${v.width ?? '—'}x${v.height ?? '—'}${v.codec ? '  ' + v.codec : ''}`,
        `       loss ${(v.loss_pct ?? 0).toFixed(1)}%  jbuf ${(v.jitter_buffer_ms ?? 0).toFixed(0)}ms`,
        `       decode ${(v.decode_ms ?? 0).toFixed(0)}ms  e2e ${v.e2e_latency_ms ? v.e2e_latency_ms.toFixed(0) + 'ms' : '—'}  freezes ${v.freezes ?? 0}`,
        `Cmd    ${cmdLine}`,
        `       ${rateLine}`,
        `Clock  RTT ${state.liveStats.rttMs != null ? state.liveStats.rttMs.toFixed(0) : '—'}ms`,
    ];
}

// Structured telemetry for the cockpit grid: groups of {label, value, health}.
// health ∈ 'good'|'warn'|'bad'|null — drives per-value tint; null = neutral.
export function hudDetailRows() {
    const v = state.liveStats.video || {};
    const c = state.liveStats.cmd;
    const rtt = state.liveStats.rttMs;
    const fps = v.fps ?? 0;
    const fmt = (n, d = 0) => (n == null ? '—' : n.toFixed(d));
    // Threshold helpers (mirror statsHealth axes).
    const band = (x, warn, bad, invert = false) =>
        x == null ? null : invert
            ? (x < bad ? 'bad' : x < warn ? 'warn' : 'good')
            : (x > bad ? 'bad' : x > warn ? 'warn' : 'good');
    return [
        { group: 'Link', rows: [
            { label: 'Transport', value: transportLabel(), health: null },
            { label: 'Path', value: iceTypeLabel() || '—',
              health: state.liveStats.iceType === 'turn' ? 'warn' : null },
            { label: 'RTT', value: `${fmt(rtt)} ms`, health: band(rtt, 100, 200) },
        ]},
        { group: 'Video', rows: [
            { label: 'FPS', value: fmt(fps), health: band(fps, 12, 6, true) },
            { label: 'Bitrate', value: `${fmt((v.kbps ?? 0) / 1000, 1)} mbps`, health: null },
            { label: 'Resolution', value: `${v.width ?? '—'}×${v.height ?? '—'}`, health: null },
            // H.264 → hardware decode (green); VP8 → usually software (warn).
            { label: 'Codec', value: v.codec || '—',
              health: v.codec ? (v.codec === 'H264' ? 'good' : 'warn') : null },
            { label: 'Loss', value: `${fmt(v.loss_pct, 1)} %`, health: band(v.loss_pct, 1, 3) },
            { label: 'Jitter buf', value: `${fmt(v.jitter_buffer_ms)} ms`, health: null },
            { label: 'Decode', value: `${fmt(v.decode_ms)} ms`, health: null },
            { label: 'E2E', value: v.e2e_latency_ms ? `${fmt(v.e2e_latency_ms)} ms` : '—',
              health: v.e2e_latency_ms ? band(v.e2e_latency_ms, 150, 300) : null },
            // freezeCount is a monotonic SESSION TOTAL, not a rate — a few over
            // a long drive is normal, so keep 1–3 green; warn past a handful,
            // bad only when it's clearly a bad link. (Ideally a per-window
            // delta like fps/loss; total is what getStats gives cheaply.)
            { label: 'Freezes', value: `${v.freezes ?? 0}`, health: band(v.freezes, 8, 20) },
        ]},
        { group: 'Command', rows: [
            { label: 'Latency', value: c ? `${fmt(c.latency_ms)} ms` : '—',
              health: c ? band(c.latency_ms, 100, 200) : null },
            { label: 'Jitter', value: c ? `${fmt(c.jitter_ms)} ms` : '—', health: null },
            { label: 'Send rate', value: `${fmt(state.liveStats.cmdHz)} Hz`, health: null },
            { label: 'Recv rate', value: c ? `${fmt(c.rate_hz)} Hz` : '—', health: null },
        ]},
    ];
}

// ─── Stamp-strip display crop ────────────────────────────────────────────
// The robot appends a 16px timestamp strip below the frame when benchmarking
// (latency_stamp). Hide it from the operator with a clip-path sized to the
// object-contain content rect — display-only, so the stamp decoder (which
// samples source pixels) keeps working. No-op (and clears itself) when the
// robot isn't stamping.
export function applyStampCrop() {
    const v = document.getElementById('robot-cam');
    if (!v) return;
    const strip = state.liveStats.stampStripPx || 0;
    // Skip while the camera is the floating PiP: the clip-path cuts through the
    // PiP's rounded box/border and the main-view map shows through the gap. The
    // strip is negligible at PiP size anyway; crop only when the camera is main.
    if (!strip || !v.videoWidth || !v.clientHeight || v.classList.contains('is-pip')) {
        if (v.style.clipPath) v.style.clipPath = '';
        return;
    }
    // object-contain: content is centered and scaled by min ratio; the strip
    // occupies the bottom strip*scale px of the content rect.
    const scale = Math.min(v.clientWidth / v.videoWidth, v.clientHeight / v.videoHeight);
    const padY = (v.clientHeight - v.videoHeight * scale) / 2;
    const bottom = padY + strip * scale;
    v.style.clipPath = `inset(0 0 ${bottom.toFixed(1)}px 0)`;
}

// ─── Browser HUD (DOM) ───────────────────────────────────────────────────
// Corner pill (click to expand). Mounted on connect, 1Hz refresh.
export function mountHud() {
    if (document.getElementById('live-hud')) return;
    const hud = document.createElement('div');
    hud.id = 'live-hud';
    // Always-on card: header (health · transport · robot) over live stats.
    hud.style.cssText =
        'position:fixed;top:12px;right:12px;z-index:50;font-family:ui-monospace,monospace;' +
        'user-select:none;background:rgba(21,21,21,0.92);border:1px solid #2a2a2a;' +
        'border-radius:10px;padding:10px 12px;color:#e5e7eb;font-size:12px;' +
        'backdrop-filter:blur(4px);min-width:236px;';
    hud.innerHTML = `
        <div style="display:flex;align-items:center;gap:8px;
            border-bottom:1px solid #2a2a2a;padding-bottom:7px;margin-bottom:7px;">
            <span id="live-hud-dot" style="width:9px;height:9px;border-radius:9999px;
                background:${HUD_WARN};"></span>
            <span id="live-hud-link" style="text-transform:uppercase;letter-spacing:0.1em;
                font-size:11px;font-weight:600;color:#b0e1f0;">—</span>
            <span id="live-hud-robot" style="margin-left:auto;color:#9ca3af;font-size:11px;"></span>
        </div>
        <pre id="live-hud-panel" style="margin:0;color:#cbd5e1;font-size:11px;
            line-height:1.6;white-space:pre;"></pre>`;
    document.body.appendChild(hud);

    let lastSampleMs = performance.now();
    state.hudTimer = setInterval(() => {
        const now = performance.now();
        sampleCmdHz((now - lastSampleMs) / 1000);
        lastSampleMs = now;
        refreshHud();
    }, 1000);
    refreshHud();
}

function refreshHud() {
    applyStampCrop();  // keyboard view shares the 1Hz tick
    const dot = document.getElementById('live-hud-dot');
    if (!dot) return;
    dot.style.background = healthColor();
    const link = document.getElementById('live-hud-link');
    if (link) link.textContent = transportLabel();
    const robot = document.getElementById('live-hud-robot');
    if (robot) robot.textContent = state.activeRobot?.robot_name || '';
    const panel = document.getElementById('live-hud-panel');
    // Link line is in the header.
    if (panel) panel.textContent = hudDetailLines().slice(1).join('\n');
}

export function unmountHud() {
    if (state.hudTimer) { clearInterval(state.hudTimer); state.hudTimer = null; }
    const hud = document.getElementById('live-hud');
    if (hud) hud.remove();
}
