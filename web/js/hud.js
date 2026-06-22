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
    const v = state.liveStats.video;
    const c = state.liveStats.cmd;
    if (!v) return 'warn';
    const fps = v.fps ?? 0;
    const rtt = state.liveStats.rttMs ?? Infinity;
    const cmdLat = c ? (c.latency_ms ?? 0) : 0;
    if (fps < 8 || rtt > 200 || cmdLat > 200) return 'bad';
    if (fps < 18 || rtt > 100 || (v.loss_pct ?? 0) > 3 || cmdLat > 100) return 'warn';
    return 'good';
}
export function healthColor() {
    return { good: HUD_GOOD, warn: HUD_WARN, bad: HUD_BAD }[statsHealth()];
}

// SFU the operator is connected through.
export function transportLabel() {
    return state.activeRobot?.transport === 'livekit' ? 'LiveKit' : 'Cloudflare';
}

export function hudSummaryLine() {
    const v = state.liveStats.video;
    const fps = v ? `${(v.fps ?? 0).toFixed(0)}fps` : '—fps';
    const mbps = v ? `${(((v.kbps ?? 0) / 1000)).toFixed(1)}mbps` : '—mbps';
    const rtt = state.liveStats.rttMs != null ? `RTT ${state.liveStats.rttMs.toFixed(0)}ms` : 'RTT —';
    return `${fps}  ${mbps}  ${rtt}`;
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
    return [
        `Link   ${transportLabel()}`,
        `Video  ${(v.fps ?? 0).toFixed(0)}fps  ${(((v.kbps ?? 0) / 1000)).toFixed(1)}mbps  ${v.width ?? '—'}x${v.height ?? '—'}`,
        `       loss ${(v.loss_pct ?? 0).toFixed(1)}%  jbuf ${(v.jitter_buffer_ms ?? 0).toFixed(0)}ms`,
        `       decode ${(v.decode_ms ?? 0).toFixed(0)}ms  freezes ${v.freezes ?? 0}`,
        `Cmd    ${cmdLine}`,
        `       ${rateLine}`,
        `Clock  RTT ${state.liveStats.rttMs != null ? state.liveStats.rttMs.toFixed(0) : '—'}ms`,
    ];
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
        // Sample cmd rate from the send() counter (kb twists + VR poses).
        const now = performance.now();
        const dt = (now - lastSampleMs) / 1000;
        if (dt > 0) state.liveStats.cmdHz = state.cmdSendCount / dt;
        state.cmdSendCount = 0;
        lastSampleMs = now;
        refreshHud();
    }, 1000);
    refreshHud();
}

function refreshHud() {
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

// ─── VR stats quad (WebGL) ───────────────────────────────────────────────
// XR has no DOM, so stats render to a 2D canvas → texture → small dimmed
// quad pinned to the video quad's upper-right corner in world space.
export function initStatsQuad() {
    const gl = state.gl;
    state.statsCanvas = document.createElement('canvas');
    state.statsCanvas.width = 512; state.statsCanvas.height = 256;
    state.statsCtx = state.statsCanvas.getContext('2d');

    state.statsTex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, state.statsTex);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);

    // Smaller panel (0.34m wide, 2:1 to match the 512x256 canvas); its own
    // buffer; reuses the video quad's shader program + attributes.
    const w = 0.17, h = 0.085;
    const verts = new Float32Array([
        -w, -h, 0, 1,   w, -h, 1, 1,   w, h, 1, 0,
        -w, -h, 0, 1,   w, h, 1, 0,   -w, h, 0, 0,
    ]);
    state.statsBuf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, state.statsBuf);
    gl.bufferData(gl.ARRAY_BUFFER, verts, gl.STATIC_DRAW);
}

// ?vrdebug=1 → panel dead-center + opaque red, to distinguish "drawing
// off-screen" from "not drawing".
const VR_HUD_DEBUG = new URLSearchParams(location.search).has('vrdebug');

function renderStatsToCanvas() {
    const ctx = state.statsCtx, W = state.statsCanvas.width, H = state.statsCanvas.height;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = VR_HUD_DEBUG ? 'rgba(220,38,38,1.0)' : 'rgba(21,21,21,0.62)';
    ctx.fillRect(0, 0, W, H);

    ctx.fillStyle = healthColor();
    ctx.beginPath(); ctx.arc(28, 34, 11, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#e5e7eb';
    ctx.font = '600 26px ui-monospace, monospace';
    ctx.fillText(hudSummaryLine(), 50, 43);

    ctx.fillStyle = '#b0e1f0';  // brand pale-cyan for detail lines
    ctx.font = '22px ui-monospace, monospace';
    const lines = hudDetailLines();
    for (let i = 0; i < lines.length; i++) ctx.fillText(lines[i], 16, 86 + i * 30);
}

// Offset from the video quad's center (in its local frame) — upper-right
// corner, slightly forward to avoid depth-fighting.
const STATS_OFFSET_X = 0.40;
const STATS_OFFSET_Y = 0.22;
const STATS_OFFSET_Z = 0.05;

export function drawStatsQuad(frame, glLayer, mat4mul, videoQuadWorldModel) {
    if (!state.statsBuf || !videoQuadWorldModel) return;
    const pose = frame.getViewerPose(state.xrRefSpace);
    if (!pose) return;

    const gl = state.gl;
    renderStatsToCanvas();
    gl.disable(gl.DEPTH_TEST);  // HUD overlay, like the video quad
    gl.bindTexture(gl.TEXTURE_2D, state.statsTex);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, state.statsCanvas);

    gl.useProgram(state.quadProgram);
    gl.bindBuffer(gl.ARRAY_BUFFER, state.statsBuf);
    gl.enableVertexAttribArray(state.quadUniforms.aPos);
    gl.vertexAttribPointer(state.quadUniforms.aPos, 2, gl.FLOAT, false, 16, 0);
    gl.enableVertexAttribArray(state.quadUniforms.aUV);
    gl.vertexAttribPointer(state.quadUniforms.aUV, 2, gl.FLOAT, false, 16, 8);
    gl.uniform1i(state.quadUniforms.tex, 0);

    // Child of the video quad: statsModel = videoModel * offset.
    const offset = new Float32Array([
        1,0,0,0, 0,1,0,0, 0,0,1,0,
        VR_HUD_DEBUG ? 0 : STATS_OFFSET_X,
        VR_HUD_DEBUG ? 0 : STATS_OFFSET_Y,
        VR_HUD_DEBUG ? 0 : STATS_OFFSET_Z,
        1,
    ]);
    const statsWorldModel = mat4mul(videoQuadWorldModel, offset);

    for (const view of pose.views) {
        const vp = glLayer.getViewport(view);
        gl.viewport(vp.x, vp.y, vp.width, vp.height);
        const viewProj = mat4mul(view.projectionMatrix, view.transform.inverse.matrix);
        const mvp = mat4mul(viewProj, statsWorldModel);
        gl.uniformMatrix4fv(state.quadUniforms.mvp, false, mvp);
        gl.drawArrays(gl.TRIANGLES, 0, 6);
    }
}
