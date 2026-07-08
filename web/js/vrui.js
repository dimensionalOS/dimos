// VR cockpit UI — the Go2 cockpit (go2.js) as spatial canvas panels.
// Each panel is a 2D canvas on a three plane; interaction is controller-ray →
// panel UV → canvas px → hit-test against registered rects. Commands are the
// same JSON go2.js sends, so acks/telemetry flow back through the shared
// state.onCmdAck / state.onRobotState hooks. Layout: MAP left · CAMERA centre ·
// BUTTONS right · STATS far right.

import * as THREE from 'three';

import { CONFIRM_ACTIONS, POSTURE_STATE, SPEEDS, sendEstop, sendEstopClear } from './go2cmd.js';
import { hudDetailRows, healthColor, socHealth, statsHealth, transportLabel } from './hud.js';
import { state } from './state.js';

// Palette (matches index.html cockpit CSS + hud.js).
const C = {
    bg: 'rgba(18,19,19,0.92)', bgSolid: '#121313', panel: '#0d0e0e',
    line: '#2a2a2a', text: '#d1d5db', dim: '#6b7280', cyan: '#b0e1f0',
    cyanDim: '#3d6a7a', good: '#34d399', warn: '#ffcc00', bad: '#ff5252',
    estopBg: '#7a1f1f', estopBorder: '#d97777',
};
const HEALTH = { good: C.good, warn: C.warn, bad: C.bad };

// Display catalogs — VR uses tighter labels than the DOM cockpit, so these
// stay local; the shared name/scale/confirm/posture data is in go2cmd.js.
const POSTURE = [
    { name: 'StandReady', label: 'Stand / Drive' },
    { name: 'StandDown', label: 'Sit' },
];
const ACTIONS = [
    { name: 'Hello', label: 'Shake Hand' },
    { name: 'Stretch', label: 'Stretch' },
    { name: 'FrontPounce', label: 'Pounce' },
    { name: 'FrontJump', label: 'Jump Fwd' },
];
const CAMS = [{ id: 'cam1', label: 'Cam 1' }, { id: 'cam2', label: 'Cam 2' }];
// Discrete light presets — a raycast slider is fiddly; presets read cleanly.
const LIGHTS = [
    { label: 'Off', v: 0 }, { label: 'Low', v: 0.34 },
    { label: 'Med', v: 0.67 }, { label: 'Full', v: 1.0 },
];
const GO2_LEN_M = 0.70, GO2_WID_M = 0.31;  // footprint, matches cockpit glyph

// ── Shared cockpit UI state (mirror of go2.js `ui`) ──────────────────
export const vui = {
    posture: 'StandReady', estopped: false, speedMode: 'normal',
    selectedCams: ['cam1'], obstacleAvoid: true, light: 0,
    robotVideoStalled: false, nonce: 0,
    pending: new Map(),        // nonce → { region, panel, timer }
    confirm: null,             // { name, expiry } two-press guard for acrobatics
    lastMap: null, lastOdom: null, navGoal: null,
};

function chanReady() {
    return state.stateChannel && state.stateChannel.readyState === 'open';
}
function sendJSON(obj) { if (chanReady()) state.stateChannel.send(JSON.stringify(obj)); }

// ── Panel: canvas → CanvasTexture → plane, with hit regions ──────────
class Panel {
    constructor({ wM, hM, cw, ch, opacity = 1 }) {
        this.cw = cw; this.ch = ch;
        this.canvas = document.createElement('canvas');
        this.canvas.width = cw; this.canvas.height = ch;
        this.ctx = this.canvas.getContext('2d');
        this.tex = new THREE.CanvasTexture(this.canvas);
        this.tex.colorSpace = THREE.SRGBColorSpace;
        this.mesh = new THREE.Mesh(
            new THREE.PlaneGeometry(wM, hM),
            new THREE.MeshBasicMaterial({ map: this.tex, transparent: true, opacity }),
        );
        this.mesh.userData.panel = this;
        this.regions = [];     // { id, x, y, w, h }
        this.hoverId = null;
        this.dirty = true;
    }
    place(pos, headPos) {
        this.mesh.position.copy(pos);
        this.mesh.lookAt(headPos);  // plane front (+Z) faces the user
    }
    // Console placement: fixed tilt about X (top edge away), no lookAt — a
    // desk panel angled up toward the operator's eyes.
    placeFlat(pos, rotX) {
        this.mesh.position.copy(pos);
        this.mesh.rotation.set(rotX, 0, 0);
    }
    // Fold about Y, positioning so this panel's edge (edgeSign +1 right /
    // -1 left) lands exactly on `hinge` — center = hinge − rotated edge — so
    // adjacent folded panels' seams coincide at any width/angle.
    placeHinged(hinge, edgeSign, yaw) {
        this.mesh.rotation.set(0, yaw, 0);
        const h = this.mesh.geometry.parameters.width / 2;
        const edge = new THREE.Vector3(edgeSign * h, 0, 0).applyEuler(new THREE.Euler(0, yaw, 0));
        this.mesh.position.copy(hinge).sub(edge);
    }
    markDirty() { this.dirty = true; }
    // UV (three) → canvas px. VideoTexture/CanvasTexture are flipY, so v=0 is
    // the bottom row; canvas y grows down → row = (1-v)*ch.
    hitTest(uv) {
        const px = uv.x * this.cw, py = (1 - uv.y) * this.ch;
        for (const r of this.regions) {
            if (px >= r.x && px <= r.x + r.w && py >= r.y && py <= r.y + r.h) return r.id;
        }
        return null;
    }
    setHover(id) { if (id !== this.hoverId) { this.hoverId = id; this.dirty = true; } }

    // ── drawing primitives ──
    bg() {
        const x = this.ctx;
        x.clearRect(0, 0, this.cw, this.ch);
        x.fillStyle = C.bg;
        roundRect(x, 2, 2, this.cw - 4, this.ch - 4, 18); x.fill();
        x.strokeStyle = C.line; x.lineWidth = 2; x.stroke();
        this.regions = [];
    }
    header(text) {
        const x = this.ctx;
        x.fillStyle = C.dim;
        x.font = '600 22px ui-monospace, monospace';
        x.fillText(text.toUpperCase(), 24, 40);
    }
    // A chip button; `st` ∈ idle|active|pending|done|error|confirm.
    chip(id, bx, by, bw, bh, label, st = 'idle') {
        const x = this.ctx, hot = this.hoverId === id;
        let fill = C.bgSolid, border = C.line, txt = C.text;
        if (st === 'active') { fill = C.cyan; border = C.cyan; txt = C.panel; }
        else if (st === 'done') { fill = C.cyan; border = C.cyan; txt = C.panel; }
        else if (st === 'error') { fill = '#4a1d1d'; border = C.estopBorder; txt = '#f3b4b4'; }
        else if (st === 'pending') { border = C.cyanDim; txt = C.dim; }
        else if (st === 'confirm') { fill = '#4a3a1d'; border = C.warn; txt = C.warn; }
        if (hot && st === 'idle') border = C.cyan;
        x.fillStyle = fill; roundRect(x, bx, by, bw, bh, 10); x.fill();
        x.lineWidth = hot ? 2.5 : 1.5; x.strokeStyle = border; x.stroke();
        x.fillStyle = txt;
        x.font = '600 20px ui-monospace, monospace';
        x.textAlign = 'center'; x.textBaseline = 'middle';
        x.fillText(label, bx + bw / 2, by + bh / 2 + 1);
        x.textAlign = 'left'; x.textBaseline = 'alphabetic';
        this.regions.push({ id, x: bx, y: by, w: bw, h: bh });
    }
    dispose() {
        this.tex.dispose();
        this.mesh.geometry.dispose();
        this.mesh.material.dispose();
    }
}

function roundRect(x, bx, by, bw, bh, r) {
    x.beginPath();
    x.moveTo(bx + r, by);
    x.arcTo(bx + bw, by, bx + bw, by + bh, r);
    x.arcTo(bx + bw, by + bh, bx, by + bh, r);
    x.arcTo(bx, by + bh, bx, by, r);
    x.arcTo(bx, by, bx + bw, by, r);
    x.closePath();
}

// ── Command senders (mirror go2.js; add ack tracking per region) ─────
function nonceCmd(panel, region, obj, timeoutMs = 3000) {
    const nonce = ++vui.nonce;
    obj.nonce = nonce;
    sendJSON(obj);
    setRegionState(panel, region, 'pending');
    const timer = setTimeout(() => resolveAck(nonce, false), timeoutMs);
    vui.pending.set(nonce, { panel, region, timer });
    return nonce;
}

function sendSportCmd(name, panel, region) {
    if (!chanReady() || vui.estopped) return;
    if (CONFIRM_ACTIONS.has(name)) {
        // Two-press confirm (no confirm() dialog in XR): first press arms.
        const now = performance.now();
        if (!vui.confirm || vui.confirm.name !== name || now > vui.confirm.expiry) {
            vui.confirm = { name, expiry: now + 4000 };
            panel.markDirty();
            return;
        }
        vui.confirm = null;
    }
    nonceCmd(panel, region, { type: 'sport_cmd', name },
        name === 'StandReady' ? 9000 : 3000);
}

function selectSpeed(mode, send = true) {
    const spec = SPEEDS.find((s) => s.mode === mode);
    if (!spec) return;
    vui.speedMode = mode;
    state.speedScale = spec.scale;
    if (send) sendJSON({ type: 'set_mode', mode, nonce: ++vui.nonce });
}

function toggleCam(id, panel) {
    const sel = new Set(vui.selectedCams);
    if (sel.has(id)) { if (sel.size === 1) return; sel.delete(id); } else sel.add(id);
    vui.selectedCams = CAMS.map((c) => c.id).filter((cid) => sel.has(cid));
    sendJSON({ type: 'camera_select', cams: vui.selectedCams });
    panel.markDirty();
}

function toggleObstacle(panel, region) {
    if (!chanReady()) return;
    vui.obstacleAvoid = !vui.obstacleAvoid;
    nonceCmd(panel, region, { type: 'obstacle_avoidance', enabled: vui.obstacleAvoid });
}

function setLight(v, panel, region) {
    if (!chanReady()) return;
    vui.light = v;
    nonceCmd(panel, region, { type: 'light', brightness: v });
}

function estop(panel) {
    vui.estopped = true;
    sendEstop(state.stateChannel, () => ++vui.nonce);
    vui.posture = 'Damp';
    panel.markDirty();
}
function rearm(panel) {
    vui.estopped = false;
    sendEstopClear(state.stateChannel, () => ++vui.nonce);
    panel.markDirty();
}

// ── Ack + robot-state reconcile (registered on state.* by vr.js) ─────
function setRegionState(panel, region, st) {
    panel._rstate = panel._rstate || {};
    panel._rstate[region] = st;
    panel.markDirty();
}
function resolveAck(nonce, ok) {
    const p = vui.pending.get(nonce);
    if (!p) return;
    clearTimeout(p.timer);
    vui.pending.delete(nonce);
    setRegionState(p.panel, p.region, ok ? 'done' : 'error');
    setTimeout(() => { setRegionState(p.panel, p.region, 'idle'); }, 700);
}
export function onCmdAck(msg) { resolveAck(msg.nonce, !!msg.ok); }

export function onRobotState(s) {
    if (vui.pending.size > 0) return;  // don't fight a command mid-flight
    if (typeof s.posture === 'string' && POSTURE_STATE[s.posture]) vui.posture = s.posture;
    if (typeof s.estopped === 'boolean') vui.estopped = s.estopped;
    if (typeof s.obstacle_avoidance === 'boolean') vui.obstacleAvoid = s.obstacle_avoidance;
    if (typeof s.light === 'number') vui.light = s.light;
    if (Array.isArray(s.cams) && s.cams.length) vui.selectedCams = s.cams.filter((c) => CAMS.some((k) => k.id === c));
    if (typeof s.rage === 'boolean') vui.speedMode = s.rage ? 'rage' : (vui.speedMode === 'rage' ? 'normal' : vui.speedMode);
    if (typeof s.video_stalled === 'boolean') vui.robotVideoStalled = s.video_stalled;
    // Drive gate mirrors the cockpit: only StandReady drives.
    state.driveEnabled = vui.posture === 'StandReady' && !vui.estopped;
    for (const pnl of _allPanels) pnl.markDirty();
}

// ── Minimap: decode the base64-PNG occupancy grid into a texture ─────
export function onMap(msg) {
    if (!msg || !msg.png_b64) return;
    const img = new Image();
    img.onload = () => { vui.lastMap = { ...msg, img }; _mapPanel?.markDirty(); };
    img.onerror = () => {};
    img.src = 'data:image/png;base64,' + msg.png_b64;
}
export function onOdom(msg) { if (msg) { vui.lastOdom = msg; _mapPanel?.markDirty(); } }

// ─────────────────────────────────────────────────────────────────────
// Panel renderers
// ─────────────────────────────────────────────────────────────────────

// Horizontal operator console (tilted shelf under the camera panel).
// Two bands of grouped chips + a full-height E-STOP block on the right:
//   band 1: POSTURE ·· SPEED ·· CAMERAS ·· OBSTACLE
//   band 2: ACTIONS ·········· LIGHT
function renderConsole(p) {
    p.bg();
    const x = p.ctx;
    const st = (region, active) => {
        const s = (p._rstate && p._rstate[region]) || 'idle';
        return s !== 'idle' ? s : (active ? 'active' : 'idle');
    };

    // Status strip (top-left): drive state + posture, like the cockpit pills.
    const driving = state.driveEnabled && !vui.estopped;
    x.fillStyle = driving ? C.good : C.bad;
    x.beginPath(); x.arc(34, 34, 7, 0, Math.PI * 2); x.fill();
    x.fillStyle = driving ? C.good : '#f3b4b4';
    x.font = '600 19px ui-monospace,monospace';
    x.fillText(vui.estopped ? 'E-STOPPED' : driving ? 'DRIVE LIVE' : 'DRIVE OFF — press Stand / Drive', 50, 41);
    x.fillStyle = C.dim; x.textAlign = 'right';
    x.fillText(({ StandReady: 'STANDING', StandDown: 'SITTING', Damp: 'STOPPED', Sit: 'SITTING' }[vui.posture]) || vui.posture, 900, 41);
    x.textAlign = 'left';

    const header = (t, hx, hy) => {
        x.fillStyle = C.dim; x.font = '600 16px ui-monospace,monospace';
        x.fillText(t.toUpperCase(), hx, hy);
    };
    // Lay a group of chips horizontally; returns x after the group.
    const group = (label, items, gx, gy, bw, bh, render) => {
        header(label, gx, gy - 10);
        items.forEach((it, i) => render(it, gx + i * (bw + 10), gy, bw, bh));
        return gx + items.length * (bw + 10) - 10 + 26;
    };

    // ── band 1 ──
    let gx = 24;
    const B1 = 86, H = 62;
    gx = group('Posture', POSTURE, gx, B1, 128, H, (it, bx, by, bw, bh) => {
        const active = vui.posture === it.name;
        p.chip('sport:' + it.name, bx, by, bw, bh, it.label, st('sport:' + it.name, active));
    });
    gx = group('Speed', SPEEDS, gx, B1, 84, H, (it, bx, by, bw, bh) =>
        p.chip('speed:' + it.mode, bx, by, bw, bh, it.label, vui.speedMode === it.mode ? 'active' : 'idle'));
    gx = group('Cameras', CAMS, gx, B1, 84, H, (it, bx, by, bw, bh) =>
        p.chip('cam:' + it.id, bx, by, bw, bh, it.label, vui.selectedCams.includes(it.id) ? 'active' : 'idle'));
    group('Obstacle', [{}], gx, B1, 84, H, (it, bx, by, bw, bh) =>
        p.chip('obstacle', bx, by, bw, bh, vui.obstacleAvoid ? 'ON' : 'OFF', st('obstacle', vui.obstacleAvoid)));

    // ── band 2 ──
    gx = 24;
    const B2 = 216;
    gx = group('Actions', ACTIONS, gx, B2, 115, H, (it, bx, by, bw, bh) => {
        const confirming = vui.confirm && vui.confirm.name === it.name && performance.now() < vui.confirm.expiry;
        p.chip('sport:' + it.name, bx, by, bw, bh,
            confirming ? 'Confirm?' : it.label,
            confirming ? 'confirm' : st('sport:' + it.name, false));
    });
    group('Light', LIGHTS, gx, B2, 84, H, (it, bx, by, bw, bh) => {
        const active = Math.abs(vui.light - it.v) < 0.08;
        p.chip('light:' + it.v, bx, by, bw, bh, it.label, active ? 'active' : 'idle');
    });

    // Drive hint (bottom edge, small).
    x.fillStyle = C.dim; x.font = '15px ui-monospace,monospace';
    x.fillText('left stick drive · right stick turn · grip boost/slow · B/Y = E-STOP', 24, p.ch - 16);

    // ── E-STOP block: full-height, far right, always in reach ──
    const ex = 940, ey = 28, ew = p.cw - ex - 24, eh = p.ch - 56;
    x.fillStyle = vui.estopped ? C.estopBorder : C.estopBg;
    roundRect(x, ex, ey, ew, eh, 14); x.fill();
    x.lineWidth = 3; x.strokeStyle = vui.estopped ? '#fff' : C.estopBorder; x.stroke();
    x.fillStyle = '#fff'; x.textAlign = 'center'; x.textBaseline = 'middle';
    if (vui.estopped) {
        x.font = '700 30px ui-monospace,monospace';
        x.fillText('STOPPED', ex + ew / 2, ey + eh / 2 - 40);
        x.textAlign = 'left'; x.textBaseline = 'alphabetic';
        p.regions.push({ id: 'estop', x: ex, y: ey, w: ew, h: eh - 90 });
        p.chip('rearm', ex + 14, ey + eh - 72, ew - 28, 54, 're-arm →', 'idle');
    } else {
        x.font = '700 34px ui-monospace,monospace';
        x.fillText('■', ex + ew / 2, ey + eh / 2 - 26);
        x.font = '700 24px ui-monospace,monospace';
        x.fillText('E-STOP', ex + ew / 2, ey + eh / 2 + 16);
        x.textAlign = 'left'; x.textBaseline = 'alphabetic';
        p.regions.push({ id: 'estop', x: ex, y: ey, w: ew, h: eh });
    }
}

function renderStats(p) {
    p.bg();
    const x = p.ctx;
    // Health dot + transport header.
    x.fillStyle = healthColor();
    x.beginPath(); x.arc(30, 34, 9, 0, Math.PI * 2); x.fill();
    x.fillStyle = C.text; x.font = '600 20px ui-monospace,monospace';
    x.fillText(transportLabel(), 48, 40);
    const soc = state.liveStats?.soc;
    x.fillStyle = soc == null ? C.dim : { good: C.cyan, warn: C.warn, bad: C.bad }[socHealth(soc)];
    x.textAlign = 'right'; x.fillText(soc == null ? '—%' : `${Math.round(soc)}%`, p.cw - 24, 40);
    x.textAlign = 'left';

    let y = 72;
    for (const g of hudDetailRows()) {
        x.fillStyle = C.dim; x.font = '600 15px ui-monospace,monospace';
        x.fillText(g.group.toUpperCase(), 24, y); y += 22;
        for (const r of g.rows) {
            x.fillStyle = C.dim; x.font = '15px ui-monospace,monospace';
            x.fillText(r.label, 30, y);
            x.fillStyle = HEALTH[r.health] || C.text;
            x.textAlign = 'right'; x.font = '600 15px ui-monospace,monospace';
            x.fillText(String(r.value), p.cw - 24, y);
            x.textAlign = 'left';
            y += 22;
        }
        y += 8;
    }
}

// Draw the decoded occupancy grid + robot glyph + nav goal. Mirrors go2.js
// drawMap (letterbox + y-flip + footprint box) but into the panel canvas.
function renderMap(p) {
    const x = p.ctx, cw = p.cw, ch = p.ch;
    p.bg();
    p.header('Map');
    const top = 56, area = ch - top - 16;
    const m = vui.lastMap;
    if (!m || !m.img) {
        x.fillStyle = C.dim; x.font = '16px ui-monospace,monospace';
        x.fillText('awaiting map…', 28, top + 30);
        return;
    }
    const scale = Math.min(cw / m.w, area / m.h);
    const dw = m.w * scale, dh = m.h * scale;
    const dx = (cw - dw) / 2, dy = top + (area - dh) / 2;
    x.imageSmoothingEnabled = false;
    x.save();
    x.translate(dx, dy + dh); x.scale(1, -1);  // world-north = top
    x.drawImage(m.img, 0, 0, dw, dh);
    const o = vui.lastOdom;
    if (o && m.res > 0) {
        const col = (o.x - m.origin[0]) / m.res, rw = (o.y - m.origin[1]) / m.res;
        const px = col * scale, py = rw * scale;
        if (px >= 0 && px <= dw && py >= 0 && py <= dh) {
            const pxPerM = scale / m.res;
            let lenPx = GO2_LEN_M * pxPerM, widPx = GO2_WID_M * pxPerM;
            if (lenPx < 14) { widPx *= 14 / lenPx; lenPx = 14; }
            x.save(); x.translate(px, py); x.rotate(o.yaw || 0);
            x.fillStyle = 'rgba(176,225,240,0.35)'; x.strokeStyle = C.cyan; x.lineWidth = 1.5;
            x.fillRect(-lenPx / 2, -widPx / 2, lenPx, widPx);
            x.strokeRect(-lenPx / 2, -widPx / 2, lenPx, widPx);
            x.beginPath(); x.moveTo(0, 0); x.lineTo(lenPx / 2, 0);
            x.strokeStyle = C.panel; x.lineWidth = 2; x.stroke();
            x.restore();
        }
    }
    const g = vui.navGoal;
    if (g && m.res > 0) {
        const gx = ((g.x - m.origin[0]) / m.res) * scale, gy = ((g.y - m.origin[1]) / m.res) * scale;
        x.save(); x.translate(gx, gy);
        x.strokeStyle = C.cyan; x.lineWidth = 1.5;
        x.beginPath(); x.arc(0, 0, 7, 0, Math.PI * 2); x.stroke();
        x.fillStyle = C.cyan; x.beginPath(); x.arc(0, 0, 2, 0, Math.PI * 2); x.fill();
        x.restore();
    }
    x.restore();
    // Whole map area is a click target → nav goal (store geometry for inverse).
    p._mapGeom = { dx, dy, dw, dh, scale, m };
    p.regions.push({ id: 'map', x: dx, y: top, w: dw, h: area });
}

// ── click dispatch ───────────────────────────────────────────────────
function handleButtonsClick(id, panel) {
    if (id === 'estop') return estop(panel);
    if (id === 'rearm') return rearm(panel);
    if (id === 'obstacle') return toggleObstacle(panel, id);
    const [kind, val] = id.split(':');
    if (kind === 'sport') sendSportCmd(val, panel, id);
    else if (kind === 'speed') selectSpeed(val);
    else if (kind === 'cam') toggleCam(val, panel);
    else if (kind === 'light') setLight(parseFloat(val), panel, id);
    panel.markDirty();
}

function handleMapClick(panel, uv) {
    const G = panel._mapGeom;
    if (!G || !chanReady()) return;
    // panel-uv → canvas px → map cell → world metres (inverse of renderMap).
    const px = uv.x * panel.cw, py = (1 - uv.y) * panel.ch;
    const col = (px - G.dx) / G.scale;
    const row = ((G.dy + G.dh) - py) / G.scale;
    if (col < 0 || col > G.dw / G.scale || row < 0 || row > G.dh / G.scale) return;
    const wx = G.m.origin[0] + col * G.m.res;
    const wy = G.m.origin[1] + row * G.m.res;
    vui.navGoal = { x: wx, y: wy };
    sendJSON({ type: 'nav_goal', x: wx, y: wy, nonce: ++vui.nonce });
    panel.markDirty();
}

// ─────────────────────────────────────────────────────────────────────
// Public: build the panels, wire click, expose per-frame redraw
// ─────────────────────────────────────────────────────────────────────
let _allPanels = [];
let _mapPanel = null;
let _lastStatsMs = 0;  // throttle the stats-panel repaint to ~1Hz

export function buildCockpit(scene, headPos) {
    // Folded triptych: MAP + STATS hinge at the camera panel's left/right edges
    // and angle ~30° toward the operator; CONSOLE is a tilted shelf below.
    // CAM_HALF_W/PANEL_Y/PANEL_Z must agree with vr.js CAM.
    const CAM_HALF_W = 0.7, PANEL_Y = 1.52, PANEL_Z = -1.6;
    const FOLD = THREE.MathUtils.degToRad(30);

    const stats = new Panel({ wM: 0.44, hM: 0.7875, cw: 380, ch: 680, opacity: 0.96 });
    const map = new Panel({ wM: 0.9, hM: 0.7875, cw: 640, ch: 560, opacity: 0.97 });
    const console_ = new Panel({ wM: 1.5, hM: 0.52, cw: 1180, ch: 410, opacity: 0.97 });
    _mapPanel = map;

    // Map: its RIGHT edge hinges on the camera's LEFT edge, folds in (+yaw
    // turns its normal toward the user). Stats mirror on the right.
    map.placeHinged(new THREE.Vector3(-CAM_HALF_W, PANEL_Y, PANEL_Z), +1, +FOLD);
    stats.placeHinged(new THREE.Vector3(CAM_HALF_W, PANEL_Y, PANEL_Z), -1, -FOLD);
    // Console: a desk shelf below the camera — low enough that its top edge
    // stays under the sight line to the camera's bottom edge, tilted up ~34°.
    console_.placeFlat(new THREE.Vector3(0, 0.86, -1.3), -0.6);
    for (const p of [map, console_, stats]) { scene.add(p.mesh); p.mesh.renderOrder = 3; }

    _allPanels = [map, console_, stats];
    _lastStatsMs = 0;  // paint stats on the first frame of this session
    console_._render = renderConsole;
    stats._render = renderStats;
    map._render = renderMap;

    return {
        panels: _allPanels,
        meshes: _allPanels.map((p) => p.mesh),
        onClick(panel, uv) {
            if (panel === map) return handleMapClick(panel, uv);
            const id = panel.hitTest(uv);
            if (id) handleButtonsClick(id, panel);
        },
        // Stats data changes at 1Hz — refresh it at 1Hz, not per XR frame (a
        // per-frame canvas repaint + texture upload would risk Quest judder).
        // Buttons/map still redraw immediately via their own dirty flags.
        tick(nowMs) {
            if (nowMs - _lastStatsMs >= 1000) { stats.markDirty(); _lastStatsMs = nowMs; }
            for (const p of _allPanels) {
                if (!p.dirty) continue;
                p._render(p);
                p.tex.needsUpdate = true;
                p.dirty = false;
            }
        },
        dispose() {
            for (const p of _allPanels) { scene.remove(p.mesh); p.dispose(); }
            _allPanels = []; _mapPanel = null;
        },
    };
}

