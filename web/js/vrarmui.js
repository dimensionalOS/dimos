// VR cockpit UI for the xArm — minimal first cut: ONE console panel below the
// camera showing engage state (per hand), E-STOP / clear, camera select, and a
// stats line. Parallel to vrui.js (the Go2 cockpit) but deliberately small.
//
// It reuses vrui.js's *contract* (buildCockpit → { panels, meshes, onClick,
// tick, dispose }) that vrarm.js consumes, and a self-contained Panel (the
// generic canvas→plane→hit-region bits copied from vrui.js) so we don't have to
// refactor/export vrui.js internals yet. Extract a shared Panel later, once both
// cockpits are stable.

import * as THREE from 'three';

import { transportLabel } from './hud.js';
import { state } from './state.js';
import { sendCameraSelect, sendEstop, sendEstopClear } from './xarmcmd.js';

const C = {
    bg: 'rgba(18,19,19,0.92)', bgSolid: '#121313', panel: '#0d0e0e',
    line: '#2a2a2a', text: '#d1d5db', dim: '#6b7280', cyan: '#b0e1f0',
    good: '#34d399', warn: '#ffcc00', bad: '#ff5252',
    estopBorder: '#d97777',
};

const CAMS = [{ id: 'cam1', label: 'Cam 1' }, { id: 'cam2', label: 'Cam 2' }];

// Cockpit UI state — reconciled from robot_telemetry (authoritative on connect).
export const aui = {
    engaged: { left: false, right: false },
    estopped: false,
    selectedCams: ['cam1'],
    nonce: 0,
    pending: new Map(),   // nonce → { id, expiry } for cmd_ack feedback
};

function nextNonce() { return ++aui.nonce; }
function chanReady() { return state.stateChannel && state.stateChannel.readyState === 'open'; }

// ── Panel: canvas → CanvasTexture → plane, with hit regions (from vrui.js) ──
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
        this.regions = [];
        this.hoverId = null;
        this.dirty = true;
    }
    // Desk shelf below the camera: fixed tilt about X, no lookAt.
    placeFlat(pos, rotX) { this.mesh.position.copy(pos); this.mesh.rotation.set(rotX, 0, 0); }
    markDirty() { this.dirty = true; }
    // UV (three, flipY) → canvas px; row = (1-v)*ch.
    hitTest(uv) {
        const px = uv.x * this.cw, py = (1 - uv.y) * this.ch;
        for (const r of this.regions) {
            if (px >= r.x && px <= r.x + r.w && py >= r.y && py <= r.y + r.h) return r.id;
        }
        return null;
    }
    setHover(id) { if (id !== this.hoverId) { this.hoverId = id; this.dirty = true; } }
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
    // A chip button; st ∈ idle|active|pending|error.
    chip(id, bx, by, bw, bh, label, st = 'idle') {
        const x = this.ctx, hot = this.hoverId === id;
        let fill = C.bgSolid, border = C.line, txt = C.text;
        if (st === 'active') { fill = C.cyan; border = C.cyan; txt = C.panel; }
        else if (st === 'error') { fill = '#4a1d1d'; border = C.estopBorder; txt = '#f3b4b4'; }
        else if (st === 'pending') { border = C.cyan; txt = C.dim; }
        if (hot && st === 'idle') border = C.cyan;
        x.fillStyle = fill; roundRect(x, bx, by, bw, bh, 10); x.fill();
        x.lineWidth = hot ? 2.5 : 1.5; x.strokeStyle = border; x.stroke();
        x.fillStyle = txt;
        x.font = '600 22px ui-monospace, monospace';
        x.textAlign = 'center'; x.textBaseline = 'middle';
        x.fillText(label, bx + bw / 2, by + bh / 2 + 1);
        x.textAlign = 'left'; x.textBaseline = 'alphabetic';
        this.regions.push({ id, x: bx, y: by, w: bw, h: bh });
    }
    // A read-only status pill (engaged L/R) — no hit region.
    pill(bx, by, bw, bh, label, on) {
        const x = this.ctx;
        x.fillStyle = on ? C.cyan : C.bgSolid;
        roundRect(x, bx, by, bw, bh, 10); x.fill();
        x.lineWidth = 1.5; x.strokeStyle = on ? C.cyan : C.line; x.stroke();
        x.fillStyle = on ? C.panel : C.dim;
        x.font = '600 22px ui-monospace, monospace';
        x.textAlign = 'center'; x.textBaseline = 'middle';
        x.fillText(label, bx + bw / 2, by + bh / 2 + 1);
        x.textAlign = 'left'; x.textBaseline = 'alphabetic';
    }
    dispose() { this.tex.dispose(); this.mesh.geometry.dispose(); this.mesh.material.dispose(); }
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

// ── Console render ───────────────────────────────────────────────────
function renderConsole(p) {
    p.bg();
    p.header('xArm cockpit');

    // Engage status (read-only; the robot decides engage from the held button).
    const x = p.ctx;
    x.fillStyle = C.dim; x.font = '600 20px ui-monospace, monospace';
    x.fillText('ENGAGE', 24, 96);
    p.pill(150, 74, 130, 44, `L ${aui.engaged.left ? 'ON' : '—'}`, aui.engaged.left);
    p.pill(292, 74, 130, 44, `R ${aui.engaged.right ? 'ON' : '—'}`, aui.engaged.right);

    // Camera select.
    x.fillStyle = C.dim; x.fillText('CAMERA', 24, 176);
    CAMS.forEach((c, i) => {
        const on = aui.selectedCams.includes(c.id);
        p.chip(`cam:${c.id}`, 150 + i * 142, 154, 130, 44, c.label, on ? 'active' : 'idle');
    });

    // E-STOP / clear — always reachable, big.
    if (aui.estopped) {
        p.chip('estop_clear', 24, 238, 540, 120, 'E-STOP LATCHED — CLEAR', 'error');
    } else {
        const pend = [...aui.pending.values()].some((v) => v.id === 'estop');
        p.chip('estop', 24, 238, 540, 120, 'E-STOP', pend ? 'pending' : 'idle');
    }

    // Stats line (transport + robot-measured cmd latency).
    x.fillStyle = C.dim; x.font = '500 18px ui-monospace, monospace';
    const cmd = state.liveStats.cmd;
    const lat = cmd && cmd.latency_ms != null ? `${Math.round(cmd.latency_ms)}ms` : '—';
    const hz = cmd && cmd.rate_hz != null ? `${Math.round(cmd.rate_hz)}Hz` : '—';
    const tl = transportLabel ? transportLabel() : (state.activeRobot?.transport || '');
    x.fillText(`cmd ${lat} · ${hz} · ${tl}`, 600, 96);
}

function handleClick(id) {
    if (id === 'estop') {
        aui.estopped = true;  // optimistic; robot_telemetry reconciles
        markPending('estop');
        sendEstop(state.stateChannel, nextNonce);
    } else if (id === 'estop_clear') {
        aui.estopped = false;
        markPending('estop_clear');
        sendEstopClear(state.stateChannel, nextNonce);
    } else if (id.startsWith('cam:')) {
        const cam = id.slice(4);
        // Toggle within the known set; never allow an empty selection.
        let sel = aui.selectedCams.includes(cam)
            ? aui.selectedCams.filter((c) => c !== cam)
            : [...aui.selectedCams, cam];
        if (sel.length === 0) sel = [cam];
        aui.selectedCams = CAMS.map((c) => c.id).filter((c) => sel.includes(c));
        sendCameraSelect(state.stateChannel, aui.selectedCams);
    }
}

function markPending(id) {
    aui.pending.set(aui.nonce + 1, { id, expiry: 0 });  // nonce bumps on send
}

// cmd_ack handler (wired via state.onCmdAck) — clear the pending chip.
export function onCmdAck(msg) {
    aui.pending.delete(msg.nonce);
    _dirty();
}

// robot_telemetry.state handler (wired via state.onRobotState) — authoritative.
export function onRobotState(s) {
    if (s.engaged) aui.engaged = { left: !!s.engaged.left, right: !!s.engaged.right };
    if (typeof s.estopped === 'boolean') aui.estopped = s.estopped;
    if (Array.isArray(s.cams) && s.cams.length) aui.selectedCams = s.cams;
    _dirty();
}

let _panel = null;
function _dirty() { if (_panel) _panel.markDirty(); }

export function buildArmCockpit(scene, _headPos) {
    // One console shelf below the camera panel (CAM in vrarm.js sits at y≈1.52,
    // z≈-1.6). Tilt up ~34° toward the operator, like the Go2 console.
    const console_ = new Panel({ wM: 1.5, hM: 0.52, cw: 1180, ch: 410, opacity: 0.97 });
    console_.placeFlat(new THREE.Vector3(0, 0.86, -1.3), -0.6);
    console_.mesh.renderOrder = 3;
    scene.add(console_.mesh);
    _panel = console_;
    let lastStatsMs = 0;  // repaint stats (cmd latency) at 1Hz, not per frame

    return {
        panels: [console_],
        meshes: [console_.mesh],
        onClick(panel, uv) {
            const id = panel.hitTest(uv);
            if (id) handleClick(id);
        },
        tick(nowMs) {
            if (nowMs - lastStatsMs >= 1000) { console_.markDirty(); lastStatsMs = nowMs; }
            if (!console_.dirty) return;
            renderConsole(console_);
            console_.tex.needsUpdate = true;
            console_.dirty = false;
        },
        dispose() {
            scene.remove(console_.mesh);
            console_.dispose();
            _panel = null;
        },
    };
}
