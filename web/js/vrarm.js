// Immersive WebXR cockpit for the xArm (manipulation). Parallel to vr.js (the
// Go2 drive cockpit): same generic shell — XR session, renderer, camera video
// panel, controller rays, hover, stats — but the input plane streams 6-DoF
// controller poses + gripper instead of thumbstick drive twists.
//
//   controller gripSpace pose ──cmd_unreliable──▶ PoseStamped (frame_id=hand)
//   trigger/buttons           ──cmd_unreliable──▶ Joy  (axes[2]=gripper, btn4=engage)
//
// Engage = hold the primary button (X left / A right); the robot recaptures the
// baseline pose on engage and streams deltas, so we send ABSOLUTE poses. The
// robot's webxr_to_robot owns the frame conversion — we send raw WebXR poses.

import * as THREE from 'three';

import { sampleCmdHz } from './hud.js';
import { createStallGate, videoMediaTime } from './stall.js';
import { disconnect } from './disconnect.js';
import { sendInterval, state } from './state.js';
import { buildArmCockpit, aui, onCmdAck, onRobotState } from './vrarmui.js';
import { buildJoy, buildPoseStamped, sendEstop } from './xarmcmd.js';

const HEAD = new THREE.Vector3(0, 1.55, 0);
// Camera panel — front centre, 16:9. Matches vrarmui console placement.
const CAM = { w: 1.4, h: 0.7875, x: 0, y: 1.52, z: -1.6 };

let renderer = null, scene = null, camera = null;
let cockpit = null, controllers = [];
let videoMesh = null, videoTex = null;
let stallGate = null;
let camEl = null;
let estopNonce = 0;
let _estopCooldown = 0;
const raycaster = new THREE.Raycaster();
const _rayOrigin = new THREE.Vector3();
const _rayDir = new THREE.Vector3();

function buildScene() {
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(70, 1, 0.05, 100);

    videoMesh = new THREE.Mesh(
        new THREE.PlaneGeometry(CAM.w, CAM.h),
        new THREE.MeshBasicMaterial({ color: 0x0d0e0e }),
    );
    videoMesh.position.set(CAM.x, CAM.y, CAM.z);
    videoMesh.renderOrder = 1;
    scene.add(videoMesh);

    cockpit = buildArmCockpit(scene, HEAD);
}

function initControllers() {
    const lineGeo = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -5),
    ]);
    for (let i = 0; i < 2; i++) {
        const ctrl = renderer.xr.getController(i);
        const laser = new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: 0xb0e1f0, transparent: true, opacity: 0.5 }));
        laser.scale.z = 1;
        ctrl.add(laser);
        const dot = new THREE.Mesh(
            new THREE.SphereGeometry(0.012, 12, 12),
            new THREE.MeshBasicMaterial({ color: 0xb0e1f0 }),
        );
        dot.visible = false;
        ctrl.userData.dot = dot;
        scene.add(dot);
        ctrl.addEventListener('selectstart', () => onSelect(ctrl));
        scene.add(ctrl);
        controllers.push(ctrl);
    }
}

function onSelect(ctrl) {
    const hit = raycastPanels(ctrl);
    if (hit) cockpit.onClick(hit.object.userData.panel, hit.uv);
}

function raycastPanels(ctrl) {
    _rayOrigin.setFromMatrixPosition(ctrl.matrixWorld);
    _rayDir.set(0, 0, -1).applyQuaternion(ctrl.quaternion).normalize();
    raycaster.set(_rayOrigin, _rayDir);
    const hits = raycaster.intersectObjects(cockpit.meshes, false);
    return hits.length ? hits[0] : null;
}

function updateVideoTexture() {
    const v = camEl;
    if (!v || v.readyState < 2 || !v.videoWidth) return;
    if (!videoTex || videoTex.image !== v) {
        videoTex?.dispose();
        videoTex = new THREE.VideoTexture(v);
        videoTex.colorSpace = THREE.SRGBColorSpace;
        videoMesh.material.dispose();
        videoMesh.material = new THREE.MeshBasicMaterial({ map: videoTex });
    }
    const strip = state.liveStats.stampStripPx || 0;
    const frac = strip && v.videoHeight ? strip / v.videoHeight : 0;
    videoTex.offset.y = frac;
    videoTex.repeat.y = 1 - frac;
}

// ── Arm command plane: stream controller pose + Joy per hand ─────────
let lastSend = 0;

function streamArmPose(frame) {
    const now = performance.now();
    if (now - lastSend < sendInterval) return;
    lastSend = now;

    const chan = state.cmdChannel;
    if (!chan || chan.readyState !== 'open') return;

    // Video-freshness gate: don't stream poses onto a frozen frame — the
    // operator would be commanding blind. (No held-state to track: any pose is
    // only acted on while a hand is engaged, and engage needs a live view.)
    const gate = stallGate.sample(videoMediaTime(camEl), now, false);
    state.videoStall = gate;
    if (gate.blocked) return;

    const nowMs = Date.now() + state.clockOffsetMs;
    for (const src of frame.session.inputSources) {
        const space = src.gripSpace || src.targetRaySpace;
        if (!space) continue;
        const hand = src.handedness;
        if (hand !== 'left' && hand !== 'right') continue;
        const pose = frame.getPose(space, state.xrRefSpace);
        if (!pose) continue;

        chan.send(buildPoseStamped(hand, pose.transform.position, pose.transform.orientation, nowMs).encode());
        state.cmdSendCount++;

        if (src.gamepad) {
            chan.send(buildJoy(hand, src.gamepad, nowMs).encode());
            // Menu button (index 6) → E-STOP, debounced.
            if (src.gamepad.buttons[6]?.pressed) triggerEstop();
        }
    }
}

function triggerEstop() {
    const now = performance.now();
    if (now < _estopCooldown || aui.estopped) return;
    _estopCooldown = now + 1500;
    aui.estopped = true;
    sendEstop(state.stateChannel, () => ++estopNonce);
    cockpit.panels.forEach((p) => p.markDirty());
}

let lastCmdSampleMs = 0;
function tickCmdHz(nowMs) {
    if (!lastCmdSampleMs) { lastCmdSampleMs = nowMs; return; }
    if (nowMs - lastCmdSampleMs < 1000) return;
    sampleCmdHz((nowMs - lastCmdSampleMs) / 1000);
    lastCmdSampleMs = nowMs;
}

function updateHover() {
    for (const p of cockpit.panels) p.setHover(null);
    for (const ctrl of controllers) {
        const dot = ctrl.userData.dot;
        const hit = raycastPanels(ctrl);
        if (!hit) { dot.visible = false; continue; }
        dot.visible = true;
        dot.position.copy(hit.point);
        const panel = hit.object.userData.panel;
        panel.setHover(panel.hitTest(hit.uv));
    }
}

function onFrame(timeMs, frame) {
    if (!camEl) camEl = document.getElementById('robot-cam');
    if (frame) streamArmPose(frame);
    updateHover();
    updateVideoTexture();
    if (videoMesh.material.map) {
        const stalled = state.videoStall?.stalled;
        videoMesh.material.color.setHex(stalled ? 0x552222 : 0xffffff);
    }
    cockpit.tick(timeMs);
    tickCmdHz(timeMs);
    renderer.render(scene, camera);
}

export async function startArmVR() {
    lastCmdSampleMs = 0; lastSend = 0;
    stallGate = createStallGate();
    state.videoStall = { stalled: false, blocked: false, armed: false };
    const canvas = document.getElementById('canvas');
    canvas.style.display = 'block';

    if (!renderer) {
        // xrCompatible at context creation — on Quest, without it the GL context
        // is unusable by the immersive session and video panels come up black.
        renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, xrCompatible: true });
        renderer.autoClear = true;
        renderer.xr.enabled = true;
        renderer.xr.setReferenceSpaceType('local-floor');
    }
    buildScene();
    controllers = [];

    // Passthrough (AR) when available; opaque VR otherwise. Must run inside the
    // Connect click gesture.
    let session = null, ar = false;
    try {
        session = await navigator.xr.requestSession('immersive-ar', {
            requiredFeatures: ['local-floor'], optionalFeatures: ['hand-tracking'],
        });
        ar = true;
    } catch (e) {
        session = await navigator.xr.requestSession('immersive-vr', {
            requiredFeatures: ['local-floor'], optionalFeatures: ['hand-tracking'],
        });
    }
    scene.background = ar ? null : new THREE.Color(0x0a0b0b);
    renderer.setClearAlpha(ar ? 0 : 1);

    state.xrSession = session;
    await renderer.xr.setSession(session);
    state.xrRefSpace = renderer.xr.getReferenceSpace();
    initControllers();

    // Route acks + robot-state onto the arm cockpit.
    state.onCmdAck = onCmdAck;
    state.onRobotState = onRobotState;

    session.addEventListener('end', () => {
        state.xrSession = null;
        state.onCmdAck = state.onRobotState = null;
        renderer.setAnimationLoop(null);
        cockpit?.dispose();
        cockpit = null;
        videoTex?.dispose(); videoTex = null;
        videoMesh?.geometry.dispose(); videoMesh?.material.dispose(); videoMesh = null;
        camEl = null;
        disconnect();
    });
    renderer.setAnimationLoop(onFrame);
}
