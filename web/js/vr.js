// Immersive WebXR scene: video on a textured quad in front of the viewer,
// stats quad in the upper-right, controller poses + Joy streamed each frame.

import { geometry_msgs, sensor_msgs, std_msgs } from 'https://esm.sh/jsr/@dimos/msgs@0.1.4';
import { disconnect } from './disconnect.js';
import { drawStatsQuad, initStatsQuad } from './hud.js';
import { sendInterval, state } from './state.js';
import { send } from './webrtc.js';

function _compileShader(type, src) {
    const gl = state.gl;
    const s = gl.createShader(type);
    gl.shaderSource(s, src);
    gl.compileShader(s);
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) {
        throw new Error('shader: ' + gl.getShaderInfoLog(s));
    }
    return s;
}

function initVideoQuad() {
    const gl = state.gl;
    const vs = `
        attribute vec2 aPos; attribute vec2 aUV; varying vec2 vUV;
        uniform mat4 uMVP;
        void main() { vUV = aUV; gl_Position = uMVP * vec4(aPos, 0.0, 1.0); }`;
    const fs = `
        precision mediump float; varying vec2 vUV; uniform sampler2D uTex;
        void main() { gl_FragColor = texture2D(uTex, vUV); }`;
    state.quadProgram = gl.createProgram();
    gl.attachShader(state.quadProgram, _compileShader(gl.VERTEX_SHADER, vs));
    gl.attachShader(state.quadProgram, _compileShader(gl.FRAGMENT_SHADER, fs));
    gl.linkProgram(state.quadProgram);

    // Quad centered at origin, 1.2m wide × 0.675m tall (16:9), half-extents
    // below. Interleaved pos(x,y) + uv; V flipped so the video isn't upside-down.
    const w = 0.6, h = 0.3375;
    const verts = new Float32Array([
        -w, -h, 0, 1,   w, -h, 1, 1,   w, h, 1, 0,
        -w, -h, 0, 1,   w, h, 1, 0,   -w, h, 0, 0,
    ]);
    state.quadBuf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, state.quadBuf);
    gl.bufferData(gl.ARRAY_BUFFER, verts, gl.STATIC_DRAW);

    state.quadTex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, state.quadTex);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);

    state.quadUniforms = {
        mvp: gl.getUniformLocation(state.quadProgram, 'uMVP'),
        tex: gl.getUniformLocation(state.quadProgram, 'uTex'),
        aPos: gl.getAttribLocation(state.quadProgram, 'aPos'),
        aUV: gl.getAttribLocation(state.quadProgram, 'aUV'),
    };
}

function initGL() {
    const canvas = document.getElementById('canvas');
    state.gl = canvas.getContext('webgl', { xrCompatible: true, alpha: true });
    if (!state.gl) throw new Error('WebGL not supported');
    state.gl.clearColor(0, 0, 0, 0);
    initVideoQuad();
    initStatsQuad();
}

// column-major 4x4 multiply (a * b), arrays of length 16.
function mat4mul(a, b) {
    const o = new Float32Array(16);
    for (let c = 0; c < 4; c++)
        for (let r = 0; r < 4; r++)
            o[c * 4 + r] =
                a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] +
                a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
    return o;
}

// World-stationary in local-floor — the headset moves around the quad (like
// a TV on a wall), not the other way round. local-floor's origin is where
// the user started, so these absolute coords sit in front of them.
const PANEL_POS_X = 0.0;
const PANEL_POS_Y = 1.4;   // ~eye height (floor-relative)
const PANEL_POS_Z = -1.5;  // 1.5m forward
const videoQuadWorldModel = new Float32Array([
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    PANEL_POS_X, PANEL_POS_Y, PANEL_POS_Z, 1,
]);

function drawVideoQuad(frame, glLayer) {
    const gl = state.gl;
    const v = document.getElementById('robot-cam');
    if (!v || v.readyState < 2 || !v.videoWidth) return;  // no frame yet
    const pose = frame.getViewerPose(state.xrRefSpace);
    if (!pose) return;

    gl.disable(gl.DEPTH_TEST);  // HUD-style panel: always visible
    gl.bindTexture(gl.TEXTURE_2D, state.quadTex);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, v);

    gl.useProgram(state.quadProgram);
    gl.bindBuffer(gl.ARRAY_BUFFER, state.quadBuf);
    gl.enableVertexAttribArray(state.quadUniforms.aPos);
    gl.vertexAttribPointer(state.quadUniforms.aPos, 2, gl.FLOAT, false, 16, 0);
    gl.enableVertexAttribArray(state.quadUniforms.aUV);
    gl.vertexAttribPointer(state.quadUniforms.aUV, 2, gl.FLOAT, false, 16, 8);
    gl.uniform1i(state.quadUniforms.tex, 0);

    for (const view of pose.views) {
        const vp = glLayer.getViewport(view);
        gl.viewport(vp.x, vp.y, vp.width, vp.height);
        // World-stationary: projection * world→eye * worldModel.
        const viewProj = mat4mul(view.projectionMatrix, view.transform.inverse.matrix);
        const mvp = mat4mul(viewProj, videoQuadWorldModel);
        gl.uniformMatrix4fv(state.quadUniforms.mvp, false, mvp);
        gl.drawArrays(gl.TRIANGLES, 0, 6);
    }
}

function processTracking(frame) {
    const now = performance.now();
    if (now - state.lastSendTime < sendInterval) return;
    state.lastSendTime = now;

    // Modality-agnostic: we just stream poses + Joy. The robot blueprint
    // decides what to do with them (arm IK or thumbstick → base velocity).
    for (const inputSource of frame.session.inputSources) {
        const trackingSpace = inputSource.gripSpace || inputSource.targetRaySpace;
        if (!trackingSpace) continue;
        const handedness = inputSource.handedness;
        if (handedness !== 'left' && handedness !== 'right') continue;

        const pose = frame.getPose(trackingSpace, state.xrRefSpace);
        if (!pose) continue;

        const pos = pose.transform.position;
        const rot = pose.transform.orientation;
        const nowMs = Date.now() + state.clockOffsetMs;
        const stamp = new std_msgs.Time({
            sec: Math.floor(nowMs / 1000),
            nsec: (nowMs % 1000) * 1_000_000,
        });

        const poseStamped = new geometry_msgs.PoseStamped({
            header: new std_msgs.Header({ stamp, frame_id: handedness }),
            pose: new geometry_msgs.Pose({
                position: new geometry_msgs.Point({ x: pos.x, y: pos.y, z: pos.z }),
                orientation: new geometry_msgs.Quaternion({ x: rot.x, y: rot.y, z: rot.z, w: rot.w }),
            }),
        });
        send(poseStamped.encode());

        const gamepad = inputSource.gamepad;
        if (gamepad) {
            // Quest Touch thumbstick lives at axes[2]/[3]; [0]/[1] is the
            // dead legacy touchpad. Packed into Joy axes[0]/[1] for the robot.
            const stickX = gamepad.axes[2] ?? gamepad.axes[0] ?? 0.0;
            const stickY = gamepad.axes[3] ?? gamepad.axes[1] ?? 0.0;
            const axes = [
                stickX,
                stickY,
                gamepad.buttons[0]?.value ?? 0.0,
                gamepad.buttons[1]?.value ?? 0.0,
            ];
            const buttons = [];
            for (let i = 0; i < gamepad.buttons.length; i++) {
                buttons.push(gamepad.buttons[i]?.pressed ? 1 : 0);
            }
            const joyMsg = new sensor_msgs.Joy({
                header: new std_msgs.Header({ stamp, frame_id: handedness }),
                axes_length: axes.length,
                buttons_length: buttons.length,
                axes,
                buttons,
            });
            send(joyMsg.encode());
        }
    }
}

function onXRFrame(time, frame) {
    if (!state.xrSession) return;
    state.xrSession.requestAnimationFrame(onXRFrame);
    processTracking(frame);

    const gl = state.gl;
    const glLayer = state.xrSession.renderState.baseLayer;
    gl.bindFramebuffer(gl.FRAMEBUFFER, glLayer.framebuffer);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    drawVideoQuad(frame, glLayer);
    sampleCmdHz(time);                                              // VR has no DOM hudTimer
    drawStatsQuad(frame, glLayer, mat4mul, videoQuadWorldModel);    // anchored to the video quad
}

// Roll cmdSendCount → liveStats.cmdHz once per second. The browser HUD's
// hudTimer does this for the DOM view; VR drives it off the XR frame loop.
let lastCmdSampleMs = 0;
function sampleCmdHz(nowMs) {
    if (!lastCmdSampleMs) { lastCmdSampleMs = nowMs; return; }
    const dt = (nowMs - lastCmdSampleMs) / 1000;
    if (dt < 1.0) return;
    state.liveStats.cmdHz = state.cmdSendCount / dt;
    state.cmdSendCount = 0;
    lastCmdSampleMs = nowMs;
}

export async function startVR() {
    const canvas = document.getElementById('canvas');
    canvas.style.display = 'block';
    initGL();

    let session = null;
    try {
        session = await navigator.xr.requestSession('immersive-ar', {
            requiredFeatures: ['local-floor'],
            optionalFeatures: ['hand-tracking'],
        });
    } catch (e) {
        session = await navigator.xr.requestSession('immersive-vr', {
            requiredFeatures: ['local-floor'],
            optionalFeatures: ['hand-tracking'],
        });
    }

    state.xrSession = session;
    // Real Quest needs an explicit makeXRCompatible — the xrCompatible flag
    // at context creation isn't enough (the emulator is lenient about this).
    await state.gl.makeXRCompatible();
    const glLayer = new XRWebGLLayer(session, state.gl);
    await session.updateRenderState({ baseLayer: glLayer });
    state.xrRefSpace = await session.requestReferenceSpace('local-floor');

    session.addEventListener('end', () => {
        state.xrSession = null;
        disconnect();
    });
    session.requestAnimationFrame(onXRFrame);
}
