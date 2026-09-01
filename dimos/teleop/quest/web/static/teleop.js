// Global error handler
window.onerror = (msg, url, line, col, error) => {
    console.error(`[ERROR] ${msg} at ${url}:${line}:${col}`, error);
    document.getElementById('status').textContent = `Error: ${msg}`;
};

import { geometry_msgs, std_msgs, sensor_msgs } from "https://esm.sh/jsr/@dimos/msgs@0.1.4";

// WebSocket and VR state
let ws = null;
let xrSession = null;
let xrRefSpace = null;
let gl = null;
let lastSendTime = 0;
const sendInterval = 1000 / 80; // ~80Hz target
const handSelectActive = new Map();
const GRIPPER_PINCH_DISTANCE_METERS = 0.04;

// Video panel state
const videoEl = document.getElementById('videoFeed');
let videoTex = null;
let quadProgram = null;
let quadVbo = null;
let quadAttribs = null;
let quadUniforms = null;
let videoReady = false;  // true after first frame loads
let videoDirty = false;  // true when a new JPEG has finished decoding
let videoAspect = 1.0;   // cached at load time — see videoEl.onload
let prevBlobUrl = null;  // revoked when the next-next blob arrives
const videoModelMatrix = new Float32Array(16);
const hudModelMatrix = new Float32Array(16);
const hudLocalMatrix = new Float32Array(16);

const PANEL_POS_X = 0.0;
const PANEL_POS_Y = 1.4;   // ~eye height
const PANEL_POS_Z = -1.5;  // 1.5m in front of starting position
const PANEL_HEIGHT = 0.9;

// Collection HUD state. The panel is anchored below the initial headset pose
// and remains fixed in the local-floor scene.
let episodeStatus = null;
let episodeStatusReceivedAtMs = 0;
let hudOffline = false;
let hudCanvas = null;
let hudContext = null;
let hudTexture = null;
let hudDirty = false;
let hudElapsedSecond = -1;
let hudPlaced = false;

const HUD_WIDTH_PX = 2048;
const HUD_HEIGHT_PX = 256;
const HUD_WIDTH_METERS = 1.08;
const HUD_HEIGHT_METERS = HUD_WIDTH_METERS * HUD_HEIGHT_PX / HUD_WIDTH_PX;
const HUD_OFFSET_Y = -0.6;
const HUD_OFFSET_Z = -1.2;

// UI elements
const statusEl = document.getElementById('status');
const connectBtn = document.getElementById('connectBtn');
const disconnectBtn = document.getElementById('disconnectBtn');
const canvas = document.getElementById('canvas');

function setStatus(msg) {
    statusEl.textContent = msg;
}

// WebSocket setup (LCM bridge)
function setupWebSocket() {
    return new Promise((resolve, reject) => {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws`;

        setStatus('Connecting to server...');
        ws = new WebSocket(wsUrl);
        ws.binaryType = 'blob';

        ws.onopen = () => {
            hudOffline = false;
            hudDirty = true;
            setStatus('Server connected');
            resolve();
        };
        ws.onerror = (error) => {
            setStatus('WebSocket error');
            console.error('WebSocket error:', error);
            reject(error);
        };
        ws.onclose = () => {
            hudOffline = true;
            hudDirty = true;
            setStatus('WebSocket closed');
        };
        // Defer revoking the previous blob URL by one message — revoking
        // immediately after setting src can race with the browser's load
        // on some engines, briefly dropping naturalWidth to 0.
        ws.onmessage = (e) => {
            if (typeof e.data === 'string') {
                handleServerMessage(e.data);
                return;
            }
            if (!(e.data instanceof Blob)) return;
            const newUrl = URL.createObjectURL(e.data);
            if (prevBlobUrl) URL.revokeObjectURL(prevBlobUrl);
            prevBlobUrl = videoEl.src.startsWith('blob:') ? videoEl.src : null;
            videoEl.src = newUrl;
        };
    });
}

// Initialize WebGL
function initGL() {
    gl = canvas.getContext('webgl', {
        xrCompatible: true,
        alpha: true
    });
    if (!gl) {
        throw new Error('WebGL not supported');
    }
    gl.clearColor(0, 0, 0, 0); // Transparent background for passthrough
    initVideoPanel();
    initCollectionHud();
}

// Compile one textured-quad pipeline for the world-locked video and HUD.
function initVideoPanel() {
    const vsSrc = `
        attribute vec2 a_pos;
        attribute vec2 a_uv;
        uniform mat4 u_proj;
        uniform mat4 u_view;
        uniform mat4 u_model;
        varying vec2 v_uv;
        void main() {
            gl_Position = u_proj * u_view * u_model
                        * vec4(a_pos.x, a_pos.y, 0.0, 1.0);
            v_uv = a_uv;
        }`;
    const fsSrc = `
        precision mediump float;
        varying vec2 v_uv;
        uniform sampler2D u_tex;
        void main() {
            gl_FragColor = texture2D(u_tex, v_uv);
        }`;

    const compile = (type, src) => {
        const sh = gl.createShader(type);
        gl.shaderSource(sh, src);
        gl.compileShader(sh);
        if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) {
            throw new Error('Shader compile failed: ' + gl.getShaderInfoLog(sh));
        }
        return sh;
    };
    const vs = compile(gl.VERTEX_SHADER, vsSrc);
    const fs = compile(gl.FRAGMENT_SHADER, fsSrc);
    quadProgram = gl.createProgram();
    gl.attachShader(quadProgram, vs);
    gl.attachShader(quadProgram, fs);
    gl.linkProgram(quadProgram);
    if (!gl.getProgramParameter(quadProgram, gl.LINK_STATUS)) {
        throw new Error('Program link failed: ' + gl.getProgramInfoLog(quadProgram));
    }

    // Quad as TRIANGLE_STRIP: x, y, u, v. v=0 at top of quad +
    // UNPACK_FLIP_Y_WEBGL=false → image displays upright.
    const verts = new Float32Array([
        -1, -1, 0, 1,
         1, -1, 1, 1,
        -1,  1, 0, 0,
         1,  1, 1, 0,
    ]);
    quadVbo = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, quadVbo);
    gl.bufferData(gl.ARRAY_BUFFER, verts, gl.STATIC_DRAW);

    quadAttribs = {
        pos: gl.getAttribLocation(quadProgram, 'a_pos'),
        uv:  gl.getAttribLocation(quadProgram, 'a_uv'),
    };
    quadUniforms = {
        proj:  gl.getUniformLocation(quadProgram, 'u_proj'),
        view:  gl.getUniformLocation(quadProgram, 'u_view'),
        model: gl.getUniformLocation(quadProgram, 'u_model'),
        tex:   gl.getUniformLocation(quadProgram, 'u_tex'),
    };

    videoTex = createTexture();
    gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);

    // Cache aspect here — naturalWidth can transiently drop to 0
    // between loads, which would collapse the panel to 1:1.
    videoEl.onload = () => {
        videoReady = true;
        videoDirty = true;
        if (videoEl.naturalHeight) {
            videoAspect = videoEl.naturalWidth / videoEl.naturalHeight;
        }
    };
}

function createTexture() {
    const texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    return texture;
}

function uploadVideoTexture() {
    gl.bindTexture(gl.TEXTURE_2D, videoTex);
    gl.texImage2D(
        gl.TEXTURE_2D, 0, gl.RGB, gl.RGB, gl.UNSIGNED_BYTE, videoEl
    );
}

function setModelMatrix(matrix, halfWidth, halfHeight, x, y, z) {
    matrix.fill(0);
    matrix[0] = halfWidth;
    matrix[5] = halfHeight;
    matrix[10] = 1;
    matrix[12] = x;
    matrix[13] = y;
    matrix[14] = z;
    matrix[15] = 1;
}

function multiplyMatrices(out, left, right) {
    for (let column = 0; column < 4; column++) {
        for (let row = 0; row < 4; row++) {
            let value = 0;
            for (let index = 0; index < 4; index++) {
                value += left[index * 4 + row] * right[column * 4 + index];
            }
            out[column * 4 + row] = value;
        }
    }
}

function updateVideoModelMatrix() {
    const halfH = PANEL_HEIGHT * 0.5;
    setModelMatrix(
        videoModelMatrix,
        halfH * videoAspect,
        halfH,
        PANEL_POS_X,
        PANEL_POS_Y,
        PANEL_POS_Z,
    );
}

function renderTexturedQuad(view, viewport, texture, modelMatrix, viewMatrix) {
    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
    gl.useProgram(quadProgram);

    gl.bindBuffer(gl.ARRAY_BUFFER, quadVbo);
    gl.enableVertexAttribArray(quadAttribs.pos);
    gl.vertexAttribPointer(quadAttribs.pos, 2, gl.FLOAT, false, 16, 0);
    gl.enableVertexAttribArray(quadAttribs.uv);
    gl.vertexAttribPointer(quadAttribs.uv,  2, gl.FLOAT, false, 16, 8);

    gl.uniformMatrix4fv(quadUniforms.proj, false, view.projectionMatrix);
    gl.uniformMatrix4fv(quadUniforms.view, false, viewMatrix);
    gl.uniformMatrix4fv(quadUniforms.model, false, modelMatrix);

    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.uniform1i(quadUniforms.tex, 0);

    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
}

function renderVideoPanel(view, viewport) {
    renderTexturedQuad(
        view,
        viewport,
        videoTex,
        videoModelMatrix,
        view.transform.inverse.matrix,
    );
}

function handleServerMessage(data) {
    try {
        const message = JSON.parse(data);
        if (message.type !== 'episode_status') return;
        episodeStatus = message;
        episodeStatusReceivedAtMs = performance.now();
        hudOffline = false;
        hudDirty = true;
    } catch (error) {
        console.warn('Ignoring invalid server message', error);
    }
}

function initCollectionHud() {
    hudCanvas = document.createElement('canvas');
    hudCanvas.width = HUD_WIDTH_PX;
    hudCanvas.height = HUD_HEIGHT_PX;
    hudContext = hudCanvas.getContext('2d');

    hudTexture = createTexture();
    setModelMatrix(
        hudLocalMatrix,
        HUD_WIDTH_METERS * 0.5,
        HUD_HEIGHT_METERS * 0.5,
        0,
        HUD_OFFSET_Y,
        HUD_OFFSET_Z,
    );
}

function placeCollectionHud(pose) {
    multiplyMatrices(hudModelMatrix, pose.transform.matrix, hudLocalMatrix);
    hudPlaced = true;
}

function drawHudSection(
    context,
    x,
    width,
    label,
    value,
    color,
    dotColor = null,
) {
    if (x > 24) {
        context.fillStyle = 'rgba(178, 196, 219, 0.22)';
        context.fillRect(x, 36, 2, HUD_HEIGHT_PX - 72);
    }

    const left = x + 28;
    context.fillStyle = '#c0cfdf';
    context.font = '700 27px sans-serif';
    context.fillText(label, left, 78);

    let valueLeft = left;
    if (dotColor) {
        context.fillStyle = dotColor;
        context.beginPath();
        context.arc(left + 10, 159, 10, 0, Math.PI * 2);
        context.fill();
        valueLeft += 34;
    }
    context.fillStyle = color;
    const font = 'ui-monospace, SFMono-Regular, Menlo, monospace';
    context.font = `600 43px ${font}`;
    const availableWidth = x + width - valueLeft - 24;
    const measuredWidth = context.measureText(value).width;
    const fontSize = Math.min(43, 43 * availableWidth / measuredWidth);
    context.font = `600 ${fontSize}px ${font}`;
    context.fillText(value, valueLeft, 174);
}

function collectionElapsedSeconds() {
    if (!episodeStatus || episodeStatus.state !== 'recording') return 0;
    const sinceUpdate = (performance.now() - episodeStatusReceivedAtMs) / 1000;
    return Math.max(0, Math.floor(episodeStatus.elapsed_s + sinceUpdate));
}

function formatElapsed(seconds) {
    const minutes = Math.floor(seconds / 60).toString().padStart(2, '0');
    const remainder = (seconds % 60).toString().padStart(2, '0');
    return `${minutes}:${remainder}`;
}

function updateHudTexture() {
    if (!episodeStatus || !hudContext) return;
    const elapsed = collectionElapsedSeconds();
    if (!hudDirty && elapsed === hudElapsedSecond) return;
    hudDirty = false;
    hudElapsedSecond = elapsed;

    const saved = episodeStatus.episodes_saved;
    const discarded = episodeStatus.episodes_discarded;
    const recording = episodeStatus.state === 'recording';
    const state = hudOffline ? 'OFFLINE' : recording ? 'RECORDING' : 'READY';
    const stateColor = hudOffline ? '#f6b94d' : recording ? '#ff6b6b' : '#63d6a3';
    const lastEvent = episodeStatus.last_event === 'init' ? 'NONE' : episodeStatus.last_event.toUpperCase();
    const sections = [
        [400, 'STATE', state, '#f4f7fb', stateColor],
        [220, 'TAKE', String(saved + discarded + 1).padStart(3, '0'), '#f4f7fb'],
        [280, 'ELAPSED', hudOffline ? '--:--' : formatElapsed(elapsed), '#f4f7fb'],
        [260, 'SAVED', String(saved).padStart(3, '0'), '#8de2bd'],
        [320, 'DISCARDED', String(discarded).padStart(3, '0'), '#f7c66d'],
        [568, 'LAST ACTION', lastEvent, '#f4f7fb'],
    ];

    hudContext.clearRect(0, 0, HUD_WIDTH_PX, HUD_HEIGHT_PX);
    hudContext.beginPath();
    hudContext.roundRect(8, 8, HUD_WIDTH_PX - 16, HUD_HEIGHT_PX - 16, 34);
    hudContext.fillStyle = 'rgba(4, 9, 17, 0.88)';
    hudContext.fill();
    hudContext.strokeStyle = 'rgba(188, 210, 235, 0.35)';
    hudContext.lineWidth = 3;
    hudContext.stroke();

    let x = 0;
    for (const [width, label, value, color, dotColor] of sections) {
        drawHudSection(hudContext, x, width, label, value, color, dotColor);
        x += width;
    }

    gl.bindTexture(gl.TEXTURE_2D, hudTexture);
    gl.texImage2D(
        gl.TEXTURE_2D,
        0,
        gl.RGBA,
        gl.RGBA,
        gl.UNSIGNED_BYTE,
        hudCanvas,
    );
}

function renderCollectionHud(view, viewport) {
    if (!episodeStatus || !hudPlaced) return;
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    renderTexturedQuad(
        view,
        viewport,
        hudTexture,
        hudModelMatrix,
        view.transform.inverse.matrix,
    );
    gl.disable(gl.BLEND);
}

function sendPose(handedness, pose) {
    const pos = pose.transform.position;
    const rot = pose.transform.orientation;
    const nowMs = Date.now();
    const poseStamped = new geometry_msgs.PoseStamped({
        header: new std_msgs.Header({
            stamp: new std_msgs.Time({ sec: Math.floor(nowMs / 1000), nsec: (nowMs % 1000) * 1_000_000 }),
            frame_id: handedness
        }),
        pose: new geometry_msgs.Pose({
            position: new geometry_msgs.Point({ x: pos.x, y: pos.y, z: pos.z }),
            orientation: new geometry_msgs.Quaternion({ x: rot.x, y: rot.y, z: rot.z, w: rot.w })
        })
    });
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(poseStamped.encode());
    }
}

function sendJoy(handedness, axes, buttons) {
    const nowMs = Date.now();
    const joyMsg = new sensor_msgs.Joy({
        header: new std_msgs.Header({
            stamp: new std_msgs.Time({ sec: Math.floor(nowMs / 1000), nsec: (nowMs % 1000) * 1_000_000 }),
            frame_id: handedness
        }),
        axes_length: axes.length,
        buttons_length: buttons.length,
        axes: axes,
        buttons: buttons
    });
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(joyMsg.encode());
    }
}

// Send raw controller and wrist tracking data (no processing - done in Python)
function processTracking(frame) {
    // Rate limit tracking data
    const now = performance.now();
    if (now - lastSendTime < sendInterval) {
        return;
    }
    lastSendTime = now;

    // Process controller and hand input sources.
    for (const inputSource of frame.session.inputSources) {
        const handedness = inputSource.handedness;
        if (handedness !== 'left' && handedness !== 'right') continue;

        const hand = inputSource.hand;
        if (hand) {
            const wristPose = frame.getJointPose(hand.get('wrist'), xrRefSpace);
            const thumbTipPose = frame.getJointPose(hand.get('thumb-tip'), xrRefSpace);
            const middleTipPose = frame.getJointPose(hand.get('middle-finger-tip'), xrRefSpace);
            if (!wristPose || !thumbTipPose || !middleTipPose) continue;

            const thumb = thumbTipPose.transform.position;
            const middle = middleTipPose.transform.position;
            const gripperPinched = Math.hypot(
                thumb.x - middle.x,
                thumb.y - middle.y,
                thumb.z - middle.z,
            ) < GRIPPER_PINCH_DISTANCE_METERS;

            sendPose(handedness, wristPose);
            // Index pinch selects arm tracking; middle pinch closes the gripper.
            sendJoy(
                handedness,
                [0, 0, gripperPinched ? 1 : 0, 0],
                [0, 0, 0, 0, handSelectActive.get(handedness) ? 1 : 0, 0, 0],
            );
            continue;
        }

        const trackingSpace = inputSource.gripSpace || inputSource.targetRaySpace;
        if (!trackingSpace) continue;
        const pose = frame.getPose(trackingSpace, xrRefSpace);
        if (!pose) continue;

        sendPose(handedness, pose);

        // Send Joy message with all buttons and axes
        const gamepad = inputSource.gamepad;
        if (gamepad) {
            const isXrStandard = gamepad.mapping === 'xr-standard';
            const stickX = (isXrStandard ? gamepad.axes[2] : gamepad.axes[0]) ?? 0.0;
            const stickY = (isXrStandard ? gamepad.axes[3] : gamepad.axes[1]) ?? 0.0;
            const axes = [
                stickX,
                stickY,
                gamepad.buttons[0]?.value ?? 0.0,
                gamepad.buttons[1]?.value ?? 0.0,
            ];

            // Buttons layout (int32, 0 or 1):
            // [0] = trigger (digital)
            // [1] = grip (digital)
            // [2] = touchpad press
            // [3] = thumbstick press
            // [4] = X/A button
            // [5] = Y/B button
            // [6] = menu (if exposed)
            const buttons = [];
            for (let i = 0; i < gamepad.buttons.length; i++) {
                buttons.push(gamepad.buttons[i]?.pressed ? 1 : 0);
            }

            sendJoy(handedness, axes, buttons);
        }
    }
}

// VR render loop
function onXRFrame(_time, frame) {
    if (!xrSession) return;
    xrSession.requestAnimationFrame(onXRFrame);
    // Process and send tracking data
    processTracking(frame);

    const glLayer = xrSession.renderState.baseLayer;
    gl.bindFramebuffer(gl.FRAMEBUFFER, glLayer.framebuffer);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    // Keep rendering the existing texture between loads to avoid blinking.
    if (videoReady && videoDirty && videoEl.naturalWidth) {
        uploadVideoTexture();
        videoDirty = false;
    }
    if (videoReady) updateVideoModelMatrix();
    updateHudTexture();
    const pose = frame.getViewerPose(xrRefSpace);
    if (pose) {
        if (!hudPlaced) placeCollectionHud(pose);
        for (const view of pose.views) {
            const viewport = glLayer.getViewport(view);
            if (videoReady) renderVideoPanel(view, viewport);
            renderCollectionHud(view, viewport);
        }
    }
}

// Start VR session with passthrough
async function startVR() {
    try {
        setStatus('Initializing WebGL...');
        initGL();
        setStatus('Requesting VR session...');

        // Try immersive-ar first (true passthrough), fall back to immersive-vr
        let session = null;
        try {
            session = await navigator.xr.requestSession('immersive-ar', {
                requiredFeatures: ['local-floor'],
                optionalFeatures: ['hand-tracking']
            });
            console.log('Started immersive-ar session (passthrough)');
        } catch (arError) {
            console.log('immersive-ar not available, trying immersive-vr');
            session = await navigator.xr.requestSession('immersive-vr', {
                requiredFeatures: ['local-floor'],
                optionalFeatures: ['hand-tracking']
            });
            console.log('Started immersive-vr session');
        }

        xrSession = session;
        hudPlaced = false;

        // Setup WebGL layer
        const glLayer = new XRWebGLLayer(session, gl);
        await session.updateRenderState({
            baseLayer: glLayer
        });

        // Get reference space
        xrRefSpace = await session.requestReferenceSpace('local-floor');

        setStatus('VR active');

        // Session event handlers
        session.addEventListener('end', () => {
            setStatus('VR session ended');
            handSelectActive.clear();
            hudPlaced = false;
            xrSession = null;
            window.disconnect();
        });

        session.addEventListener('selectstart', (event) => {
            const { handedness, hand } = event.inputSource;
            if (hand && (handedness === 'left' || handedness === 'right')) {
                handSelectActive.set(handedness, true);
            }
        });
        session.addEventListener('selectend', (event) => {
            const { handedness, hand } = event.inputSource;
            if (hand && (handedness === 'left' || handedness === 'right')) {
                handSelectActive.set(handedness, false);
            }
        });

        // Start render loop
        session.requestAnimationFrame(onXRFrame);

    } catch (error) {
        setStatus('VR failed: ' + error.message);
        console.error('VR session error:', error);
        throw error;
    }
}

// Connect button handler
window.connect = async function() {
    try {
        connectBtn.disabled = true;

        // Check WebXR support
        if (!navigator.xr) {
            throw new Error('WebXR not supported. Use Quest 3 browser.');
        }

        // Setup WebSocket
        await setupWebSocket();

        // Start VR
        await startVR();

        // Update UI
        connectBtn.classList.add('hidden');
        disconnectBtn.classList.remove('hidden');

    } catch (error) {
        setStatus('Connection failed');
        console.error('Connection error:', error);
        connectBtn.disabled = false;
    }
};

// Disconnect button handler
window.disconnect = async function() {
    setStatus('Disconnecting...');

    if (xrSession) {
        await xrSession.end().catch(console.error);
        xrSession = null;
    }

    if (ws) {
        ws.close();
        ws = null;
    }

    // Update UI
    connectBtn.classList.remove('hidden');
    connectBtn.disabled = false;
    disconnectBtn.classList.add('hidden');
    setStatus('Disconnected');
};

// Check WebXR availability on load
window.addEventListener('load', async () => {
    if (!navigator.xr) {
        setStatus('WebXR not available');
        connectBtn.disabled = true;
        return;
    }

    try {
        // Check for AR (passthrough) or VR support
        const arSupported = await navigator.xr.isSessionSupported('immersive-ar').catch(() => false);
        const vrSupported = await navigator.xr.isSessionSupported('immersive-vr').catch(() => false);

        if (!arSupported && !vrSupported) {
            setStatus('VR/AR not supported');
            connectBtn.disabled = true;
        }
    } catch (error) {
        console.error('WebXR check failed:', error);
    }
});
