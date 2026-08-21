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
let videoProgram = null;
let videoVbo = null;
let videoAttribs = null;
let videoUniforms = null;
let videoReady = false;  // true after first frame loads
let videoDirty = false;  // true when a new JPEG has finished decoding
let videoAspect = 1.0;   // cached at load time — see videoEl.onload
let prevBlobUrl = null;  // revoked when the next-next blob arrives
const videoModelMatrix = new Float32Array(16);

const PANEL_POS_X = 0.0;
const PANEL_POS_Y = 1.4;   // ~eye height
const PANEL_POS_Z = -1.5;  // 1.5m in front of starting position
const PANEL_HEIGHT = 0.9;

// Collection HUD state. The texture is rendered in view space so it stays
// fixed near the bottom of the operator's view while their head moves.
let episodeStatus = null;
let episodeStatusReceivedAtMs = 0;
let hudOffline = false;
let hudCanvas = null;
let hudContext = null;
let hudTexture = null;
let hudProgram = null;
let hudVbo = null;
let hudAttribs = null;
let hudUniforms = null;
let hudDirty = false;
let hudElapsedSecond = -1;

const HUD_WIDTH_PX = 2048;
const HUD_HEIGHT_PX = 256;
const HUD_WIDTH_METERS = 1.08;
const HUD_HEIGHT_METERS = HUD_WIDTH_METERS * HUD_HEIGHT_PX / HUD_WIDTH_PX;
const HUD_CENTER_Y = -0.34;
const HUD_CENTER_Z = -1.2;

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

// Compile + link a textured-quad pipeline that renders the camera
// feed as a world-locked panel inside the WebXR scene.
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
    videoProgram = gl.createProgram();
    gl.attachShader(videoProgram, vs);
    gl.attachShader(videoProgram, fs);
    gl.linkProgram(videoProgram);
    if (!gl.getProgramParameter(videoProgram, gl.LINK_STATUS)) {
        throw new Error('Program link failed: ' + gl.getProgramInfoLog(videoProgram));
    }

    // Quad as TRIANGLE_STRIP: x, y, u, v. v=0 at top of quad +
    // UNPACK_FLIP_Y_WEBGL=false → image displays upright.
    const verts = new Float32Array([
        -1, -1, 0, 1,
         1, -1, 1, 1,
        -1,  1, 0, 0,
         1,  1, 1, 0,
    ]);
    videoVbo = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, videoVbo);
    gl.bufferData(gl.ARRAY_BUFFER, verts, gl.STATIC_DRAW);

    videoAttribs = {
        pos: gl.getAttribLocation(videoProgram, 'a_pos'),
        uv:  gl.getAttribLocation(videoProgram, 'a_uv'),
    };
    videoUniforms = {
        proj:  gl.getUniformLocation(videoProgram, 'u_proj'),
        view:  gl.getUniformLocation(videoProgram, 'u_view'),
        model: gl.getUniformLocation(videoProgram, 'u_model'),
        tex:   gl.getUniformLocation(videoProgram, 'u_tex'),
    };

    videoTex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, videoTex);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
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

function uploadVideoTexture() {
    gl.bindTexture(gl.TEXTURE_2D, videoTex);
    gl.texImage2D(
        gl.TEXTURE_2D, 0, gl.RGB, gl.RGB, gl.UNSIGNED_BYTE, videoEl
    );
}

function updateModelMatrix() {
    const halfH = PANEL_HEIGHT * 0.5;
    const halfW = halfH * videoAspect;
    videoModelMatrix.fill(0);
    videoModelMatrix[0] = halfW;
    videoModelMatrix[5] = halfH;
    videoModelMatrix[10] = 1;
    videoModelMatrix[12] = PANEL_POS_X;
    videoModelMatrix[13] = PANEL_POS_Y;
    videoModelMatrix[14] = PANEL_POS_Z;
    videoModelMatrix[15] = 1;
}

function renderVideoPanel(view, viewport) {
    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
    gl.useProgram(videoProgram);

    gl.bindBuffer(gl.ARRAY_BUFFER, videoVbo);
    gl.enableVertexAttribArray(videoAttribs.pos);
    gl.vertexAttribPointer(videoAttribs.pos, 2, gl.FLOAT, false, 16, 0);
    gl.enableVertexAttribArray(videoAttribs.uv);
    gl.vertexAttribPointer(videoAttribs.uv,  2, gl.FLOAT, false, 16, 8);

    gl.uniformMatrix4fv(videoUniforms.proj,  false, view.projectionMatrix);
    gl.uniformMatrix4fv(videoUniforms.view,  false, view.transform.inverse.matrix);
    gl.uniformMatrix4fv(videoUniforms.model, false, videoModelMatrix);

    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, videoTex);
    gl.uniform1i(videoUniforms.tex, 0);

    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
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

    const vsSrc = `
        attribute vec2 a_pos;
        attribute vec2 a_uv;
        uniform mat4 u_proj;
        varying vec2 v_uv;
        void main() {
            vec3 position = vec3(
                a_pos.x * ${HUD_WIDTH_METERS * 0.5},
                ${HUD_CENTER_Y} + a_pos.y * ${HUD_HEIGHT_METERS * 0.5},
                ${HUD_CENTER_Z}
            );
            gl_Position = u_proj * vec4(position, 1.0);
            v_uv = a_uv;
        }`;
    const fsSrc = `
        precision mediump float;
        varying vec2 v_uv;
        uniform sampler2D u_tex;
        void main() {
            gl_FragColor = texture2D(u_tex, v_uv);
        }`;

    const compile = (type, source) => {
        const shader = gl.createShader(type);
        gl.shaderSource(shader, source);
        gl.compileShader(shader);
        if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
            throw new Error(
                'HUD shader compile failed: ' + gl.getShaderInfoLog(shader),
            );
        }
        return shader;
    };

    hudProgram = gl.createProgram();
    gl.attachShader(hudProgram, compile(gl.VERTEX_SHADER, vsSrc));
    gl.attachShader(hudProgram, compile(gl.FRAGMENT_SHADER, fsSrc));
    gl.linkProgram(hudProgram);
    if (!gl.getProgramParameter(hudProgram, gl.LINK_STATUS)) {
        throw new Error(
            'HUD program link failed: ' + gl.getProgramInfoLog(hudProgram),
        );
    }

    const verts = new Float32Array([
        -1,
        -1,
        0,
        1,
        1,
        -1,
        1,
        1,
        -1,
        1,
        0,
        0,
        1,
        1,
        1,
        0,
    ]);
    hudVbo = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, hudVbo);
    gl.bufferData(gl.ARRAY_BUFFER, verts, gl.STATIC_DRAW);

    hudAttribs = {
        pos: gl.getAttribLocation(hudProgram, 'a_pos'),
        uv: gl.getAttribLocation(hudProgram, 'a_uv'),
    };
    hudUniforms = {
        proj: gl.getUniformLocation(hudProgram, 'u_proj'),
        tex: gl.getUniformLocation(hudProgram, 'u_tex'),
    };

    hudTexture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, hudTexture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
}

function roundedRect(context, x, y, width, height, radius) {
    context.beginPath();
    context.moveTo(x + radius, y);
    context.lineTo(x + width - radius, y);
    context.quadraticCurveTo(x + width, y, x + width, y + radius);
    context.lineTo(x + width, y + height - radius);
    context.quadraticCurveTo(
        x + width,
        y + height,
        x + width - radius,
        y + height,
    );
    context.lineTo(x + radius, y + height);
    context.quadraticCurveTo(x, y + height, x, y + height - radius);
    context.lineTo(x, y + radius);
    context.quadraticCurveTo(x, y, x + radius, y);
    context.closePath();
}

function fitHudText(context, text, maxWidth) {
    if (context.measureText(text).width <= maxWidth) return text;
    let shortened = text;
    while (
        shortened.length > 1 &&
        context.measureText(shortened + '...').width > maxWidth
    ) {
        shortened = shortened.slice(0, -1);
    }
    return shortened + '...';
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
    context.fillStyle = '#93a7bf';
    context.font = '600 27px sans-serif';
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
    context.font = '600 43px ui-monospace, SFMono-Regular, Menlo, monospace';
    context.fillText(
        fitHudText(context, value, x + width - valueLeft - 24),
        valueLeft,
        174,
    );
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
        [310, 'STATE', state, '#f4f7fb', stateColor],
        [160, 'TAKE', String(saved + discarded + 1).padStart(3, '0'), '#f4f7fb'],
        [210, 'ELAPSED', hudOffline ? '--:--' : formatElapsed(elapsed), '#f4f7fb'],
        [230, 'COLLECTED', String(saved).padStart(3, '0'), '#8de2bd'],
        [240, 'DISCARDED', String(discarded).padStart(3, '0'), '#f7c66d'],
        [180, 'LAST', lastEvent, '#f4f7fb'],
        [718, 'TASK', episodeStatus.task_label || 'UNASSIGNED', '#f4f7fb'],
    ];

    hudContext.clearRect(0, 0, HUD_WIDTH_PX, HUD_HEIGHT_PX);
    roundedRect(hudContext, 8, 8, HUD_WIDTH_PX - 16, HUD_HEIGHT_PX - 16, 34);
    hudContext.fillStyle = 'rgba(8, 16, 29, 0.76)';
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
    if (!episodeStatus) return;
    gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
    gl.useProgram(hudProgram);
    gl.bindBuffer(gl.ARRAY_BUFFER, hudVbo);
    gl.enableVertexAttribArray(hudAttribs.pos);
    gl.vertexAttribPointer(hudAttribs.pos, 2, gl.FLOAT, false, 16, 0);
    gl.enableVertexAttribArray(hudAttribs.uv);
    gl.vertexAttribPointer(hudAttribs.uv, 2, gl.FLOAT, false, 16, 8);
    gl.uniformMatrix4fv(hudUniforms.proj, false, view.projectionMatrix);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, hudTexture);
    gl.uniform1i(hudUniforms.tex, 0);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
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
    if (videoReady) updateModelMatrix();
    updateHudTexture();
    const pose = frame.getViewerPose(xrRefSpace);
    if (pose) {
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
