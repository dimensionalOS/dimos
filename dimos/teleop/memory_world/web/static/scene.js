// Three.js scene — first-person walkthrough of a recorded point cloud.
//
// Coordinate frames:
//   robot frame:  X forward, Y left, Z up   (data on the wire is in this frame)
//   three.js:     X right,   Y up,    Z back (right-handed)
//
// We parent all world data under a "frame-rotate" group that applies a -90°
// rotation around X, which maps (rx, ry, rz) -> (rx, rz, -ry). Outside that
// rotate group, normal Y-up three.js logic applies.
//
// Locomotion strategy: we don't move the camera (WebXR drives it). Instead
// we translate / rotate / scale `_worldGroup`, which contains everything the
// user is looking at. Walking forward = world moves backward, etc.
//
// Public methods (called by main.js):
//   setSession(session, perFrame)
//   setPointCloud(header, payloadArrayBuffer)
//   setImagePoses(header, payloadArrayBuffer)
//   setOdomTrail(header, payloadArrayBuffer)
//   applyLocomote({stickX, stickY, dt})
//   applySnapTurn({sign})
//   setTeleportAim({originWorld, dirWorld})
//   clearTeleportAim()
//   applyTeleportCommit()
//   applyScale({factor, pivotWorld})
//   resetView()
//
// Public read-only helpers used by InputAdapter:
//   getCameraForwardXZ() -> [x, z] unit vector in world space
//   getCameraPositionWorld() -> THREE.Vector3
//   worldToRobot(point)  -- for diag / future use

import * as THREE from 'https://esm.sh/three@0.160.0';

const WALK_SPEED_M_PER_S = 1.4;               // headset-relative
const POINT_SIZE = 0.025;                     // metres
const TELEPORT_ARC_SEGMENTS = 24;
const TELEPORT_MAX_DISTANCE = 8.0;            // metres along ray
const MIN_SCALE = 0.05;
const MAX_SCALE = 10.0;
// Desktop (non-XR) fallback. WebXR normally supplies the head pose; without a
// headset we drive `camera` ourselves from mouse-look at a fixed standing height.
const EYE_HEIGHT_M = 1.6;
const DESKTOP_LOOK_SENSITIVITY = 0.0022;      // radians per pixel of mouse travel
const DESKTOP_PITCH_LIMIT = 1.45;             // just under 90deg, avoids gimbal flip
const DESKTOP_SPRINT_MULTIPLIER = 3.0;
const DESKTOP_SCALE_STEP = 1.08;              // per wheel notch
const DESKTOP_MOVE_KEYS = new Set(['KeyW', 'KeyA', 'KeyS', 'KeyD']);
const TOUCH_LOOK_SENSITIVITY = 0.006;         // radians per CSS pixel of one-finger drag
const TOUCH_WALK_GAIN = 40;                   // two-finger drag: a screen-height sweep = full stick x40
// GTA-style HUD minimap — head-locked, sits at lower-left of view.
const HUD_PANEL_SIZE = 0.22;          // metres (square)
const HUD_MARKER_RADIUS = 0.008;
const HUD_DISTANCE = 0.55;            // metres in front of head
const HUD_OFFSET_DOWN = 0.25;
const HUD_OFFSET_LEFT = 0.32;
const HUD_FOLLOW_LERP = 0.18;         // damping per frame
const ANSWER_PANEL_W = 0.62;          // metres; the canvas behind it is 4:1
const ANSWER_PANEL_H = 0.155;
// Image-thumbnail quads at capture poses.
const IMAGE_QUAD_W = 0.60;
const IMAGE_QUAD_H = 0.34;            // 16:9-ish
const IMAGE_QUAD_HEIGHT = 0.9;        // robot z (metres) — chest height in VR
// Only poses within this radius of the viewer get a decoded thumbnail, and at
// most this many exist at once. A recording has hundreds of poses; without a
// budget every one becomes its own texture, material and draw call.
const IMAGE_RENDER_DISTANCE_M = 12.0;
const IMAGE_QUAD_BUDGET = 24;
const IMAGE_LOD_INTERVAL_S = 0.2;     // how often the visible set is recomputed
// Rolling window for the frame-time readout, ~4s at 60fps.
const PERF_WINDOW = 240;
// Voxels within a highlighted point's radius are repainted in these. The map
// itself stays inside a navy-to-cyan band, so warm colours read as "answer".
const VOXEL_HIGHLIGHT_COLOR = 0xffb347;
const VOXEL_HIGHLIGHT_FOCUS_COLOR = 0xff5c3a;
// How far in front of the viewer a focused answer is brought.
const FOCUS_DISTANCE_M = 4.0;

export class WorldScene {
    constructor(diag, backgroundMode = 'black') {
        this.diag = diag || (() => {});
        this.backgroundMode = backgroundMode;
        const passthrough = backgroundMode === 'passthrough';

        this.three = new THREE.WebGLRenderer({ alpha: passthrough, antialias: false });
        this.three.setSize(window.innerWidth || 800, window.innerHeight || 600);
        this.three.setPixelRatio(window.devicePixelRatio || 1);
        this.three.xr.enabled = true;
        this.three.setClearColor(0x06090f, passthrough ? 0 : 1);

        const dom = this.three.domElement;
        dom.style.position = 'fixed';
        dom.style.top = '0';
        dom.style.left = '0';
        dom.style.width = '100vw';
        dom.style.height = '100vh';
        dom.style.zIndex = '50';
        dom.style.pointerEvents = 'none';
        document.body.appendChild(dom);

        this.scene = new THREE.Scene();
        this.scene.background = passthrough ? null : new THREE.Color(0x06090f);
        this.scene.add(new THREE.AmbientLight(0xffffff, 0.45));
        const sun = new THREE.DirectionalLight(0xffffff, 1.3);
        sun.position.set(2, 4, 3);
        this.scene.add(sun);

        this.camera = new THREE.PerspectiveCamera(70, 1, 0.05, 500);

        // _worldGroup is moved by locomotion. _frameRotate inside it converts
        // robot-Z-up into three-Y-up, so the rest of the code can think purely
        // in robot coords (x forward, y left, z up).
        this._worldGroup = new THREE.Group();
        this._frameRotate = new THREE.Group();
        this._frameRotate.rotation.x = -Math.PI / 2;
        this._worldGroup.add(this._frameRotate);
        this.scene.add(this._worldGroup);

        // Origin grid in robot frame for visual reference (10m, 1m cells).
        const grid = new THREE.GridHelper(20, 20, 0x1f2a3a, 0x1f2a3a);
        grid.rotation.x = Math.PI / 2;             // grid is XZ in three; we want XY in robot
        this._frameRotate.add(grid);

        // Containers we (re)populate on payload receive.
        this._pointsObj = null;               // THREE.InstancedMesh
        this._cloudData = null;               // {n, positions, colors, voxelSize}
        this._imagePoseGroup = new THREE.Group();     // always-on ring markers
        this._frameRotate.add(this._imagePoseGroup);
        this._imageRings = null;                      // THREE.InstancedMesh
        this._imageQuadGroup = new THREE.Group();     // textured quads, toggleable
        this._imageQuadGroup.visible = false;
        this._frameRotate.add(this._imageQuadGroup);
        this._imagePoseMeta = [];                     // per-index {pos, quat}
        this._imageQuadsByIndex = new Map();          // index -> THREE.Mesh
        this._thumbnailBytes = new Map();             // index -> ArrayBuffer, decoded on demand
        this._thumbnailDecoding = new Set();
        this._imageQuadGeom = new THREE.PlaneGeometry(IMAGE_QUAD_W, IMAGE_QUAD_H);
        this._imageLodAccumS = IMAGE_LOD_INTERVAL_S;
        this._selectedImageIds = new Set();
        this._odomLine = null;

        // Query results are independent from the recorded odom and can be
        // replaced atomically when a new answer arrives.
        this._highlightGroup = new THREE.Group();
        this._frameRotate.add(this._highlightGroup);
        this._highlightedVoxels = [];                 // instance indices repainted by the last result
        this._lastResultPoints = [];                  // so a rebuilt cloud gets repainted too
        this._activeQueryId = null;                   // query images for any other id are stale
        this._queryImages = [];                       // headers of the frames behind the last answer
        this._queryImageMeshes = [];                  // their quads, so one can be shown alone
        this._queryImageCursor = -1;

        // Top-down map: shared texture, used twice (ground projection + HUD).
        this._topDownTex = null;
        this._topDownBounds = null;
        this._groundMesh = null;

        // HUD minimap — head-locked panel attached to scene root (not world).
        this._hudGroup = new THREE.Group();
        this.scene.add(this._hudGroup);
        this._hudPanelMat = new THREE.MeshBasicMaterial({
            color: 0x182a40,
            transparent: true,
            opacity: 0.85,
            side: THREE.DoubleSide,
        });
        this._hudPanel = new THREE.Mesh(
            new THREE.PlaneGeometry(HUD_PANEL_SIZE, HUD_PANEL_SIZE),
            this._hudPanelMat,
        );
        this._hudGroup.add(this._hudPanel);
        this._hudMarker = new THREE.Mesh(
            new THREE.CircleGeometry(HUD_MARKER_RADIUS, 16),
            new THREE.MeshBasicMaterial({ color: 0xff3344 }),
        );
        // Marker is child of the panel — its local XY is mm in panel space.
        this._hudPanel.add(this._hudMarker);
        this._hudMarker.position.z = 0.001;           // avoid z-fight
        // Heading needle (small line in front of marker showing camera forward).
        this._hudHeading = new THREE.Line(
            new THREE.BufferGeometry().setFromPoints([
                new THREE.Vector3(0, 0, 0),
                new THREE.Vector3(0, 0.04, 0),
            ]),
            new THREE.LineBasicMaterial({ color: 0xff3344 }),
        );
        this._hudPanel.add(this._hudHeading);
        this._hudHeading.position.z = 0.001;

        const answerCanvas = document.createElement('canvas');
        answerCanvas.width = 1024;
        answerCanvas.height = 256;
        this._answerCanvas = answerCanvas;
        this._answerTexture = new THREE.CanvasTexture(answerCanvas);
        this._answerPanel = new THREE.Mesh(
            new THREE.PlaneGeometry(ANSWER_PANEL_W, ANSWER_PANEL_H),
            new THREE.MeshBasicMaterial({
                map: this._answerTexture,
                transparent: true,
                depthTest: false,
                side: THREE.DoubleSide,
            }),
        );
        this._answerPanel.position.set(0, 0.2, 0.002);
        this._answerPanel.visible = false;
        this._hudGroup.add(this._answerPanel);

        // Teleport-aim visualisation.
        this._teleportArc = null;
        this._teleportTarget = new THREE.Vector3();
        this._teleportTargetValid = false;
        this._teleportMarker = new THREE.Mesh(
            new THREE.RingGeometry(0.12, 0.18, 32),
            new THREE.MeshBasicMaterial({ color: 0x7af0a8, transparent: true, opacity: 0.0, side: THREE.DoubleSide }),
        );
        this._teleportMarker.rotation.x = -Math.PI / 2;
        this.scene.add(this._teleportMarker);

        // Per-frame state passed by main.js's input dispatch.
        this._pendingLocomote = null;     // {stickX, stickY}
        this._pendingYawRate = 0;         // rad/s, integrated each tick
        this._lastTickMs = 0;

        // Spawned-yet flag — first cloud arrival recenters us.
        this._hasSpawned = false;

        this._frameSamples = new Float32Array(PERF_WINDOW);
        this._frameSampleCount = 0;
        this._frameSampleCursor = 0;
    }

    /** Drop the frame-time window so a measurement starts from the current state. */
    resetPerf() {
        this._frameSampleCount = 0;
        this._frameSampleCursor = 0;
    }

    /** Cost of one render, measured off the vsync clock.
     *
     * A 120Hz desktop pins every configuration at 8.3ms, which hides exactly the
     * regressions this viewer has to avoid on a headset. Rendering back to back
     * and reading a single pixel afterwards forces the GPU to finish, so the
     * result is the real per-frame cost.
     */
    benchmarkRender(frames = 120) {
        const gl = this.three.getContext();
        const pixel = new Uint8Array(4);
        this.three.render(this.scene, this.camera);
        gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, pixel);
        const started = performance.now();
        for (let i = 0; i < frames; i++) this.three.render(this.scene, this.camera);
        gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, pixel);
        const perFrameMs = (performance.now() - started) / frames;
        return { per_frame_ms: perFrameMs, frames, ...this.getPerfStats() };
    }

    /** Rolling render cost. Medians, because a single GC spike is not the story. */
    getPerfStats() {
        const filled = this._frameSamples.slice(0, this._frameSampleCount);
        const sorted = Array.from(filled).sort((a, b) => a - b);
        const at = (q) => (sorted.length ? sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * q))] : 0);
        const median = at(0.5);
        const info = this.three.info;
        return {
            fps: median > 0 ? 1000 / median : 0,
            median_ms: median,
            p95_ms: at(0.95),
            samples: sorted.length,
            draw_calls: info.render.calls,
            triangles: info.render.triangles,
            textures: info.memory.textures,
            geometries: info.memory.geometries,
            live_quads: this._imageQuadsByIndex.size,
            images_visible: this._imageQuadGroup.visible,
            cloud_visible: Boolean(this._pointsObj && this._pointsObj.visible),
        };
    }

    // ---- session bootstrap ------------------------------------------------

    async setSession(session, perFrame) {
        const gl = this.three.getContext();
        if (gl && gl.makeXRCompatible) {
            try { await gl.makeXRCompatible(); this.diag('gl_xr_compatible'); }
            catch (e) { this.diag('make_xr_compatible_failed', { error: String(e.message || e) }); }
        }

        this.three.xr.setReferenceSpaceType('local-floor');
        await this.three.xr.setSession(session);
        this.diag('three_xr_session_set');

        this.three.setAnimationLoop((time, frame) => {
            if (perFrame) perFrame(frame);
            this._tick(time);
            this.three.render(this.scene, this.camera);
        });
    }

    /** Stop rendering and drop the canvas, so a reconnect doesn't stack a second one. */
    dispose() {
        this.three.setAnimationLoop(null);
        this.three.domElement.remove();
        this.three.dispose();
    }

    /** Flat mouse-and-keyboard walkthrough for machines with no XR hardware. */
    startDesktop(perFrame) {
        this._desktopKeys = new Set();
        this._desktopYaw = 0;
        this._desktopPitch = 0;
        this._desktopSprint = false;
        this._desktopDragging = false;

        this.camera.rotation.order = 'YXZ';
        this.camera.position.set(0, EYE_HEIGHT_M, 0);
        this._resizeDesktopCamera();

        const dom = this.three.domElement;
        dom.style.pointerEvents = 'auto';
        dom.style.cursor = 'grab';
        // Two look modes: drag works everywhere (including headless automation,
        // where pointer lock is never granted), double-click opts into free-look.
        dom.addEventListener('mousedown', () => {
            this._desktopDragging = true;
            dom.style.cursor = 'grabbing';
        });
        window.addEventListener('mouseup', () => {
            this._desktopDragging = false;
            if (document.pointerLockElement !== dom) dom.style.cursor = 'grab';
        });
        dom.addEventListener('dblclick', () => {
            if (document.pointerLockElement !== dom) dom.requestPointerLock();
        });
        dom.addEventListener('wheel', (event) => {
            event.preventDefault();
            this.applyScale({ factor: event.deltaY < 0 ? DESKTOP_SCALE_STEP : 1 / DESKTOP_SCALE_STEP });
        }, { passive: false });

        document.addEventListener('pointerlockchange', () => {
            const locked = document.pointerLockElement === dom;
            dom.style.cursor = locked ? 'none' : 'grab';
            if (!locked) this._desktopKeys.clear();
        });
        document.addEventListener('mousemove', (event) => {
            if (document.pointerLockElement !== dom && !this._desktopDragging) return;
            this._desktopYaw -= event.movementX * DESKTOP_LOOK_SENSITIVITY;
            this._desktopPitch -= event.movementY * DESKTOP_LOOK_SENSITIVITY;
            this._desktopPitch = Math.max(-DESKTOP_PITCH_LIMIT, Math.min(DESKTOP_PITCH_LIMIT, this._desktopPitch));
            this.camera.rotation.set(this._desktopPitch, this._desktopYaw, 0);
        });
        window.addEventListener('keydown', (event) => this._onDesktopKey(event, true));
        window.addEventListener('keyup', (event) => this._onDesktopKey(event, false));
        window.addEventListener('resize', () => this._resizeDesktopCamera());
        this._installTouch(dom);

        this.three.setAnimationLoop((time) => {
            this._applyDesktopKeys();
            if (perFrame) perFrame();
            this._tick(time);
            this.three.render(this.scene, this.camera);
        });
        this.diag('desktop_loop_started');
    }

    _resizeDesktopCamera() {
        const width = window.innerWidth || 800;
        const height = window.innerHeight || 600;
        this.three.setSize(width, height);
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
    }

    _onDesktopKey(event, isDown) {
        this._desktopSprint = event.shiftKey;
        if (DESKTOP_MOVE_KEYS.has(event.code)) {
            event.preventDefault();
            if (isDown) this._desktopKeys.add(event.code);
            else this._desktopKeys.delete(event.code);
            return;
        }
        if (!isDown) return;
        if (event.code === 'KeyR') this.resetView();
        // J: stand where the best answer's camera stood (a known clear spot),
        // or, without a frame, bring the answer point in front of the viewer.
        if (event.code === 'KeyJ' && this._lastResultPoints.length && !this.viewFrom(0)) {
            this.focusOn(this._lastResultPoints[0].position);
        }
        if (event.code === 'KeyP' && this._queryImages.length) {
            this.viewFrom((this._queryImageCursor + 1) % this._queryImages.length);
        }
        else if (event.code === 'KeyI') this.toggleImages();
        else if (event.code === 'KeyV') this.toggleCloud();
    }

    _applyDesktopKeys() {
        const keys = this._desktopKeys;
        // _walk() scales by stick magnitude, so sprint is just a bigger deflection.
        const gain = this._desktopSprint ? DESKTOP_SPRINT_MULTIPLIER : 1;
        const touch = this._touchStick || { x: 0, y: 0 };
        this.applyLocomote({
            stickX: ((keys.has('KeyD') ? 1 : 0) - (keys.has('KeyA') ? 1 : 0)) * gain + touch.x,
            stickY: ((keys.has('KeyS') ? 1 : 0) - (keys.has('KeyW') ? 1 : 0)) * gain + touch.y,
        });
    }

    /** Phone controls: one finger looks, two fingers walk (drag) and scale (pinch). */
    _installTouch(dom) {
        dom.style.touchAction = 'none';
        this._touchStick = { x: 0, y: 0 };
        let last = null;   // {x, y} of one finger, or {x, y, spread} of two
        const centre = (touches) => {
            const points = Array.from(touches);
            const x = points.reduce((sum, t) => sum + t.clientX, 0) / points.length;
            const y = points.reduce((sum, t) => sum + t.clientY, 0) / points.length;
            const spread = points.length > 1
                ? Math.hypot(points[0].clientX - points[1].clientX, points[0].clientY - points[1].clientY)
                : 0;
            return { x, y, spread, count: points.length };
        };
        const begin = (event) => { last = centre(event.touches); };
        dom.addEventListener('touchstart', (event) => { event.preventDefault(); begin(event); }, { passive: false });
        dom.addEventListener('touchmove', (event) => {
            event.preventDefault();
            const now = centre(event.touches);
            if (!last || last.count !== now.count) { last = now; return; }
            if (now.count === 1) {
                this._desktopYaw -= (now.x - last.x) * TOUCH_LOOK_SENSITIVITY;
                this._desktopPitch -= (now.y - last.y) * TOUCH_LOOK_SENSITIVITY;
                this._desktopPitch = Math.max(-DESKTOP_PITCH_LIMIT, Math.min(DESKTOP_PITCH_LIMIT, this._desktopPitch));
                this.camera.rotation.set(this._desktopPitch, this._desktopYaw, 0);
            } else {
                // Two fingers: drag walks (up = forward), pinch scales the world.
                const height = dom.clientHeight || 1;
                this._touchStick = {
                    x: Math.max(-1, Math.min(1, (now.x - last.x) / height * TOUCH_WALK_GAIN)),
                    y: Math.max(-1, Math.min(1, (now.y - last.y) / height * TOUCH_WALK_GAIN)),
                };
                if (last.spread > 0 && now.spread > 0) this.applyScale({ factor: now.spread / last.spread });
            }
            last = now;
        }, { passive: false });
        const end = (event) => {
            this._touchStick = { x: 0, y: 0 };
            last = event.touches.length ? centre(event.touches) : null;
        };
        dom.addEventListener('touchend', end);
        dom.addEventListener('touchcancel', end);
    }

    _tick(timeMs) {
        const frameMs = this._lastTickMs ? timeMs - this._lastTickMs : 0;
        if (frameMs > 0) {
            this._frameSamples[this._frameSampleCursor] = frameMs;
            this._frameSampleCursor = (this._frameSampleCursor + 1) % PERF_WINDOW;
            this._frameSampleCount = Math.min(this._frameSampleCount + 1, PERF_WINDOW);
        }
        const dt = this._lastTickMs ? Math.max((timeMs - this._lastTickMs) / 1000, 0) : 0;
        this._lastTickMs = timeMs;

        const loc = this._pendingLocomote;
        if (loc && dt > 0 && (Math.abs(loc.stickX) > 0.1 || Math.abs(loc.stickY) > 0.1)) {
            this._walk(loc.stickX, loc.stickY, dt);
        }

        if (dt > 0 && Math.abs(this._pendingYawRate) > 1e-3) {
            const pivot = this.getCameraPositionWorld();
            this._rotateWorldAround(pivot, this._pendingYawRate * dt);
        }

        this._updateImageLod(dt);
        this._updateHud();
    }

    _updateHud() {
        // Place the HUD panel relative to the head: forward + down + left in
        // the head's yaw frame, kept upright (pitch ignored) so it doesn't
        // tumble when the user looks up.
        const cam = this.three.xr.isPresenting
            ? this.three.xr.getCamera(this.camera)
            : this.camera;
        const headPos = new THREE.Vector3();
        cam.getWorldPosition(headPos);

        // Extract camera local axes directly from its world matrix. More
        // robust than getWorldDirection in XR mode where the matrix may be
        // set externally and getWorldDirection's auto-update can miss it.
        cam.updateMatrixWorld();
        const right = new THREE.Vector3();
        const fwd = new THREE.Vector3();
        right.setFromMatrixColumn(cam.matrixWorld, 0);   // camera local +X = user's right
        fwd.setFromMatrixColumn(cam.matrixWorld, 2);     // camera local +Z = backward
        fwd.negate();                                    // flip to forward (-Z is forward)
        right.y = 0; fwd.y = 0;
        if (right.lengthSq() < 1e-6 || fwd.lengthSq() < 1e-6) return;
        right.normalize(); fwd.normalize();

        // HUD goes to the user's LEFT, which is -right. A headset's field of
        // view swallows that offset; a desktop window's does not, so there the
        // offset shrinks until the answer panel's far edge stays on screen.
        let offsetLeft = HUD_OFFSET_LEFT;
        if (!this.three.xr.isPresenting) {
            const halfHeight = HUD_DISTANCE * Math.tan(THREE.MathUtils.degToRad(this.camera.fov) / 2);
            const halfWidth = halfHeight * this.camera.aspect;
            // The panel is turned toward the head, so its near edge projects wider than flat: keep a fat margin.
            offsetLeft = Math.max(0, Math.min(HUD_OFFSET_LEFT, halfWidth - ANSWER_PANEL_W / 2 - 0.12));
        }
        const target = new THREE.Vector3()
            .copy(headPos)
            .addScaledVector(fwd, HUD_DISTANCE)
            .addScaledVector(right, -offsetLeft);
        target.y -= HUD_OFFSET_DOWN;
        // Tilt the panel slightly toward the user (downward tilt around X).
        this._hudGroup.position.lerp(target, HUD_FOLLOW_LERP);
        // Face the user — look at head from panel position, then tilt up a bit.
        this._hudGroup.lookAt(headPos);

        // Update marker dot position to where the camera is *in world*.
        // We need the camera's robot-frame XY. Camera is at headPos in three-world;
        // un-apply worldGroup transform + frameRotate to get robot frame.
        if (this._topDownBounds) {
            const robotXY = this._worldPosToRobotXY(headPos);
            if (robotXY) {
                const uv = this._robotXYToHudUV(robotXY[0], robotXY[1]);
                // Panel is HUD_PANEL_SIZE wide centred at (0,0). Map u,v in [0,1]
                // to [-S/2, S/2].
                const s = HUD_PANEL_SIZE;
                this._hudMarker.position.x = (uv[0] - 0.5) * s;
                this._hudMarker.position.y = (0.5 - uv[1]) * s;
                this._hudHeading.position.copy(this._hudMarker.position);
                // Rotate heading needle to match camera yaw in robot frame.
                // robot forward = world fwd transformed back. Easier: yaw
                // in three world is atan2(fwd.x, fwd.z) but we want yaw in
                // the *map* (robot) frame. After frame-rotate (rx = -90°),
                // robot +X is three +X; robot +Y is three -Z. So robot yaw =
                // atan2(world_fwd_x, -world_fwd_z) ... rendered on a Y-up
                // panel where +X is right and +Y is up (map north = robot +Y).
                const robotYaw = Math.atan2(fwd.x, -fwd.z);
                this._hudHeading.rotation.z = -robotYaw;
            }
        }
    }

    _worldPosToRobotXY(worldPos) {
        // Invert worldGroup transform: translate, then inverse R_y(rot.y).
        // R_y(θ): x' = c*x + s*z, z' = -s*x + c*z. Inverse rotation matrix
        // is the transpose: x = c*x' - s*z', z = s*x' + c*z'.
        const wg = this._worldGroup;
        const s = wg.scale.x || 1;
        const dx = (worldPos.x - wg.position.x) / s;
        const dz = (worldPos.z - wg.position.z) / s;
        const c = Math.cos(wg.rotation.y);
        const sn = Math.sin(wg.rotation.y);
        const rx = c * dx - sn * dz;
        const rz = sn * dx + c * dz;
        // Un-apply frame-rotate (R_x(-π/2)): three (x, y, z) -> robot (x, -z, y).
        return [rx, -rz];
    }

    _robotXYToHudUV(rx, ry) {
        // u = (rx - x_min) / (x_max - x_min); v same for ry but flipped.
        const b = this._topDownBounds;
        const u = (rx - b.x_min) / Math.max(b.x_max - b.x_min, 1e-6);
        const v = 1.0 - (ry - b.y_min) / Math.max(b.y_max - b.y_min, 1e-6);
        return [Math.max(0, Math.min(1, u)), Math.max(0, Math.min(1, v))];
    }

    _walk(stickX, stickY, dt) {
        // Quest left stick: forward push = stickY < 0, right push = stickX > 0.
        const fwd = this.getCameraForwardXZ();           // unit, world XZ
        // right = cross(forward, up) in three.js right-handed Y-up coords.
        const right = [-fwd[1], fwd[0]];
        // Desired CAMERA motion in world XZ.
        const camDx = right[0] * stickX + fwd[0] * (-stickY);
        const camDz = right[1] * stickX + fwd[1] * (-stickY);
        // World translates opposite of camera intent.
        const speed = WALK_SPEED_M_PER_S;
        this._worldGroup.position.x -= camDx * speed * dt;
        this._worldGroup.position.z -= camDz * speed * dt;
    }

    // ---- public locomotion API -------------------------------------------

    applyLocomote(g) {
        this._pendingLocomote = { stickX: g.stickX || 0, stickY: g.stickY || 0 };
    }

    applyYaw(g) {
        this._pendingYawRate = g.rate || 0;
    }

    applyTeleportCommit() {
        if (!this._teleportTargetValid) return;
        const head = this.getCameraPositionWorld();
        // Move world so that head sits where the marker is. Keep head Y the
        // same — the user doesn't physically jump.
        const dx = this._teleportTarget.x - head.x;
        const dz = this._teleportTarget.z - head.z;
        this._worldGroup.position.x -= dx;
        this._worldGroup.position.z -= dz;
        this.clearTeleportAim();
    }

    setTeleportAim(g) {
        // g: { originWorld:[x,y,z], dirWorld:[x,y,z] }
        const o = g.originWorld;
        const d = g.dirWorld;
        if (!o || !d) return;

        // Cast a gravity-pulled parabola. y(t) = o.y + d.y*t - 0.5*g*t^2
        // Find t where y(t) == ground (we use floor y = 0 — local-floor origin).
        // Simpler: shoot a straight ray and stop at floor plane, then bend if
        // it'd go above the user. For MVP a straight-ray-to-floor is enough.
        const groundY = 0;
        let t = (groundY - o[1]) / (d[1] < -1e-3 ? d[1] : -1e-3);
        if (t < 0 || t > TELEPORT_MAX_DISTANCE) {
            t = TELEPORT_MAX_DISTANCE;
        }
        const hit = new THREE.Vector3(o[0] + d[0] * t, groundY, o[2] + d[2] * t);
        this._teleportTarget.copy(hit);
        this._teleportTargetValid = true;

        // Render a curved arc from controller to hit. Parabola through the
        // midpoint raised by 1/4 of the horizontal distance.
        const pts = [];
        const start = new THREE.Vector3(o[0], o[1], o[2]);
        const horiz = Math.hypot(hit.x - start.x, hit.z - start.z);
        const apexY = (start.y + hit.y) / 2 + Math.max(0.1, horiz * 0.25);
        for (let i = 0; i <= TELEPORT_ARC_SEGMENTS; i++) {
            const u = i / TELEPORT_ARC_SEGMENTS;
            // Quadratic Bézier with control at (mid.x, apexY, mid.z).
            const cx = (start.x + hit.x) / 2;
            const cz = (start.z + hit.z) / 2;
            const x = (1 - u) ** 2 * start.x + 2 * (1 - u) * u * cx + u * u * hit.x;
            const y = (1 - u) ** 2 * start.y + 2 * (1 - u) * u * apexY + u * u * hit.y;
            const z = (1 - u) ** 2 * start.z + 2 * (1 - u) * u * cz + u * u * hit.z;
            pts.push(new THREE.Vector3(x, y, z));
        }
        if (this._teleportArc) {
            this._teleportArc.geometry.dispose();
            this._teleportArc.geometry = new THREE.BufferGeometry().setFromPoints(pts);
        } else {
            const geom = new THREE.BufferGeometry().setFromPoints(pts);
            const mat = new THREE.LineBasicMaterial({ color: 0x7af0a8, transparent: true, opacity: 0.85 });
            this._teleportArc = new THREE.Line(geom, mat);
            this.scene.add(this._teleportArc);
        }
        this._teleportArc.visible = true;
        this._teleportMarker.position.copy(hit);
        this._teleportMarker.position.y += 0.005;
        this._teleportMarker.material.opacity = 0.9;
    }

    clearTeleportAim() {
        this._teleportTargetValid = false;
        if (this._teleportArc) {
            this._teleportArc.visible = false;
        }
        this._teleportMarker.material.opacity = 0.0;
    }

    applyScale(g) {
        const factor = Math.max(0.2, Math.min(5.0, g.factor || 1.0));
        const pivot = g.pivotWorld
            ? new THREE.Vector3(g.pivotWorld[0], g.pivotWorld[1], g.pivotWorld[2])
            : this.getCameraPositionWorld();

        const newScale = this._worldGroup.scale.x * factor;
        if (newScale < MIN_SCALE || newScale > MAX_SCALE) return;
        this._scaleWorldAround(pivot, factor);
    }

    resetView() {
        this._worldGroup.position.set(0, 0, 0);
        this._worldGroup.rotation.set(0, 0, 0);
        this._worldGroup.scale.set(1, 1, 1);
        this._hasSpawned = false;
        if (this._pointsObj) this._spawnAtCentroid();
    }

    // ---- coordinate helpers ----------------------------------------------

    getCameraForwardXZ() {
        // The XR camera (renderer.xr.getCamera()) reflects head pose. Fall
        // back to the non-XR perspective camera when no session is active.
        const cam = this.three.xr.isPresenting
            ? this.three.xr.getCamera(this.camera)
            : this.camera;
        const fwd = new THREE.Vector3();
        cam.getWorldDirection(fwd);
        fwd.y = 0;
        const len = Math.hypot(fwd.x, fwd.z) || 1;
        return [fwd.x / len, fwd.z / len];
    }

    getCameraPositionWorld() {
        const cam = this.three.xr.isPresenting
            ? this.three.xr.getCamera(this.camera)
            : this.camera;
        const p = new THREE.Vector3();
        cam.getWorldPosition(p);
        return p;
    }

    getViewerRobotPosition() {
        const xy = this._worldPosToRobotXY(this.getCameraPositionWorld());
        return [xy[0], xy[1], 0];
    }

    _rotateWorldAround(pivot, angle) {
        // Apply R_y(angle) to (worldGroup.position - pivot), then translate back.
        // Three.js right-handed Y rotation: x' = c*x + s*z, z' = -s*x + c*z.
        const dx = this._worldGroup.position.x - pivot.x;
        const dz = this._worldGroup.position.z - pivot.z;
        const c = Math.cos(angle), s = Math.sin(angle);
        this._worldGroup.position.x = pivot.x + (c * dx + s * dz);
        this._worldGroup.position.z = pivot.z + (-s * dx + c * dz);
        this._worldGroup.rotation.y += angle;
    }

    _scaleWorldAround(pivot, factor) {
        // Move position toward/away from pivot proportionally.
        this._worldGroup.position.x = pivot.x + factor * (this._worldGroup.position.x - pivot.x);
        this._worldGroup.position.z = pivot.z + factor * (this._worldGroup.position.z - pivot.z);
        this._worldGroup.position.y = pivot.y + factor * (this._worldGroup.position.y - pivot.y);
        this._worldGroup.scale.multiplyScalar(factor);
    }

    // ---- payload handlers -------------------------------------------------

    setPointCloud(header, payloadArrayBuffer) {
        const n = header.n | 0;
        if (n === 0) {
            this.diag('point_cloud_empty');
            return;
        }
        const positions = new Float32Array(payloadArrayBuffer.slice(0, n * 12));
        let colors = null;
        if (header.has_colors) {
            const rgb = new Uint8Array(payloadArrayBuffer, n * 12, n * 3);
            colors = new Float32Array(n * 3);
            for (let i = 0; i < n * 3; i++) colors[i] = rgb[i] / 255;
        }
        this._cloudData = {
            n,
            positions,
            colors,
            voxelSize: (header.voxel_size && header.voxel_size > 0) ? header.voxel_size : POINT_SIZE,
        };
        this._rebuildCloud();

        this._cloudBounds = header.bounds || null;
        if (!this._hasSpawned) this._spawnAtCentroid();
        this.diag('point_cloud_loaded', { n, has_colors: !!header.has_colors });
    }

    _rebuildCloud() {
        const d = this._cloudData;
        if (!d) return;

        if (this._pointsObj) {
            this._frameRotate.remove(this._pointsObj);
            this._pointsObj.geometry.dispose();
            if (this._pointsObj.material) this._pointsObj.material.dispose();
            this._pointsObj = null;
        }

        // Lit cube faces preserve depth cues against both opaque and
        // passthrough backgrounds. Unlit cubes appeared like square sprites.
        const box = new THREE.BoxGeometry(d.voxelSize, d.voxelSize, d.voxelSize);
        const mat = new THREE.MeshStandardMaterial({ roughness: 0.8, metalness: 0 });
        const mesh = new THREE.InstancedMesh(box, mat, d.n);
        mesh.instanceMatrix.setUsage(THREE.StaticDrawUsage);
        const dummy = new THREE.Object3D();
        const col = new THREE.Color();
        for (let i = 0; i < d.n; i++) {
            dummy.position.set(d.positions[i * 3], d.positions[i * 3 + 1], d.positions[i * 3 + 2]);
            dummy.updateMatrix();
            mesh.setMatrixAt(i, dummy.matrix);
            if (d.colors) {
                col.setRGB(d.colors[i * 3], d.colors[i * 3 + 1], d.colors[i * 3 + 2]);
                mesh.setColorAt(i, col);
            }
        }
        mesh.instanceMatrix.needsUpdate = true;
        if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
        this._pointsObj = mesh;
        this._frameRotate.add(this._pointsObj);
        this._highlightedVoxels = [];
        this._highlightVoxels(this._lastResultPoints);
    }

    /** Repaint the voxels around each point that carries a radius; the first
     *  point is the answer and gets the hotter colour. Points without a radius
     *  are capture poses, and painting the floor under the robot would mislead. */
    _highlightVoxels(points) {
        const d = this._cloudData;
        const mesh = this._pointsObj;
        if (!d || !mesh) return;
        const original = new THREE.Color();
        for (const index of this._highlightedVoxels) {
            if (d.colors) {
                original.setRGB(d.colors[index * 3], d.colors[index * 3 + 1], d.colors[index * 3 + 2]);
            } else {
                original.setRGB(1, 1, 1);
            }
            mesh.setColorAt(index, original);
        }
        this._highlightedVoxels = [];
        points.forEach((point, order) => {
            if (!(point.radius > 0)) return;
            const [px, py, pz] = point.position;
            const r2 = point.radius * point.radius;
            const paint = new THREE.Color(order === 0 ? VOXEL_HIGHLIGHT_FOCUS_COLOR : VOXEL_HIGHLIGHT_COLOR);
            for (let i = 0; i < d.n; i++) {
                const dx = d.positions[i * 3] - px;
                const dy = d.positions[i * 3 + 1] - py;
                const dz = d.positions[i * 3 + 2] - pz;
                if (dx * dx + dy * dy + dz * dz <= r2) {
                    mesh.setColorAt(i, paint);
                    this._highlightedVoxels.push(i);
                }
            }
        });
        if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
        this.diag('voxels_highlighted', { n: this._highlightedVoxels.length, points: points.length });
    }

    /** Bring a robot-frame point to a few metres in front of the viewer. On the
     *  desktop the world is also lifted so the point sits at eye level; in VR
     *  the floor stays where the floor is. */
    focusOn(position) {
        const [x, y, z] = position;
        // frameRotate maps robot (x, y, z) to three (x, z, -y); worldGroup then scales and moves it.
        const local = new THREE.Vector3(x, z, -y).multiplyScalar(this._worldGroup.scale.x);
        const head = this.getCameraPositionWorld();
        const fwd = this.getCameraForwardXZ();
        this._worldGroup.position.x = head.x + fwd[0] * FOCUS_DISTANCE_M - local.x;
        this._worldGroup.position.z = head.z + fwd[1] * FOCUS_DISTANCE_M - local.z;
        if (!this.three.xr.isPresenting) {
            this._worldGroup.position.y = head.y - 0.4 - local.y;
        }
        this._queryImageCursor = -1;
        this._queryImageMeshes.forEach((mesh) => { if (mesh) mesh.visible = true; });
        this.diag('focused', { x, y, z });
    }

    toggleCloud() {
        if (!this._pointsObj) return;
        this._pointsObj.visible = !this._pointsObj.visible;
        this.diag('cloud_toggle', { visible: this._pointsObj.visible });
    }

    setImagePoses(header, payloadArrayBuffer) {
        if (this._imageRings) {
            this._imagePoseGroup.remove(this._imageRings);
            this._imageRings.geometry.dispose();
            this._imageRings.material.dispose();
            this._imageRings = null;
        }
        this._releaseAllThumbnails();
        this._thumbnailBytes.clear();
        this._imagePoseMeta = [];

        const n = header.n | 0;
        if (n === 0) return;

        const positions = new Float32Array(payloadArrayBuffer, 0, n * 3);
        const quats = new Float32Array(payloadArrayBuffer, n * 12, n * 4);

        // Default PlaneGeometry normal is +Z. Rotate so the normal points along
        // robot -X (i.e. "behind" the capture direction), so the image is seen
        // face-on when the viewer stands in front of the pose.
        const faceBackward = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -Math.PI / 2);
        const standUpright = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -Math.PI / 2);

        this._imageRings = new THREE.InstancedMesh(
            new THREE.RingGeometry(0.10, 0.13, 24),
            new THREE.MeshBasicMaterial({
                color: 0xffffff,
                transparent: true,
                opacity: 0.7,
                side: THREE.DoubleSide,
            }),
            n,
        );
        this._imageRings.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
        this._imagePoseGroup.add(this._imageRings);

        for (let i = 0; i < n; i++) {
            const quat = new THREE.Quaternion(
                quats[i * 4 + 0], quats[i * 4 + 1], quats[i * 4 + 2], quats[i * 4 + 3],
            ).multiply(faceBackward).multiply(standUpright);
            this._imagePoseMeta.push({
                id: header.ids?.[i] ?? null,
                rx: positions[i * 3 + 0],
                ry: positions[i * 3 + 1],
                rz: positions[i * 3 + 2],
                quadQuat: quat,
                selected: false,
            });
            this._writeRingInstance(i);
            this._imageRings.setColorAt(i, new THREE.Color(0x4cd9ff));
        }
        this._imageRings.instanceMatrix.needsUpdate = true;
        this._imageRings.instanceColor.needsUpdate = true;
        this.diag('image_poses_loaded', { n });
    }

    /** Position/scale of one ring marker. Selected poses get a bigger ring. */
    _writeRingInstance(index) {
        const meta = this._imagePoseMeta[index];
        const scale = meta.selected ? 1.8 : 1.0;
        this._imageRings.setMatrixAt(index, new THREE.Matrix4().compose(
            new THREE.Vector3(meta.rx, meta.ry, 0.02),
            new THREE.Quaternion(),
            new THREE.Vector3(scale, scale, scale),
        ));
    }

    /** Thumbnails arrive once and are kept as JPEG bytes; decoding is deferred
     *  to `_updateImageLod` so only nearby poses ever cost a texture. */
    addImageThumbnail(index, jpegArrayBuffer) {
        if (!this._imagePoseMeta[index]) return;
        this._thumbnailBytes.set(index, jpegArrayBuffer);
    }

    toggleImages() {
        this._imageQuadGroup.visible = !this._imageQuadGroup.visible;
        if (!this._imageQuadGroup.visible) this._releaseAllThumbnails();
        this._imageLodAccumS = IMAGE_LOD_INTERVAL_S;
        this.diag('images_toggle', { visible: this._imageQuadGroup.visible });
    }

    /** Keep decoded thumbnails to the nearest `IMAGE_QUAD_BUDGET` poses within
     *  `IMAGE_RENDER_DISTANCE_M`, measured in world metres so zooming out drops
     *  quads rather than piling up hundreds of textured, sorted transparents. */
    _updateImageLod(dt) {
        this._imageLodAccumS += dt;
        if (this._imageLodAccumS < IMAGE_LOD_INTERVAL_S) return;
        this._imageLodAccumS = 0;
        if (!this._imageQuadGroup.visible || this._imagePoseMeta.length === 0) return;

        const eye = this._imageQuadGroup.worldToLocal(this.camera.getWorldPosition(new THREE.Vector3()));
        const scale = this._worldGroup.scale.x || 1;
        // A query answer is a handful of poses anywhere in the recording, so the
        // distance cutoff would hide the very thing the user asked to see. The
        // budget alone is enough to bound the cost there.
        const maxDist = this._selectedImageIds.size > 0 ? Infinity : IMAGE_RENDER_DISTANCE_M / scale;

        const candidates = [];
        for (let i = 0; i < this._imagePoseMeta.length; i++) {
            const meta = this._imagePoseMeta[i];
            if (this._selectedImageIds.size > 0 && !this._selectedImageIds.has(meta.id)) continue;
            if (!this._thumbnailBytes.has(i)) continue;
            const dx = meta.rx - eye.x;
            const dy = meta.ry - eye.y;
            const dz = IMAGE_QUAD_HEIGHT - eye.z;
            const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (dist > maxDist) continue;
            candidates.push([dist, i]);
        }
        candidates.sort((a, b) => a[0] - b[0]);
        const wanted = new Set(candidates.slice(0, IMAGE_QUAD_BUDGET).map(([, i]) => i));

        for (const index of Array.from(this._imageQuadsByIndex.keys())) {
            if (!wanted.has(index)) this._releaseThumbnail(index);
        }
        for (const index of wanted) {
            if (!this._imageQuadsByIndex.has(index)) this._decodeThumbnail(index);
        }
    }

    _decodeThumbnail(index) {
        if (this._thumbnailDecoding.has(index)) return;
        this._thumbnailDecoding.add(index);
        // Three.js doesn't reliably apply flipY to ImageBitmap textures, so we
        // flip at decode time and disable the texture's own flip — otherwise the
        // photos render upside down.
        const blob = new Blob([this._thumbnailBytes.get(index)], { type: 'image/jpeg' });
        createImageBitmap(blob, { imageOrientation: 'flipY' }).then((bitmap) => {
            this._thumbnailDecoding.delete(index);
            const meta = this._imagePoseMeta[index];
            if (!meta || this._imageQuadsByIndex.has(index)) {
                bitmap.close();
                return;
            }
            const texture = new THREE.Texture(bitmap);
            texture.flipY = false;
            texture.colorSpace = THREE.SRGBColorSpace;
            texture.generateMipmaps = false;
            texture.minFilter = THREE.LinearFilter;
            texture.needsUpdate = true;
            const quad = new THREE.Mesh(
                this._imageQuadGeom,
                new THREE.MeshBasicMaterial({ map: texture, side: THREE.DoubleSide }),
            );
            quad.position.set(meta.rx, meta.ry, IMAGE_QUAD_HEIGHT);
            quad.quaternion.copy(meta.quadQuat);
            this._imageQuadGroup.add(quad);
            this._imageQuadsByIndex.set(index, quad);
        }).catch((e) => {
            this._thumbnailDecoding.delete(index);
            this.diag('thumbnail_decode_failed', { index, error: String(e.message || e) });
        });
    }

    _releaseThumbnail(index) {
        const quad = this._imageQuadsByIndex.get(index);
        if (!quad) return;
        this._imageQuadGroup.remove(quad);
        quad.material.map.dispose();
        quad.material.dispose();
        this._imageQuadsByIndex.delete(index);
    }

    _releaseAllThumbnails() {
        for (const index of Array.from(this._imageQuadsByIndex.keys())) this._releaseThumbnail(index);
    }

    setQueryResult(result) {
        this._clearHighlightGroup();

        for (const region of result.regions || []) {
            const points = region.points || [];
            if (points.length < 3) continue;
            const shape = new THREE.Shape();
            shape.moveTo(points[0][0], points[0][1]);
            for (let i = 1; i < points.length; i++) shape.lineTo(points[i][0], points[i][1]);
            shape.closePath();
            const fill = new THREE.Mesh(
                new THREE.ShapeGeometry(shape),
                new THREE.MeshBasicMaterial({
                    color: region.color || '#f9e547',
                    transparent: true,
                    opacity: region.opacity ?? 0.35,
                    depthWrite: false,
                    side: THREE.DoubleSide,
                }),
            );
            fill.position.z = points.reduce((sum, p) => sum + p[2], 0) / points.length + 0.06;
            this._highlightGroup.add(fill);

            const boundaryPoints = points.map((p) => new THREE.Vector3(p[0], p[1], p[2] + 0.075));
            this._highlightGroup.add(new THREE.LineLoop(
                new THREE.BufferGeometry().setFromPoints(boundaryPoints),
                new THREE.LineBasicMaterial({ color: region.color || '#f9e547' }),
            ));
        }

        for (const path of result.evidence_paths || []) {
            this._addHighlightTube(path, 0.035, path.color || '#ffd166');
        }
        if (result.route) {
            this._addHighlightTube(result.route, 0.065, result.route.color || '#64ff8f');
        }
        this._lastResultPoints = result.points || [];
        this._highlightVoxels(this._lastResultPoints);
        for (const point of result.points || []) {
            const marker = new THREE.Mesh(
                new THREE.SphereGeometry(0.13, 16, 12),
                new THREE.MeshBasicMaterial({ color: point.color || '#ff4d6d' }),
            );
            marker.position.set(...point.position);
            this._highlightGroup.add(marker);
        }
        if (result.focus_point) {
            const focus = new THREE.Mesh(
                new THREE.SphereGeometry(0.18, 20, 16),
                new THREE.MeshBasicMaterial({ color: 0xffffff }),
            );
            focus.position.set(...result.focus_point);
            this._highlightGroup.add(focus);
        }

        this._selectedImageIds = new Set(result.observation_ids || []);
        for (let i = 0; i < this._imagePoseMeta.length; i++) {
            const meta = this._imagePoseMeta[i];
            meta.selected = this._selectedImageIds.has(meta.id);
            this._writeRingInstance(i);
            this._imageRings.setColorAt(i, new THREE.Color(meta.selected ? 0xfff06a : 0x4cd9ff));
        }
        if (this._imageRings) {
            this._imageRings.instanceMatrix.needsUpdate = true;
            this._imageRings.instanceColor.needsUpdate = true;
        }
        if (this._selectedImageIds.size > 0) this._imageQuadGroup.visible = true;
        // The selection changes which poses deserve a texture, so rebuild now.
        this._releaseAllThumbnails();
        this._imageLodAccumS = IMAGE_LOD_INTERVAL_S;

        this._activeQueryId = result.query_id || null;
        this._queryImages = [];
        this._queryImageMeshes = [];
        this._queryImageCursor = -1;
        this._setAnswer(result.answer || 'Memory result');
        this.diag('query_result_loaded', {
            query_id: result.query_id,
            revision: result.revision,
            regions: (result.regions || []).length,
            evidence_paths: (result.evidence_paths || []).length,
            route: Boolean(result.route),
        });
    }

    /** Hang the frame behind an answer on its camera's image plane: a quad
     *  `distance_m` in front of where the camera stood, sized by its field of
     *  view, with thin lines back to the camera so the frustum reads. */
    addQueryImage(header, jpegArrayBuffer) {
        if (header.query_id !== this._activeQueryId) return;
        const blob = new Blob([jpegArrayBuffer], { type: 'image/jpeg' });
        createImageBitmap(blob, { imageOrientation: 'flipY' }).then((bitmap) => {
            if (header.query_id !== this._activeQueryId) { bitmap.close(); return; }
            const texture = new THREE.Texture(bitmap);
            texture.flipY = false;
            texture.colorSpace = THREE.SRGBColorSpace;
            texture.generateMipmaps = false;
            texture.minFilter = THREE.LinearFilter;
            texture.needsUpdate = true;

            const eye = new THREE.Vector3(...header.position);
            const forward = new THREE.Vector3(...header.forward).normalize();
            const up = new THREE.Vector3(...header.up).normalize();
            const distance = header.distance_m || 1.0;
            const width = 2 * distance * Math.tan(THREE.MathUtils.degToRad(header.hfov_deg || 70) / 2);
            const height = width / (header.aspect || 16 / 9);

            const quad = new THREE.Mesh(
                new THREE.PlaneGeometry(width, height),
                new THREE.MeshBasicMaterial({ map: texture, side: THREE.DoubleSide }),
            );
            quad.position.copy(eye).addScaledVector(forward, distance);
            // lookAt works in world space; the group is a child of the frame rotation.
            this._frameRotate.updateWorldMatrix(true, false);
            quad.up.copy(up).transformDirection(this._frameRotate.matrixWorld);
            this._highlightGroup.add(quad);
            quad.lookAt(this._frameRotate.localToWorld(eye.clone()));
            quad.rotateY(Math.PI); // lookAt aims +z at the eye; the picture faces the other way

            const right = new THREE.Vector3().crossVectors(forward, up).normalize();
            const centre = quad.position.clone();
            const corners = [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([sx, sy]) =>
                centre.clone().addScaledVector(right, sx * width / 2).addScaledVector(up, sy * height / 2));
            const segments = [];
            for (const corner of corners) segments.push(eye.clone(), corner);
            for (let i = 0; i < 4; i++) segments.push(corners[i], corners[(i + 1) % 4]);
            this._highlightGroup.add(new THREE.LineSegments(
                new THREE.BufferGeometry().setFromPoints(segments),
                new THREE.LineBasicMaterial({ color: header.index === 0 ? 0xff5c3a : 0xffb347 }),
            ));
            this._queryImages[header.index] = header;
            this._queryImageMeshes[header.index] = quad;
            // Frames from nearby poses overlap; while standing at one camera, only its frame shows.
            if (this._queryImageCursor >= 0) quad.visible = header.index === this._queryImageCursor;
            this.diag('query_image_placed', { index: header.index, width: Number(width.toFixed(2)) });
        }).catch((e) => {
            this.diag('query_image_failed', { index: header.index, error: String(e.message || e) });
        });
    }

    /** Stand where the camera behind answer *index* stood and look the way it
     *  looked, so the photo lines up with the voxels it was taken from.
     *  Desktop only: in VR the head is the camera. */
    viewFrom(index) {
        const header = this._queryImages[index];
        if (!header || this.three.xr.isPresenting) return false;
        const eye = new THREE.Vector3(...header.position);
        const forward = new THREE.Vector3(...header.forward).normalize();
        // Robot -> three: (x, y, z) -> (x, z, -y), then the world group's scale.
        const scale = this._worldGroup.scale.x;
        const eyeThree = new THREE.Vector3(eye.x, eye.z, -eye.y).multiplyScalar(scale);
        const fwdThree = new THREE.Vector3(forward.x, forward.z, -forward.y);
        const head = this.getCameraPositionWorld();
        this._worldGroup.position.set(head.x - eyeThree.x, head.y - eyeThree.y, head.z - eyeThree.z);
        // The desktop camera looks down -z at yaw 0; pitch is positive looking up.
        this._desktopYaw = Math.atan2(-fwdThree.x, -fwdThree.z);
        this._desktopPitch = Math.asin(Math.max(-1, Math.min(1, fwdThree.y)));
        this.camera.rotation.set(this._desktopPitch, this._desktopYaw, 0);
        this._queryImageCursor = index;
        this._queryImageMeshes.forEach((mesh, i) => { if (mesh) mesh.visible = i === index; });
        this.diag('view_from', { index });
        return true;
    }

    _addHighlightTube(path, radius, color) {
        const points = (path.points || []).map((p) => new THREE.Vector3(p[0], p[1], p[2]));
        if (points.length < 2) return;
        const curve = new THREE.CatmullRomCurve3(points, false, 'centripetal');
        const geometry = new THREE.TubeGeometry(curve, Math.max(16, points.length * 3), radius, 8, false);
        const material = new THREE.MeshBasicMaterial({ color });
        this._highlightGroup.add(new THREE.Mesh(geometry, material));
    }

    _clearHighlightGroup() {
        while (this._highlightGroup.children.length) {
            const child = this._highlightGroup.children.pop();
            child.traverse((obj) => {
                if (obj.geometry) obj.geometry.dispose();
                if (obj.material) obj.material.dispose();
            });
        }
    }

    /** Show what the microphone heard, so the headset confirms before the answer. */
    setHeardText(text) {
        this._setAnswer(`“${text}” …`);
    }

    _setAnswer(answer) {
        const ctx = this._answerCanvas.getContext('2d');
        ctx.clearRect(0, 0, this._answerCanvas.width, this._answerCanvas.height);
        ctx.fillStyle = 'rgba(5, 10, 16, 0.92)';
        ctx.fillRect(0, 0, this._answerCanvas.width, this._answerCanvas.height);
        ctx.strokeStyle = '#7af0a8';
        ctx.lineWidth = 8;
        ctx.strokeRect(4, 4, this._answerCanvas.width - 8, this._answerCanvas.height - 8);
        ctx.fillStyle = '#d8e6f4';
        ctx.font = '42px monospace';
        const words = String(answer).split(/\s+/);
        const lines = [];
        let line = '';
        for (const word of words) {
            const candidate = line ? `${line} ${word}` : word;
            if (ctx.measureText(candidate).width > 930 && line) {
                lines.push(line);
                line = word;
            } else {
                line = candidate;
            }
            if (lines.length === 3) break;
        }
        if (line && lines.length < 4) lines.push(line);
        lines.slice(0, 4).forEach((text, i) => ctx.fillText(text, 42, 62 + i * 50));
        this._answerTexture.needsUpdate = true;
        this._answerPanel.visible = true;
    }

    setTopDownMap(header, jpegArrayBuffer) {
        const blob = new Blob([jpegArrayBuffer], { type: 'image/jpeg' });
        createImageBitmap(blob).then((bitmap) => {
            const tex = new THREE.Texture(bitmap);
            tex.colorSpace = THREE.SRGBColorSpace;
            tex.needsUpdate = true;
            this._topDownTex = tex;
            this._topDownBounds = header;

            // 1) Ground projection in robot frame.
            const w = header.x_max - header.x_min;
            const h = header.y_max - header.y_min;
            const cx = (header.x_min + header.x_max) / 2;
            const cy = (header.y_min + header.y_max) / 2;
            const planeGeom = new THREE.PlaneGeometry(w, h);
            const planeMat = new THREE.MeshBasicMaterial({
                map: tex,
                transparent: true,
                opacity: this.backgroundMode === 'passthrough' ? 1.0 : 0.85,
                side: THREE.DoubleSide,
            });
            if (this._groundMesh) {
                this._frameRotate.remove(this._groundMesh);
                this._groundMesh.geometry.dispose();
                this._groundMesh.material.dispose();
            }
            this._groundMesh = new THREE.Mesh(planeGeom, planeMat);
            // The histogram image is built with robot +X as horizontal and
            // +Y up after a transpose+flipud. Lay it on the floor (z=0) in
            // robot frame centred on (cx, cy). PlaneGeometry's +Y axis is up
            // in its local space; for a floor-prone plane in robot Z-up we
            // keep it in the XY plane — which is exactly what PlaneGeometry
            // gives us once the frameRotate group flips back to Y-up later.
            this._groundMesh.position.set(cx, cy, 0.01);
            // Default plane lies in XY of its parent. Robot frame is what we
            // want, no extra rotation needed. But the texture's row 0 is at
            // y_max (since we did flipud), so flip Y to align UV.
            this._groundMesh.material.map.repeat.y = -1;
            this._groundMesh.material.map.offset.y = 1;
            this._frameRotate.add(this._groundMesh);

            // 2) HUD panel uses the same texture, also with V flipped.
            const hudTex = tex.clone();
            hudTex.needsUpdate = true;
            hudTex.colorSpace = THREE.SRGBColorSpace;
            hudTex.repeat.y = -1;
            hudTex.offset.y = 1;
            this._hudPanelMat.color.set(0xffffff);
            this._hudPanelMat.map = hudTex;
            this._hudPanelMat.opacity = 0.95;
            this._hudPanelMat.needsUpdate = true;

            this.diag('top_down_map_loaded', { w, h });
        }).catch((e) => {
            this.diag('top_down_decode_failed', { error: String(e.message || e) });
        });
    }

    setOdomTrail(header, payloadArrayBuffer) {
        const n = header.n | 0;
        if (n < 2) return;
        const positions = new Float32Array(payloadArrayBuffer, 0, n * 3);

        if (this._odomLine) {
            this._frameRotate.remove(this._odomLine);
            this._odomLine.geometry.dispose();
            this._odomLine.material.dispose();
        }
        const geom = new THREE.BufferGeometry();
        // Lift slightly so it doesn't z-fight with floor.
        const lifted = new Float32Array(n * 3);
        for (let i = 0; i < n; i++) {
            lifted[i * 3 + 0] = positions[i * 3 + 0];
            lifted[i * 3 + 1] = positions[i * 3 + 1];
            lifted[i * 3 + 2] = (positions[i * 3 + 2] || 0) + 0.03;
        }
        geom.setAttribute('position', new THREE.BufferAttribute(lifted, 3));
        const mat = new THREE.LineBasicMaterial({ color: 0xff9944, transparent: true, opacity: 0.85 });
        this._odomLine = new THREE.Line(geom, mat);
        this._frameRotate.add(this._odomLine);
        this.diag('odom_trail_loaded', { n });
    }

    _spawnAtCentroid() {
        if (!this._cloudBounds) return;
        const b = this._cloudBounds;
        const cx = (b.x_min + b.x_max) / 2;
        const cy = (b.y_min + b.y_max) / 2;
        // Robot (cx, cy, 0) -> three (cx, 0, -cy) after frame rotate.
        // We want the camera to be near robot origin instead of inside a wall:
        // place worldGroup such that the centroid sits a few metres in front.
        const head = this.getCameraPositionWorld();
        const fwd = this.getCameraForwardXZ();
        const target = new THREE.Vector3(
            head.x + fwd[0] * 1.5,
            0,
            head.z + fwd[1] * 1.5,
        );
        // After frame rotate the centroid is at three (cx, 0, -cy). Translate
        // the world so that point lands at `target`.
        this._worldGroup.position.x = target.x - cx;
        this._worldGroup.position.z = target.z - (-cy);
        this._hasSpawned = true;
        this.diag('spawned', { cx, cy });
    }
}
