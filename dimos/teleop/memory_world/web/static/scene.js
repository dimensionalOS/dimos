// Three.js scene — first-person walkthrough of a recorded point cloud.
//
// World data hangs under a frame-rotate group: -90° about X, (rx,ry,rz) -> (rx,rz,-ry).
//
// Locomotion moves `_worldGroup`, not the camera (WebXR drives that).

import * as THREE from 'https://esm.sh/three@0.160.0';
import { DESKTOP_PITCH_LIMIT, installTouch } from './touch.js';
import { SPRITE_FRAGMENT_SHADER, SPRITE_VERTEX_GLSL, spriteUniforms, viewportHeight, viewportHeightPx } from '/static_mw/voxel_sprites.js';
import { ANSWER_PANEL_W, HUD_PANEL_SIZE, placeHud } from '/static_mw/hud.js';
import { addQueryImage, sightLineFor } from '/static_mw/evidence.js';
import { OrbitControl } from '/static_mw/orbit.js';

const WALK_SPEED_M_PER_S = 1.4;               // headset-relative
const WALK_EASE_S = 0.16;                     // velocity time constant: start/stop ramps
const POINT_SIZE = 0.025;                     // metres
const TELEPORT_ARC_SEGMENTS = 24;
const TELEPORT_MAX_DISTANCE = 8.0;            // metres along ray
const MIN_SCALE = 0.05;
const MAX_SCALE = 10.0;
const EYE_HEIGHT_M = 1.6;
const DESKTOP_LOOK_SENSITIVITY = 0.0022;      // radians per pixel of mouse travel
const DESKTOP_SPRINT_MULTIPLIER = 3.0;
const DESKTOP_MOVE_KEYS = new Set(['KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyQ', 'KeyE']);
const HUD_MARKER_RADIUS = 0.008;
const ANSWER_PANEL_H = 0.155;
const CAMERA_PANEL_W = 0.40;          // replay camera frame, 16:9, above the answer
const CAMERA_FRUSTUM_M = 0.5;         // how far the drawn frustum reaches from the camera
const IMAGE_QUAD_W = 0.60;
const IMAGE_QUAD_H = 0.34;            // 16:9-ish
// Photo markers hang at the height the camera actually was; a recording whose odom
// starts metres off the floor would otherwise float them all in the air.
// Thumbnails decode only near the viewer, and only this many at once.
const IMAGE_RENDER_DISTANCE_M = 12.0;
const IMAGE_QUAD_BUDGET = 24;
const IMAGE_LOD_INTERVAL_S = 0.2;     // how often the visible set is recomputed
const PERF_WINDOW = 240;
// Quality steps down when the median frame is slow, back up after a settled spell.
const QUALITY_LEVELS = [
    { voxel_fraction: 1.0, voxel_range_m: Infinity, quad_budget: 24, foveation: 0.0, resolution: 1.0 },
    { voxel_fraction: 0.75, voxel_range_m: 20, quad_budget: 16, foveation: 0.4, resolution: 0.9 },
    { voxel_fraction: 0.5, voxel_range_m: 14, quad_budget: 8, foveation: 0.7, resolution: 0.8 },
    { voxel_fraction: 0.35, voxel_range_m: 10, quad_budget: 4, foveation: 1.0, resolution: 0.7 },
    { voxel_fraction: 0.2, voxel_range_m: 7, quad_budget: 0, foveation: 1.0, resolution: 0.6 },
];
const QUALITY_STEP_DOWN_MS = 1000 / 30;
const QUALITY_STEP_UP_MS = 1000 / 45;
const QUALITY_INTERVAL_S = 1.0;
const QUALITY_SETTLE_S = 3.0;
const QUALITY_SAMPLE_FRAMES = 90;
// How often the drawn voxel set is recomputed around the viewer, and how far
// the viewer must move before it is worth doing.
const VOXEL_CULL_INTERVAL_S = 0.5;
const VOXEL_CULL_MOVE_M = 1.0;
// Answer voxels repaint warm; the map stays navy-to-cyan.
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
        this._orbit = new OrbitControl();

        // Origin grid in robot frame for visual reference (10m, 1m cells).
        const grid = new THREE.GridHelper(20, 20, 0x1f2a3a, 0x1f2a3a);
        grid.rotation.x = Math.PI / 2;             // grid is XZ in three; we want XY in robot
        this._frameRotate.add(grid);

        // Containers we (re)populate on payload receive.
        this._pointsObj = null;               // THREE.Points of sphere sprites
        this._cloudWanted = true;             // the Voxel map box; the replay hides it meanwhile
        this._replayActive = false;
        this._voxelsDrawn = 0;
        this._cloudData = null;               // {n, positions, colors, voxelSize}
        this._roofCut = { value: 1e9 };       // voxels above this robot z are not drawn
        // A corridor from the eye to a photograph, carved out of the map while that photo
        // is being looked at: standing where the picture was taken puts you inside the
        // wall the robot was facing, and the picture is behind it. Radius <= 0 is off.
        this._sightFrom = { value: new THREE.Vector3() };
        this._sightTo = { value: new THREE.Vector3() };
        this._sightRadius = { value: 0 };
        this._sightLine = null;                       // which cut, once there is one
        this._imageQuadGroup = new THREE.Group();     // textured quads, toggleable
        this._imageQuadGroup.visible = false;
        this._frameRotate.add(this._imageQuadGroup);
        this._imagePoseMeta = [];                     // per-index {pos, quat}
        this._imageQuadsByIndex = new Map();          // index -> THREE.Mesh
        this._thumbnailBytes = new Map();             // index -> ArrayBuffer, decoded on demand
        this._thumbnailDecoding = new Set();
        this._imageQuadGeom = new THREE.PlaneGeometry(IMAGE_QUAD_W, IMAGE_QUAD_H);
        this._imageLodAccumS = IMAGE_LOD_INTERVAL_S;
        this._quality = 0;                            // index into QUALITY_LEVELS
        this._qualityAuto = true;
        this._qualityAccumS = 0;
        this._qualitySettleS = 0;
        this._voxelCullAccumS = 0;
        this._voxelCullEye = null;                    // robot-frame eye at the last compaction
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
        this._queryMatchMarks = [];                   // the ring and link belonging to each
        this._photosPinnedOff = false;                // set when the user turns Photos off
        this._hudOff = false;                         // likewise for the minimap and answer panel
        this._queryImageCursor = -1;
        this.onOrbitChange = null;   // set by main.js; see setOrbit

        // Top-down map: shared texture, used twice (ground projection + HUD).
        this._topDownBounds = null;

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
        // Off by default: it covers the view and most of the time you want the
        // world, not a map of it. M or the button brings it back.
        this._hudPanel.visible = false;
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

        // Replay: the camera frame at the scrubbed time, head-locked above the
        // answer, and a frustum in the world where that frame was taken.
        this._cameraPanel = new THREE.Mesh(
            new THREE.PlaneGeometry(CAMERA_PANEL_W, CAMERA_PANEL_W * 9 / 16),
            new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false, side: THREE.DoubleSide }),
        );
        this._cameraPanel.position.set(0, 0.2 + ANSWER_PANEL_H / 2 + CAMERA_PANEL_W * 9 / 32 + 0.02, 0.002);
        this._cameraPanel.visible = false;
        this._hudGroup.add(this._cameraPanel);
        // Accent green so it reads against the orange trail and the blue map.
        this._cameraFrustum = new THREE.LineSegments(
            new THREE.BufferGeometry().setFromPoints(new Array(16).fill(new THREE.Vector3())),
            new THREE.LineBasicMaterial({ color: 0x7af0a8 }),
        );
        this._cameraFrustum.visible = false;
        this._cameraFrustum.frustumCulled = false;
        this._cameraFrustum.add(new THREE.Mesh(
            new THREE.SphereGeometry(0.06, 12, 8),
            new THREE.MeshBasicMaterial({ color: 0x7af0a8 }),
        ));
        this._frameRotate.add(this._cameraFrustum);
        this._replayGroup = new THREE.Group();
        this._replayGroup.visible = false;
        this._frameRotate.add(this._replayGroup);
        this._replayLayer = null;
        this._replayHfov = 70;
        this.onTick = null;

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
            quality: this._quality,
            quality_auto: this._qualityAuto,
            replay_voxels: this._replayLayer && this._replayGroup.visible ? this._replayLayer.drawn : 0,
            voxels_drawn: this._voxelsDrawn,
            voxels_total: this._cloudData ? this._cloudData.n : 0,
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

    /** Stop rendering, free the GPU buffers and drop the canvas, so a reconnect
     *  doesn't stack a second one. */
    dispose() {
        this.three.setAnimationLoop(null);
        this.scene.traverse((obj) => {
            if (obj.geometry) obj.geometry.dispose();
            for (const material of [].concat(obj.material || [])) {
                if (material.map) { material.map.image?.close?.(); material.map.dispose(); }
                material.dispose();
            }
        });
        this._pageListeners?.abort();  // the window/document handlers of startDesktop
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
        // Canvas handlers die with the canvas; these outlive it unless aborted.
        this._pageListeners = new AbortController();
        const signal = this._pageListeners.signal;
        window.addEventListener('mouseup', () => {
            this._desktopDragging = false;
            if (document.pointerLockElement !== dom) dom.style.cursor = 'grab';
        }, { signal });
        dom.addEventListener('dblclick', () => {
            if (document.pointerLockElement !== dom) dom.requestPointerLock();
        });
        dom.addEventListener('wheel', (event) => {
            // Only while orbiting: the wheel moves the eye in and out. Walking around,
            // it used to scale the world, which reads as the speed changing under you.
            if (!this._orbit.active) return;
            event.preventDefault();
            this._orbit.zoom(event.deltaY, this);
        }, { passive: false });

        document.addEventListener('pointerlockchange', () => {
            const locked = document.pointerLockElement === dom;
            dom.style.cursor = locked ? 'none' : 'grab';
            if (!locked) this._desktopKeys.clear();
        }, { signal });
        document.addEventListener('mousemove', (event) => {
            if (document.pointerLockElement !== dom && !this._desktopDragging) return;
            this._desktopYaw -= event.movementX * DESKTOP_LOOK_SENSITIVITY;
            this._desktopPitch -= event.movementY * DESKTOP_LOOK_SENSITIVITY;
            this._desktopPitch = Math.max(-DESKTOP_PITCH_LIMIT, Math.min(DESKTOP_PITCH_LIMIT, this._desktopPitch));
            this.camera.rotation.set(this._desktopPitch, this._desktopYaw, 0);
        }, { signal });
        window.addEventListener('keydown', (event) => this._onDesktopKey(event, true), { signal });
        window.addEventListener('keyup', (event) => this._onDesktopKey(event, false), { signal });
        window.addEventListener('resize', () => this._resizeDesktopCamera(), { signal });
        installTouch(this, dom);

        this.three.setAnimationLoop((time) => {
            this._applyDesktopKeys();
            this._orbit.apply(this);
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
        const el = event.target;
        if (el && (el.tagName === 'INPUT' || el.tagName === 'SELECT' || el.tagName === 'TEXTAREA' || el.isContentEditable)) {
            this._desktopKeys.clear();  // typing: the keys are the box's, not the world's
            return;
        }
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
        if (event.code === 'KeyJ' && this.onJump) this.onJump();
        if (event.code === 'KeyM') this.toggleHud();
        if (event.code === 'KeyO') this.setOrbit(!this._orbit.active);
        if (event.code === 'KeyP') this.stepQueryImage(event.shiftKey ? -1 : 1);
        else if (event.code === 'KeyI') this.toggleImages();
        else if (event.code === 'KeyV') this.toggleCloud();
    }

    _applyDesktopKeys() {
        if (this._orbit.active) return;  // the eye is pinned to the orbit; walking would fight it
        const keys = this._desktopKeys;
        // _walk() scales by stick magnitude, so sprint is just a bigger deflection.
        const gain = this._desktopSprint ? DESKTOP_SPRINT_MULTIPLIER : 1;
        const drag = this._touchStick || { x: 0, y: 0 };
        const stick = this._stickInput || { x: 0, y: 0 };
        this.applyLocomote({
            stickX: ((keys.has('KeyD') ? 1 : 0) - (keys.has('KeyA') ? 1 : 0)) * gain + drag.x + stick.x,
            stickY: ((keys.has('KeyS') ? 1 : 0) - (keys.has('KeyW') ? 1 : 0)) * gain + drag.y + stick.y,
            up: ((keys.has('KeyE') ? 1 : 0) - (keys.has('KeyQ') ? 1 : 0)) * gain,
        });
    }

    /** Walk input from the on-screen stick: x right, y backward, each in [-1, 1].
     *  Kept apart from the two-finger drag so a finger lifting off the canvas
     *  does not stop the stick. */
    setTouchStick(x, y) {
        this._stickInput = {
            x: Math.max(-1, Math.min(1, x || 0)),
            y: Math.max(-1, Math.min(1, y || 0)),
        };
    }

    /** Hide or show the head-locked HUD: minimap and answer text together. */
    // ---- orbit: circle a frame of the robot instead of walking (orbit.js) ------

    /** Where the orbited frame is now, robot coords; the eye follows it while orbiting. */
    setOrbitTarget(position) {
        this._orbit.setTarget(position, this);
    }

    /** Turn orbit mode on or off (desktop only). Returns the new state. */
    setOrbit(enabled) {
        if (this.three.xr.isPresenting) return false;
        if (enabled) this._orbit.enable(this);
        else this._orbit.disable();
        this.diag('orbit', { on: this._orbit.active, target: this._orbit.target });
        // Whoever changed it, the buttons have to follow: flying to a place turns orbit
        // off through here, and without this the toolbar kept saying "Stop orbit" while
        // orbit was already off -- so pressing the button that said "Stop orbit" started
        // orbiting. Every path into orbit goes through this method, including the O key.
        if (this.onOrbitChange) this.onOrbitChange(this._orbit.active);
        return this._orbit.active;
    }

    isOrbiting() {
        return this._orbit.active;
    }

    toggleHud() {
        this._hudPanel.visible = !this._hudPanel.visible;
        this._hudOff = !this._hudPanel.visible;  // the user's own choice; an answer respects it
        // The head-locked group carries the answer panel, and the tour pins it off so an
        // answer cannot reappear beside an unrelated station. M toggles the user's panel,
        // but it does not get to undo that pin -- `_setAnswer` already honours it, and
        // this was the one way back in: two presses during a tour put a stale answer on
        // screen next to a card describing something else.
        this._hudGroup.visible = this._hudPanel.visible && !this._hudGroupPinnedOff;
        this.diag('hud_toggle', { visible: this._hudPanel.visible });
        if (this.onLayerChange) this.onLayerChange();
        return this._hudPanel.visible;
    }

    /** Phone controls: one finger looks, two fingers walk (drag) and scale (pinch). */
    _tick(timeMs) {
        viewportHeight.value = viewportHeightPx(this.three);
        const frameMs = this._lastTickMs ? timeMs - this._lastTickMs : 0;
        if (frameMs > 0) {
            this._frameSamples[this._frameSampleCursor] = frameMs;
            this._frameSampleCursor = (this._frameSampleCursor + 1) % PERF_WINDOW;
            this._frameSampleCount = Math.min(this._frameSampleCount + 1, PERF_WINDOW);
        }
        const dt = this._lastTickMs ? Math.max((timeMs - this._lastTickMs) / 1000, 0) : 0;
        this._lastTickMs = timeMs;
        this._updateQuality(dt);
        this._updateVoxelCull(dt);

        // Walking eases in and out: the stick sets a target, the velocity follows it.
        const loc = this._pendingLocomote || { stickX: 0, stickY: 0, up: 0 };
        const dead = (v) => (Math.abs(v) > 0.1 ? v : 0);
        const v = this._walkVelocity || (this._walkVelocity = { x: 0, y: 0, up: 0 });
        if (dt > 0) {
            const k = 1 - Math.exp(-dt / WALK_EASE_S);
            v.x += (dead(loc.stickX) - v.x) * k;
            v.y += (dead(loc.stickY) - v.y) * k;
            v.up += (dead(loc.up) - v.up) * k;
            if (Math.abs(v.x) + Math.abs(v.y) + Math.abs(v.up) > 0.01) this._walk(v.x, v.y, v.up, dt);
        }

        if (dt > 0 && Math.abs(this._pendingYawRate) > 1e-3) {
            const pivot = this.getCameraPositionWorld();
            this._rotateWorldAround(pivot, this._pendingYawRate * dt);
        }

        this._updateImageLod(dt);
        placeHud(this);
        if (this.onTick) this.onTick(dt);
    }

    // ---- timeline replay ---------------------------------------------------

    /** Hang the replay voxel layer in the world; it draws instead of the static map while active. */
    attachReplay(layer, hfovDeg) {
        if (this._replayLayer) this._replayGroup.remove(this._replayLayer);
        this._replayLayer = layer;
        this._replayHfov = hfovDeg || 70;
        layer.fraction = QUALITY_LEVELS[this._quality].voxel_fraction;
        this._replayGroup.add(layer);
    }

    setReplayActive(active) {
        this._replayGroup.visible = active;
        this._replayActive = active;
        if (this._pointsObj) this._pointsObj.visible = this._cloudWanted && !active;
        this._cameraPanel.visible = active && Boolean(this._cameraPanel.material.map);
        this._cameraFrustum.visible = active && this._cameraFrustum.userData.posed === true;
    }

    /** Show a camera frame on the HUD and draw its frustum where it was taken. */
    setCameraFrame(bitmap, meta) {
        const material = this._cameraPanel.material;
        if (material.map) material.map.dispose();
        const texture = new THREE.Texture(bitmap);
        texture.flipY = false;
        texture.colorSpace = THREE.SRGBColorSpace;
        texture.generateMipmaps = false;
        texture.minFilter = THREE.LinearFilter;
        texture.needsUpdate = true;
        material.map = texture;
        material.needsUpdate = true;
        this._cameraPanel.visible = this._replayGroup.visible;
        if (meta && meta.position && meta.forward && meta.up) {
            const eye = new THREE.Vector3(...meta.position);
            const forward = new THREE.Vector3(...meta.forward).normalize();
            const up = new THREE.Vector3(...meta.up).normalize();
            const right = new THREE.Vector3().crossVectors(forward, up).normalize();
            const width = 2 * CAMERA_FRUSTUM_M * Math.tan(THREE.MathUtils.degToRad(meta.hfov_deg || this._replayHfov) / 2);
            const height = width * bitmap.height / bitmap.width;
            const centre = eye.clone().addScaledVector(forward, CAMERA_FRUSTUM_M);
            const corners = [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([sx, sy]) =>
                centre.clone().addScaledVector(right, sx * width / 2).addScaledVector(up, sy * height / 2));
            const points = [];
            for (const corner of corners) points.push(eye, corner);
            for (let i = 0; i < 4; i++) points.push(corners[i], corners[(i + 1) % 4]);
            this._cameraFrustum.geometry.setFromPoints(points);
            this._cameraFrustum.children[0].position.copy(eye);
            this._cameraFrustum.userData.posed = true;
            this._cameraFrustum.visible = this._replayGroup.visible;
        }
    }

    /** Show no photograph at all: the scrubber is at a moment the camera did not cover.
     *
     *  FRAME_TOLERANCE_S in replay.js says "no camera frame closer than this: show none",
     *  and leaving the previous one up instead shows a picture of somewhere else, posed
     *  where that other place was, against the voxels of where you actually are.
     */
    clearCameraFrame() {
        const material = this._cameraPanel.material;
        if (material.map) {
            material.map.dispose();
            material.map = null;
            material.needsUpdate = true;
        }
        this._cameraPanel.visible = false;
        this._cameraFrustum.visible = false;
        this._cameraFrustum.userData.posed = false;
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

    _walk(stickX, stickY, up, dt) {
        // Quest left stick: forward push = stickY < 0, right push = stickX > 0.
        // up > 0 lifts the viewer (desktop Q/E only).
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
        this._worldGroup.position.y -= up * speed * dt;
    }

    // ---- public locomotion API -------------------------------------------

    applyLocomote(g) {
        this._pendingLocomote = { stickX: g.stickX || 0, stickY: g.stickY || 0, up: g.up || 0 };
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

        // A straight ray stopped at the floor plane (y = 0, the local-floor origin).
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
        this._stopOrbitBeforeMoving();
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
        // The z was a hard-coded 0 from the first commit of this package onwards, and
        // the server spent three attempts trying to work out a height without one. Three
        // y IS the robot's z (the frame-rotate is R_x(-pi/2)) and the world group's yaw
        // does not touch it, so the height costs one subtraction and a divide.
        const world = this.getCameraPositionWorld();
        const xy = this._worldPosToRobotXY(world);
        const wg = this._worldGroup;
        return [xy[0], xy[1], (world.y - wg.position.y) / (wg.scale.x || 1)];
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

        // A random visiting order, fixed per cloud: the first k entries are a
        // uniform sample, so a quality level draws voxels[order[0..k]] and
        // thins the map evenly instead of dropping one end of it.
        if (!d.order || d.order.length !== d.n) {
            d.order = new Uint32Array(d.n);
            for (let i = 0; i < d.n; i++) d.order[i] = i;
            for (let i = d.n - 1; i > 0; i--) {
                const j = Math.floor(Math.random() * (i + 1));
                const swap = d.order[i]; d.order[i] = d.order[j]; d.order[j] = swap;
            }
        }
        // What each voxel currently shows: its height colour, or a highlight.
        if (!d.paint) {
            d.paint = new Float32Array(d.n * 3);
            if (d.colors) d.paint.set(d.colors); else d.paint.fill(1);
        }

        // One sphere sprite per voxel: a vertex each, lit in the fragment
        // shader so depth cues survive against passthrough.
        const geometry = new THREE.BufferGeometry();
        const positions = new THREE.BufferAttribute(new Float32Array(d.n * 3), 3);
        const colors = new THREE.BufferAttribute(new Float32Array(d.n * 3), 3);
        positions.setUsage(THREE.DynamicDrawUsage);
        colors.setUsage(THREE.DynamicDrawUsage);
        geometry.setAttribute('position', positions);
        geometry.setAttribute('color', colors);
        geometry.setDrawRange(0, 0);
        const material = new THREE.ShaderMaterial({
            uniforms: {
                ...spriteUniforms(d.voxelSize),
                roofCut: this._roofCut,
                sightFrom: this._sightFrom,
                sightTo: this._sightTo,
                sightRadius: this._sightRadius,
            },
            vertexColors: true,
            vertexShader: `${SPRITE_VERTEX_GLSL}
                uniform float roofCut;
                uniform vec3 sightFrom;
                uniform vec3 sightTo;
                uniform float sightRadius;
                varying vec3 vColor;
                // Inside the cone from the eye to the photograph: a narrow hole at the
                // face opening out to the picture's own width, so what is carved away is
                // what would have been in front of it and nothing else.
                bool inSightLine(vec3 p) {
                    if (sightRadius <= 0.0) return false;
                    vec3 axis = sightTo - sightFrom;
                    float len2 = dot(axis, axis);
                    if (len2 <= 0.0) return false;
                    float t = dot(p - sightFrom, axis) / len2;
                    // Strictly between the eye and the picture. Clamping instead would
                    // measure anything past the far end from that end, carving a sphere
                    // out of whatever stands BEHIND the photograph -- which is the part
                    // of the map you came here to look at.
                    if (t < 0.0 || t > 1.0) return false;
                    float radius = mix(0.15, sightRadius, t);
                    return distance(p, sightFrom + axis * t) < radius;
                }
                void main() {
                    vColor = color;
                    vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
                    // Above the roof cut (robot z) the voxel vanishes: the tour looks in from above.
                    bool gone = position.z > roofCut || inSightLine(position);
                    gl_PointSize = gone ? 0.0 : spritePointSize(mvPosition);
                    gl_Position = gone ? vec4(2.0, 2.0, 2.0, 1.0) : projectionMatrix * mvPosition;
                }`,
            fragmentShader: SPRITE_FRAGMENT_SHADER,
        });
        const points = new THREE.Points(geometry, material);
        points.frustumCulled = false; // the bounding sphere would be recomputed on every compaction
        this._pointsObj = points;
        points.visible = this._cloudWanted && !this._replayActive;
        this._frameRotate.add(this._pointsObj);
        this._highlightedVoxels = [];
        this._highlightVoxels(this._lastResultPoints);
    }

    /** Write the voxels the current quality level draws into the sprite
     *  buffers: a uniform `voxel_fraction` of them, within `voxel_range_m` of
     *  the viewer. Everything else is simply not drawn. */
    _compactCloud() {
        const d = this._cloudData;
        const points = this._pointsObj;
        if (!d || !points) return;
        const level = QUALITY_LEVELS[this._quality];
        const budget = Math.max(1, Math.round(d.n * level.voxel_fraction));
        const eye = this._robotFrameEye();
        const scale = this._worldGroup.scale.x || 1;
        const range2 = Number.isFinite(level.voxel_range_m)
            ? (level.voxel_range_m / scale) * (level.voxel_range_m / scale)
            : Infinity;
        const positionAttr = points.geometry.getAttribute('position');
        const colorAttr = points.geometry.getAttribute('color');
        const positions = positionAttr.array;
        const colours = colorAttr.array;
        let written = 0;
        for (let k = 0; k < budget; k++) {
            const i = d.order[k];
            const x = d.positions[i * 3], y = d.positions[i * 3 + 1], z = d.positions[i * 3 + 2];
            if (range2 !== Infinity) {
                const dx = x - eye.x, dy = y - eye.y, dz = z - eye.z;
                if (dx * dx + dy * dy + dz * dz > range2) continue;
            }
            const c = written * 3;
            positions[c] = x; positions[c + 1] = y; positions[c + 2] = z;
            colours[c] = d.paint[i * 3]; colours[c + 1] = d.paint[i * 3 + 1]; colours[c + 2] = d.paint[i * 3 + 2];
            written++;
        }
        points.geometry.setDrawRange(0, written);
        positionAttr.needsUpdate = true;
        colorAttr.needsUpdate = true;
        this._voxelsDrawn = written;
        this._voxelCullEye = eye;
    }

    /** The viewer's eye in the robot frame the cloud is stored in. */
    _robotFrameEye() {
        return this._frameRotate.worldToLocal(this.getCameraPositionWorld());
    }

    _updateVoxelCull(dt) {
        this._voxelCullAccumS += dt;
        if (this._voxelCullAccumS < VOXEL_CULL_INTERVAL_S) return;
        this._voxelCullAccumS = 0;
        if (!Number.isFinite(QUALITY_LEVELS[this._quality].voxel_range_m) || !this._voxelCullEye) return;
        const eye = this._robotFrameEye();
        const moved = eye.distanceTo(this._voxelCullEye) * (this._worldGroup.scale.x || 1);
        if (moved >= VOXEL_CULL_MOVE_M) this._compactCloud();
    }

    // ---- quality governor ---------------------------------------------------

    _recentMedianMs(count) {
        const n = Math.min(count, this._frameSampleCount);
        if (n === 0) return 0;
        const recent = new Float32Array(n);
        for (let k = 0; k < n; k++) {
            recent[k] = this._frameSamples[(this._frameSampleCursor - 1 - k + PERF_WINDOW) % PERF_WINDOW];
        }
        recent.sort();
        return recent[Math.floor(n / 2)];
    }

    _updateQuality(dt) {
        if (!this._qualityAuto) return;
        this._qualityAccumS += dt;
        if (this._qualityAccumS < QUALITY_INTERVAL_S) return;
        this._qualityAccumS = 0;
        if (this._frameSampleCount < QUALITY_SAMPLE_FRAMES / 2) return;
        const median = this._recentMedianMs(QUALITY_SAMPLE_FRAMES);
        if (median > QUALITY_STEP_DOWN_MS && this._quality < QUALITY_LEVELS.length - 1) {
            this._qualitySettleS = 0;
            this._applyQuality(this._quality + 1, median);
        } else if (median < QUALITY_STEP_UP_MS && this._quality > 0) {
            this._qualitySettleS += QUALITY_INTERVAL_S;
            if (this._qualitySettleS >= QUALITY_SETTLE_S) {
                this._qualitySettleS = 0;
                this._applyQuality(this._quality - 1, median);
            }
        } else {
            this._qualitySettleS = 0;
        }
    }

    /** Pin a level (0 = everything) or pass null to hand control back to the governor. */
    setQuality(level) {
        if (level === null || level === undefined) {
            this._qualityAuto = true;
            return this._quality;
        }
        this._qualityAuto = false;
        this._applyQuality(Math.max(0, Math.min(QUALITY_LEVELS.length - 1, level)), this._recentMedianMs(QUALITY_SAMPLE_FRAMES));
        return this._quality;
    }

    _applyQuality(level, medianMs) {
        this._quality = level;
        const q = QUALITY_LEVELS[level];
        if (this.three.xr.isPresenting) {
            // Fixed foveated rendering is the cheap lever on a headset.
            if (this.three.xr.setFoveation) this.three.xr.setFoveation(q.foveation);
        } else {
            this.three.setPixelRatio((window.devicePixelRatio || 1) * q.resolution);
            this._resizeDesktopCamera();
        }
        this._compactCloud();
        if (this._replayLayer) {
            this._replayLayer.fraction = q.voxel_fraction;
            if (this.onQualityChange) this.onQualityChange(level);
        }
        this._imageLodAccumS = IMAGE_LOD_INTERVAL_S; // re-budget thumbnails now
        this.diag('quality', { level, median_ms: Number((medianMs || 0).toFixed(1)), auto: this._qualityAuto });
    }

    /** Repaint the voxels around each point that carries a radius; the first
     *  point is the answer and gets the hotter colour. Points without a radius
     *  are capture poses, and painting the floor under the robot would mislead. */
    _highlightVoxels(points) {
        const d = this._cloudData;
        if (!d || !this._pointsObj) return;
        for (const index of this._highlightedVoxels) {
            for (let c = 0; c < 3; c++) d.paint[index * 3 + c] = d.colors ? d.colors[index * 3 + c] : 1;
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
                    d.paint[i * 3] = paint.r; d.paint[i * 3 + 1] = paint.g; d.paint[i * 3 + 2] = paint.b;
                    this._highlightedVoxels.push(i);
                }
            }
        });
        this._compactCloud();
        this.diag('voxels_highlighted', { n: this._highlightedVoxels.length, points: points.length });
    }

    /** Bring a robot-frame point to a few metres in front of the viewer. On the
     *  desktop the world is also lifted so the point sits at eye level; in VR
     *  the floor stays where the floor is. */
    focusOn(position) {
        this._stopOrbitBeforeMoving();
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
        this._applyQueryImageVisibility();
        this.diag('focused', { x, y, z });
    }

    /** Carve the map out of the way between `from` and `to` (robot frame), opening out
     *  to `radius` at the far end. Called with nothing to put the map back.
     *
     *  Standing where a picture was taken puts the viewer inside whatever the robot was
     *  looking at, so the picture hangs behind a wall of voxels and all you see is the
     *  wall. */
    setSightLine(from, to, radius) {
        // Remembered, not just applied: Photos off closes the cut, and Photos on has to
        // open the same one again. Nulling it there left the viewer standing at the pose
        // a picture was taken from, with the picture back and the wall back in front of
        // it -- the exact state this carve exists to prevent.
        this._sightLine = (from && to && radius > 0) ? { from, to, radius } : null;
        this._showSightLine();
    }

    /** The cut is open exactly while the photograph it was made for is showing. */
    _showSightLine() {
        // With Photos off there is nothing to see past, so the cut is a hole in the map
        // with nothing behind it.
        const cut = this._imageQuadGroup.visible ? this._sightLine : null;
        if (!cut) {
            this._sightRadius.value = 0;
            return;
        }
        this._sightFrom.value.set(cut.from[0], cut.from[1], cut.from[2]);
        this._sightTo.value.set(cut.to[0], cut.to[1], cut.to[2]);
        this._sightRadius.value = cut.radius;
    }

    /** Hide voxels above robot z `z` (Infinity/null shows all). */
    setRoofCut(z) {
        this._roofCut.value = (z === null || z === undefined) ? 1e9 : z;
    }

    toggleCloud() {
        this._cloudWanted = !this._cloudWanted;
        if (this._pointsObj) this._pointsObj.visible = this._cloudWanted && !this._replayActive;
        this.diag('cloud_toggle', { visible: this._cloudWanted });
        if (this.onLayerChange) this.onLayerChange();  // keyboard toggles reach the boxes too
    }

    setImagePoses(header, payloadArrayBuffer) {
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

        for (let i = 0; i < n; i++) {
            const quat = new THREE.Quaternion(
                quats[i * 4 + 0], quats[i * 4 + 1], quats[i * 4 + 2], quats[i * 4 + 3],
            ).multiply(faceBackward).multiply(standUpright);
            this._imagePoseMeta.push({
                id: header.ids?.[i] ?? null,
                sourceId: header.source_ids?.[i] ?? null,  // analyze_memory names these
                rx: positions[i * 3 + 0],
                ry: positions[i * 3 + 1],
                rz: positions[i * 3 + 2],
                quadQuat: quat,
            });
        }
        this.diag('image_poses_loaded', { n });
    }

    /** Thumbnails arrive once and are kept as JPEG bytes; decoding is deferred
     *  to `_updateImageLod` so only nearby poses ever cost a texture. */
    addImageThumbnail(index, jpegArrayBuffer) {
        if (!this._imagePoseMeta[index]) return;
        this._thumbnailBytes.set(index, jpegArrayBuffer);
    }

    /** Whether each answer's evidence photo should be on screen right now.
     *
     * Two different things put a picture in the world: the capture-pose markers, which
     * live in _imageQuadGroup, and an answer's evidence, which lives among the
     * highlights. "Photos" means both, so this is the single rule for the second kind
     * and every place that shows or hides one calls it.
     */
    _applyQueryImageVisibility() {
        const photos = this._imageQuadGroup.visible;
        this._showSightLine();
        const cursor = this._queryImageCursor;
        const filter = this.clusterFilter;
        this._queryImageMeshes.forEach((mesh, i) => {
            if (!mesh) return;
            const cluster = this._queryImages[i]?.cluster;
            const shown = photos
                && (cursor < 0 || i === cursor)
                && !(filter >= 0 && cluster !== undefined && cluster !== filter);
            mesh.visible = shown;
            // The ring on the matched pixel and the line to the voxel it produced belong
            // to this photograph and go with it.
            (this._queryMatchMarks[i] || []).forEach((mark) => { mark.visible = shown; });
        });
    }

    toggleImages() {
        this._imageQuadGroup.visible = !this._imageQuadGroup.visible;
        this._photosPinnedOff = !this._imageQuadGroup.visible;  // the user's own choice
        if (!this._imageQuadGroup.visible) this._releaseAllThumbnails();
        this._applyQueryImageVisibility();  // an answer's photos are photos too
        this._imageLodAccumS = IMAGE_LOD_INTERVAL_S;
        this.diag('images_toggle', { visible: this._imageQuadGroup.visible });
        if (this.onLayerChange) this.onLayerChange();
    }

    /** Keep decoded thumbnails to the nearest `IMAGE_QUAD_BUDGET` poses within
     *  `IMAGE_RENDER_DISTANCE_M`, measured in world metres so zooming out drops
     *  quads rather than piling up hundreds of textured, sorted transparents. */
    _updateImageLod(dt) {
        this._imageLodAccumS += dt;
        if (this._imageLodAccumS < IMAGE_LOD_INTERVAL_S) return;
        this._imageLodAccumS = 0;
        if (!this._imageQuadGroup.visible || this._imagePoseMeta.length === 0) return;
        // An answer that brought its own photographs has already said which pictures are
        // relevant; the capture-pose markers are every photo in the recording, and drawing
        // the nearest two dozen of those beside the answer's own is what made it impossible
        // to tell which pictures the answer was actually claiming.
        if (this._queryImageMeshes.some(Boolean)) {
            this._releaseAllThumbnails();
            return;
        }

        const eye = this._imageQuadGroup.worldToLocal(this.camera.getWorldPosition(new THREE.Vector3()));
        const scale = this._worldGroup.scale.x || 1;
        // A query answer is a handful of poses anywhere in the recording, so the
        // distance cutoff would hide the very thing the user asked to see. The
        // budget alone is enough to bound the cost there.
        const level = QUALITY_LEVELS[this._quality];
        const budget = Math.min(IMAGE_QUAD_BUDGET, level.quad_budget);
        // One id space on the wire: the server snaps every engine's answer to marker
        // ids before publishing, so an answer that selects nothing genuinely has nothing.
        const selected = this._selectedImageIds.size > 0 ? this._selectedImageIds : null;
        const maxDist = selected
            ? Infinity
            : Math.min(IMAGE_RENDER_DISTANCE_M, Number.isFinite(level.voxel_range_m) ? level.voxel_range_m : Infinity) / scale;

        const candidates = [];
        for (let i = 0; i < this._imagePoseMeta.length; i++) {
            const meta = this._imagePoseMeta[i];
            if (selected && !selected.has(meta.id)) continue;
            if (!this._thumbnailBytes.has(i)) continue;
            const dx = meta.rx - eye.x;
            const dy = meta.ry - eye.y;
            const dz = meta.rz - eye.z;
            const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (dist > maxDist) continue;
            candidates.push([dist, i]);
        }
        candidates.sort((a, b) => a[0] - b[0]);
        const wanted = new Set(candidates.slice(0, budget).map(([, i]) => i));

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
            quad.position.set(meta.rx, meta.ry, meta.rz);
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
        quad.material.map.image.close();   // the ImageBitmap; dispose() only drops the GPU copy
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
        this._focusPoint = result.focus_point || null;
        if (result.focus_point) {
            const focus = new THREE.Mesh(
                new THREE.SphereGeometry(0.18, 20, 16),
                new THREE.MeshBasicMaterial({ color: 0xffffff }),
            );
            focus.position.set(...result.focus_point);
            this._highlightGroup.add(focus);
        }

        this.clusterFilter = -1;
        this._selectedImageIds = new Set(result.observation_ids || []);
        // An answer turns the photos on to show its evidence, but never over the user:
        // turning them off and then asking a question used to bring them all back.
        if (this._selectedImageIds.size > 0 && !this._imageQuadGroup.visible && !this._photosPinnedOff) {
            this._imageQuadGroup.visible = true;
            if (this.onLayerChange) this.onLayerChange();  // the photos box follows
        }
        // The selection changes which poses deserve a texture, so rebuild now.
        this._releaseAllThumbnails();
        this._imageLodAccumS = IMAGE_LOD_INTERVAL_S;

        this._activeQueryId = result.query_id || null;
        this._queryImages = [];
        this._queryImageMeshes = [];
        this._queryMatchMarks = [];
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

    /** Hang each photograph behind an answer where its camera stood (evidence.js). */
    addQueryImage(header, jpegArrayBuffer) {
        addQueryImage(this, header, jpegArrayBuffer);
    }

    /** The pictures of the place being stepped through, in the order they were sent. */
    queryImagesHere() {
        return this._queryImages
            .map((header, index) => [header, index])
            .filter(([header]) => header && (this.clusterFilter < 0
                || header.cluster === undefined
                || header.cluster === this.clusterFilter))
            .map(([, index]) => index);
    }

    /** Step to the next/previous picture of this place. `step` is +1 or -1.
     *
     * Backwards was missing: places stepped both ways and pictures only forward, so the
     * one you wanted a second look at took a full lap of the place to reach.
     */
    stepQueryImage(step = 1) {
        if (!this._queryImages.length) return false;
        const here = this.queryImagesHere();
        if (!here.length) return false;
        const at = here.indexOf(this._queryImageCursor);
        // From nowhere, forward means the first and backward means the last.
        const to = at < 0
            ? (step > 0 ? 0 : here.length - 1)
            : (at + (step > 0 ? 1 : -1) + here.length) % here.length;
        return this.viewFrom(here[to]);
    }

    /** Stand where the camera behind answer *index* stood and look the way it
     *  looked, so the photo lines up with the voxels it was taken from.
     *  Desktop only: in VR the head is the camera. */
    viewFrom(index) {
        const header = this._queryImages[index];
        if (!header || this.three.xr.isPresenting) return false;
        // A photo the filter is hiding is not somewhere to stand; see jumpToAnswer.
        if (header.cluster !== undefined && this.clusterFilter >= 0
            && header.cluster !== this.clusterFilter) return false;
        this._stopOrbitBeforeMoving();
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
        // The corridor follows the eye. Without this, P walks you inside the wall each
        // picture shows with nothing carved, while the tunnel cut for the previous one
        // stays open somewhere behind you.
        this.setSightLine(...sightLineFor(header));
        this._applyQueryImageVisibility();
        this.diag('view_from', { index });
        return true;
    }

    /** Orbit rewrites the world position every frame; drop it before moving by hand. */
    _stopOrbitBeforeMoving() {
        if (this._orbit && this._orbit.active) this.setOrbit(false);
    }

    _addHighlightTube(path, radius, color) {
        const points = (path.points || []).map((p) => new THREE.Vector3(p[0], p[1], p[2]));
        if (points.length < 2) return;
        const curve = new THREE.CatmullRomCurve3(points, false, 'centripetal');
        const geometry = new THREE.TubeGeometry(curve, Math.max(16, points.length * 3), radius, 8, false);
        const material = new THREE.MeshBasicMaterial({ color });
        this._highlightGroup.add(new THREE.Mesh(geometry, material));
    }

    /** Take the current answer off the world: its highlights, its photographs, the
     *  corridor carved toward one of them, and the panel that says what it was.
     *
     *  One place, because Close used to reset the nav bar and leave 269 objects in the
     *  scene, and a question that failed left the corridor cut. Every caller wants all
     *  of it; none of them wants half. */
    clearAnswer() {
        this._clearHighlightGroup();
        this._queryImages = [];
        this._queryImageMeshes = [];
        this._queryMatchMarks = [];
        this._queryImageCursor = -1;
        this._activeQueryId = null;
        this.setSightLine(null);
        this._answerPanel.visible = false;
        if (this.onAnswerText) this.onAnswerText(null);  // the page's copy goes with it
        // The painted voxels and the selection are part of the answer too: left behind,
        // J still flew to a place that had been closed, and the voxels stayed lit.
        this._highlightVoxels([]);
        this._lastResultPoints = [];
        this._highlightedVoxels = [];
        this._selectedImageIds.clear();   // a Set, not a list
    }

    _clearHighlightGroup() {
        while (this._highlightGroup.children.length) {
            const child = this._highlightGroup.children.pop();
            child.traverse((obj) => {
                if (obj.geometry) obj.geometry.dispose();
                if (obj.material) {
                    if (obj.material.map) {   // the evidence photo: bitmap and GPU copy
                        obj.material.map.image?.close?.();
                        obj.material.map.dispose();
                    }
                    obj.material.dispose();
                }
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
        // Real text on the page, and the drawn quad only where HTML cannot go. The canvas
        // above is unselectable, blurry off-axis, and drops everything past four wrapped
        // lines; an immersive XR session is the one place that is still the best
        // available, because the DOM is not composited into it.
        const inXr = this.three.xr.isPresenting;
        this._answerPanel.visible = inXr;
        if (this.onAnswerText) this.onAnswerText(inXr ? null : String(answer));
        if (!this._hudGroupPinnedOff && !this._hudOff) this._hudGroup.visible = true;  // it lives here
    }

    setTopDownMap(header, jpegArrayBuffer) {
        const blob = new Blob([jpegArrayBuffer], { type: 'image/jpeg' });
        createImageBitmap(blob).then((bitmap) => {
            this._topDownBounds = header;
            // Only the minimap shows it (pasted on the floor it hid the voxels you stood
            // among); V flipped: the histogram's row 0 is at y_max.
            const hudTex = new THREE.Texture(bitmap);
            hudTex.needsUpdate = true;
            hudTex.colorSpace = THREE.SRGBColorSpace;
            hudTex.repeat.y = -1;
            hudTex.offset.y = 1;
            if (this._hudPanelMat.map) { this._hudPanelMat.map.image?.close?.(); this._hudPanelMat.map.dispose(); }
            this._hudPanelMat.color.set(0xffffff);
            this._hudPanelMat.map = hudTex;
            this._hudPanelMat.opacity = 0.95;
            this._hudPanelMat.needsUpdate = true;

            this.diag('top_down_map_loaded', { w: bitmap.width, h: bitmap.height });
        }).catch((e) => {
            this.diag('top_down_decode_failed', { error: String(e.message || e) });
        });
    }

    setOdomTrail(header, payloadArrayBuffer) {
        const n = header.n | 0;
        if (n < 2) return;
        const positions = new Float32Array(payloadArrayBuffer, 0, n * 3);
        // The trail's end is where orbit mode starts until the timeline says otherwise.
        const tail = (n - 1) * 3;
        this._odomTrailPoints = [[positions[tail], positions[tail + 1], positions[tail + 2] || 0]];
        this._trailPositions = positions;  // the whole path, for the tour

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
