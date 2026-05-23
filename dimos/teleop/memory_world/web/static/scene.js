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
// GTA-style HUD minimap — head-locked, sits at lower-left of view.
const HUD_PANEL_SIZE = 0.22;          // metres (square)
const HUD_MARKER_RADIUS = 0.008;
const HUD_DISTANCE = 0.55;            // metres in front of head
const HUD_OFFSET_DOWN = 0.25;
const HUD_OFFSET_LEFT = 0.32;
const HUD_FOLLOW_LERP = 0.18;         // damping per frame
// Image-thumbnail quads at capture poses.
const IMAGE_QUAD_W = 0.35;
const IMAGE_QUAD_H = 0.20;            // 16:9-ish
const IMAGE_QUAD_HEIGHT = 0.9;        // robot z (metres) — chest height in VR

export class WorldScene {
    constructor(diag) {
        this.diag = diag || (() => {});

        this.three = new THREE.WebGLRenderer({ alpha: false, antialias: false });
        this.three.setSize(window.innerWidth || 800, window.innerHeight || 600);
        this.three.setPixelRatio(window.devicePixelRatio || 1);
        this.three.xr.enabled = true;
        this.three.setClearColor(0x06090f, 1);

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
        this.scene.background = new THREE.Color(0x06090f);
        this.scene.add(new THREE.AmbientLight(0xffffff, 1.0));

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
        this._pointsObj = null;               // THREE.Points or THREE.InstancedMesh
        this._cloudData = null;               // {n, positions, colors, voxelSize}
        this._renderMode = 'cubes';           // 'cubes' | 'points', from header / toggle
        this._basePointSize = POINT_SIZE;     // overwritten by cloud header's voxel_size
        this._imagePoseGroup = new THREE.Group();     // always-on ring markers
        this._frameRotate.add(this._imagePoseGroup);
        this._imageQuadGroup = new THREE.Group();     // textured quads, toggleable
        this._imageQuadGroup.visible = false;
        this._frameRotate.add(this._imageQuadGroup);
        this._imagePoseMeta = [];                     // per-index {pos, quat}
        this._imageQuadsByIndex = new Map();          // index -> THREE.Mesh
        this._odomLine = null;

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

    _tick(timeMs) {
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

        // Points mode only: keep sprite size matched to voxel spacing as the
        // world scales, so they stay gap-free. Cubes are real geometry and
        // scale with the world group automatically.
        if (this._pointsObj && this._renderMode === 'points') {
            const want = this._basePointSize * (this._worldGroup.scale.x || 1);
            if (Math.abs(this._pointsObj.material.size - want) > 1e-4) {
                this._pointsObj.material.size = want;
            }
        }

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

        // HUD goes to the user's LEFT, which is -right.
        const target = new THREE.Vector3()
            .copy(headPos)
            .addScaledVector(fwd, HUD_DISTANCE)
            .addScaledVector(right, -HUD_OFFSET_LEFT);
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
        // Keep the parsed cloud so we can re-render it in either mode without a
        // server round-trip when the user toggles points <-> cubes.
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
        this._renderMode = header.render === 'points' ? 'points' : 'cubes';
        this._basePointSize = this._cloudData.voxelSize;

        this._rebuildCloud();

        this._cloudBounds = header.bounds || null;
        if (!this._hasSpawned) this._spawnAtCentroid();
        this.diag('point_cloud_loaded', { n, mode: this._renderMode, has_colors: !!header.has_colors });
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

        if (this._renderMode === 'cubes') {
            // One InstancedMesh of unit cubes scaled to the voxel size. Real
            // geometry, so it scales correctly with the world group on zoom.
            const box = new THREE.BoxGeometry(d.voxelSize, d.voxelSize, d.voxelSize);
            const mat = new THREE.MeshBasicMaterial({ vertexColors: false });
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
        } else {
            const geom = new THREE.BufferGeometry();
            geom.setAttribute('position', new THREE.BufferAttribute(d.positions, 3));
            const size = d.voxelSize * (this._worldGroup.scale.x || 1);
            let mat;
            if (d.colors) {
                geom.setAttribute('color', new THREE.BufferAttribute(d.colors, 3));
                mat = new THREE.PointsMaterial({ size, vertexColors: true, sizeAttenuation: true });
            } else {
                mat = new THREE.PointsMaterial({ color: 0x7af0a8, size, sizeAttenuation: true });
            }
            this._pointsObj = new THREE.Points(geom, mat);
        }
        this._frameRotate.add(this._pointsObj);
    }

    toggleRenderMode() {
        this._renderMode = this._renderMode === 'cubes' ? 'points' : 'cubes';
        this._rebuildCloud();
        this.diag('render_mode', { mode: this._renderMode });
    }

    setImagePoses(header, payloadArrayBuffer) {
        // Clear previous.
        while (this._imagePoseGroup.children.length) {
            const c = this._imagePoseGroup.children.pop();
            if (c.geometry) c.geometry.dispose();
            if (c.material) c.material.dispose();
        }
        while (this._imageQuadGroup.children.length) {
            const c = this._imageQuadGroup.children.pop();
            if (c.geometry) c.geometry.dispose();
            if (c.material && c.material.map) c.material.map.dispose();
            if (c.material) c.material.dispose();
        }
        this._imagePoseMeta = [];
        this._imageQuadsByIndex.clear();

        const n = header.n | 0;
        if (n === 0) return;

        const positions = new Float32Array(payloadArrayBuffer, 0, n * 3);
        const quats = new Float32Array(payloadArrayBuffer, n * 12, n * 4);
        const ringGeom = new THREE.RingGeometry(0.10, 0.13, 24);
        const ringMat = new THREE.MeshBasicMaterial({
            color: 0x4cd9ff,
            transparent: true,
            opacity: 0.7,
            side: THREE.DoubleSide,
        });
        for (let i = 0; i < n; i++) {
            const rx = positions[i * 3 + 0];
            const ry = positions[i * 3 + 1];
            const rz = positions[i * 3 + 2];
            const qx = quats[i * 4 + 0];
            const qy = quats[i * 4 + 1];
            const qz = quats[i * 4 + 2];
            const qw = quats[i * 4 + 3];
            const ring = new THREE.Mesh(ringGeom, ringMat);
            ring.position.set(rx, ry, 0.02);
            this._imagePoseGroup.add(ring);
            this._imagePoseMeta.push({ rx, ry, rz, qx, qy, qz, qw });
        }
        this.diag('image_poses_loaded', { n });
    }

    addImageThumbnail(index, jpegArrayBuffer) {
        const meta = this._imagePoseMeta[index];
        if (!meta) return;
        if (this._imageQuadsByIndex.has(index)) return;

        // Decode JPEG via Blob + ImageBitmap so it's GPU-friendly + async.
        // Three.js doesn't reliably apply flipY to ImageBitmap textures, so we
        // flip at decode time and disable the texture's own flip — otherwise
        // the photos render upside down.
        const blob = new Blob([jpegArrayBuffer], { type: 'image/jpeg' });
        createImageBitmap(blob, { imageOrientation: 'flipY' }).then((bitmap) => {
            const tex = new THREE.Texture(bitmap);
            tex.flipY = false;
            tex.colorSpace = THREE.SRGBColorSpace;
            tex.needsUpdate = true;
            const mat = new THREE.MeshBasicMaterial({
                map: tex,
                side: THREE.DoubleSide,
                transparent: true,
                opacity: 0.95,
            });
            const quad = new THREE.Mesh(
                new THREE.PlaneGeometry(IMAGE_QUAD_W, IMAGE_QUAD_H),
                mat,
            );
            // Place at the pose's XY in robot frame, lifted to ~chest height
            // (Z up). Quad's local +Z points toward the camera; orient it via
            // the recorded quaternion (camera pose). The quat is in robot frame
            // where camera forward is robot +X, so we apply directly then rotate
            // so the plane faces the recorded forward direction.
            quad.position.set(meta.rx, meta.ry, IMAGE_QUAD_HEIGHT);
            // Make the quad face along the robot's forward direction at capture.
            const q = new THREE.Quaternion(meta.qx, meta.qy, meta.qz, meta.qw);
            // Default PlaneGeometry normal is +Z. Rotate so normal points
            // along robot -X (i.e. "behind" the capture direction), so the
            // image is seen face-on when the viewer is in front of the pose.
            const baseRot = new THREE.Quaternion().setFromEuler(
                new THREE.Euler(0, Math.PI / 2, Math.PI / 2)
            );
            quad.quaternion.copy(q).multiply(baseRot);
            this._imageQuadGroup.add(quad);
            this._imageQuadsByIndex.set(index, quad);
        }).catch((e) => {
            this.diag('thumbnail_decode_failed', { index, error: String(e.message || e) });
        });
    }

    toggleImages() {
        this._imageQuadGroup.visible = !this._imageQuadGroup.visible;
        this.diag('images_toggle', { visible: this._imageQuadGroup.visible });
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
                opacity: 0.85,
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
