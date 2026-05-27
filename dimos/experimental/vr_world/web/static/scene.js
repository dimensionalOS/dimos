// Three.js scene for the live VR World.
//
// Coordinate frames: robot is X-fwd/Y-left/Z-up; three.js is Y-up. Everything
// lives under _frameRotate (rotation.x = -90°) so we author in robot coords.
// Locomotion of the *viewer* moves _worldGroup (yaw / scale / teleport); the
// robot itself is driven separately via cmd_vel on the server.
//
// Public methods (called by main.js):
//   setSession(session, perFrame)
//   setVoxelMap(header, payload)   // full map resend, replace each time
//   setRobotPose([x,y,z,qx,qy,qz,qw])
//   setCameraFrame(jpegArrayBuffer)
//   applyYaw / setTeleportAim / clearTeleportAim / applyTeleportCommit /
//   applyScale / resetView / toggleRenderMode

import * as THREE from 'https://esm.sh/three@0.160.0';

const YAW_RATE_MAX = Math.PI / 2;
const POINT_SIZE = 0.05;
const MIN_SCALE = 0.02, MAX_SCALE = 20.0;
const TELEPORT_ARC_SEGMENTS = 24, TELEPORT_MAX_DISTANCE = 8.0;
const CAM_PANEL_W = 0.5, CAM_PANEL_H = 0.30;   // live camera HUD panel

export class WorldScene {
    constructor(diag) {
        this.diag = diag || (() => {});

        this.three = new THREE.WebGLRenderer({ alpha: false, antialias: false });
        this.three.setSize(window.innerWidth || 800, window.innerHeight || 600);
        this.three.setPixelRatio(window.devicePixelRatio || 1);
        this.three.xr.enabled = true;
        this.three.setClearColor(0x06090f, 1);
        const dom = this.three.domElement;
        Object.assign(dom.style, { position: 'fixed', top: '0', left: '0', width: '100vw', height: '100vh', zIndex: '50', pointerEvents: 'none' });
        document.body.appendChild(dom);

        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x06090f);
        this.scene.add(new THREE.AmbientLight(0xffffff, 1.0));
        this.camera = new THREE.PerspectiveCamera(70, 1, 0.05, 500);

        this._worldGroup = new THREE.Group();
        this._frameRotate = new THREE.Group();
        this._frameRotate.rotation.x = -Math.PI / 2;
        this._worldGroup.add(this._frameRotate);
        this.scene.add(this._worldGroup);

        const grid = new THREE.GridHelper(20, 20, 0x1f2a3a, 0x1f2a3a);
        grid.rotation.x = Math.PI / 2;
        this._frameRotate.add(grid);

        // Voxel cloud (replaced on each map resend).
        this._cloudObj = null;
        this._cloudData = null;
        this._renderMode = 'cubes';
        this._basePointSize = POINT_SIZE;

        // Robot avatar — a cone pointing along robot +X (forward), at the pose.
        const coneGeom = new THREE.ConeGeometry(0.18, 0.5, 16);
        coneGeom.rotateZ(-Math.PI / 2);   // cone tip → +X (robot forward)
        this._robot = new THREE.Mesh(coneGeom, new THREE.MeshBasicMaterial({ color: 0xff8a3d }));
        this._robot.position.set(0, 0, 0.25);
        this._frameRotate.add(this._robot);
        this._trailPts = [];
        this._trail = new THREE.Line(
            new THREE.BufferGeometry(),
            new THREE.LineBasicMaterial({ color: 0xff8a3d, transparent: true, opacity: 0.7 }),
        );
        this._frameRotate.add(this._trail);

        // Live camera HUD panel (head-locked, lower-right).
        this._camCanvas = document.createElement('canvas');
        this._camCanvas.width = 320; this._camCanvas.height = 192;
        this._camTex = new THREE.CanvasTexture(this._camCanvas);
        this._camTex.colorSpace = THREE.SRGBColorSpace;
        this._camPanel = new THREE.Mesh(
            new THREE.PlaneGeometry(CAM_PANEL_W, CAM_PANEL_H),
            new THREE.MeshBasicMaterial({ map: this._camTex, transparent: true, opacity: 0.95, side: THREE.DoubleSide }),
        );
        this._camGroup = new THREE.Group();
        this._camGroup.add(this._camPanel);
        this.scene.add(this._camGroup);

        // Teleport aim.
        this._teleportArc = null;
        this._teleportTarget = new THREE.Vector3();
        this._teleportTargetValid = false;
        this._teleportMarker = new THREE.Mesh(
            new THREE.RingGeometry(0.12, 0.18, 32),
            new THREE.MeshBasicMaterial({ color: 0xff8a3d, transparent: true, opacity: 0, side: THREE.DoubleSide }),
        );
        this._teleportMarker.rotation.x = -Math.PI / 2;
        this.scene.add(this._teleportMarker);

        this._pendingYawRate = 0;
        this._lastTickMs = 0;
        this._hasSpawned = false;
    }

    async setSession(session, perFrame) {
        const gl = this.three.getContext();
        if (gl && gl.makeXRCompatible) { try { await gl.makeXRCompatible(); } catch (_) {} }
        this.three.xr.setReferenceSpaceType('local-floor');
        await this.three.xr.setSession(session);
        this.three.setAnimationLoop((time, frame) => {
            if (perFrame) perFrame(frame);
            this._tick(time);
            this.three.render(this.scene, this.camera);
        });
    }

    _tick(timeMs) {
        const dt = this._lastTickMs ? Math.max((timeMs - this._lastTickMs) / 1000, 0) : 0;
        this._lastTickMs = timeMs;
        if (dt > 0 && Math.abs(this._pendingYawRate) > 1e-3) {
            this._rotateWorldAround(this.getCameraPositionWorld(), this._pendingYawRate * dt);
        }
        // Yaw is a per-frame command: consume it so the world stops the instant
        // the input stops feeding a rate (e.g. controller not seen this frame).
        this._pendingYawRate = 0;
        if (this._cloudObj && this._renderMode === 'points') {
            const want = this._basePointSize * (this._worldGroup.scale.x || 1);
            if (Math.abs(this._cloudObj.material.size - want) > 1e-4) this._cloudObj.material.size = want;
        }
        this._updateCamPanel();
    }

    _updateCamPanel() {
        const cam = this.three.xr.isPresenting ? this.three.xr.getCamera(this.camera) : this.camera;
        cam.updateMatrixWorld();
        const head = new THREE.Vector3(); cam.getWorldPosition(head);
        const right = new THREE.Vector3(); const fwd = new THREE.Vector3();
        right.setFromMatrixColumn(cam.matrixWorld, 0);
        fwd.setFromMatrixColumn(cam.matrixWorld, 2).negate();
        right.y = 0; fwd.y = 0;
        if (right.lengthSq() < 1e-6 || fwd.lengthSq() < 1e-6) return;
        right.normalize(); fwd.normalize();
        const target = new THREE.Vector3().copy(head).addScaledVector(fwd, 0.6).addScaledVector(right, 0.32);
        target.y -= 0.22;
        this._camGroup.position.lerp(target, 0.18);
        this._camGroup.lookAt(head);
    }

    // ---- view navigation ---------------------------------------------------

    applyYaw(g) { this._pendingYawRate = g.rate || 0; }

    applyScale(g) {
        const factor = Math.max(0.2, Math.min(5.0, g.factor || 1.0));
        const next = this._worldGroup.scale.x * factor;
        if (next < MIN_SCALE || next > MAX_SCALE) return;
        const pivot = g.pivotWorld ? new THREE.Vector3(...g.pivotWorld) : this.getCameraPositionWorld();
        this._worldGroup.position.x = pivot.x + factor * (this._worldGroup.position.x - pivot.x);
        this._worldGroup.position.y = pivot.y + factor * (this._worldGroup.position.y - pivot.y);
        this._worldGroup.position.z = pivot.z + factor * (this._worldGroup.position.z - pivot.z);
        this._worldGroup.scale.multiplyScalar(factor);
    }

    applyTeleportCommit() {
        if (!this._teleportTargetValid) return;
        const head = this.getCameraPositionWorld();
        this._worldGroup.position.x -= (this._teleportTarget.x - head.x);
        this._worldGroup.position.z -= (this._teleportTarget.z - head.z);
        this.clearTeleportAim();
    }

    setTeleportAim(g) {
        const o = g.originWorld, d = g.dirWorld;
        if (!o || !d) return;
        let t = (0 - o[1]) / (d[1] < -1e-3 ? d[1] : -1e-3);
        if (t < 0 || t > TELEPORT_MAX_DISTANCE) t = TELEPORT_MAX_DISTANCE;
        const hit = new THREE.Vector3(o[0] + d[0] * t, 0, o[2] + d[2] * t);
        this._teleportTarget.copy(hit);
        this._teleportTargetValid = true;
        const start = new THREE.Vector3(o[0], o[1], o[2]);
        const horiz = Math.hypot(hit.x - start.x, hit.z - start.z);
        const apexY = (start.y + hit.y) / 2 + Math.max(0.1, horiz * 0.25);
        const cx = (start.x + hit.x) / 2, cz = (start.z + hit.z) / 2;
        const pts = [];
        for (let i = 0; i <= TELEPORT_ARC_SEGMENTS; i++) {
            const u = i / TELEPORT_ARC_SEGMENTS;
            pts.push(new THREE.Vector3(
                (1 - u) ** 2 * start.x + 2 * (1 - u) * u * cx + u * u * hit.x,
                (1 - u) ** 2 * start.y + 2 * (1 - u) * u * apexY + u * u * hit.y,
                (1 - u) ** 2 * start.z + 2 * (1 - u) * u * cz + u * u * hit.z,
            ));
        }
        if (this._teleportArc) { this._teleportArc.geometry.dispose(); this._teleportArc.geometry = new THREE.BufferGeometry().setFromPoints(pts); this._teleportArc.visible = true; }
        else { this._teleportArc = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0xff8a3d })); this.scene.add(this._teleportArc); }
        this._teleportMarker.position.copy(hit); this._teleportMarker.position.y += 0.005;
        this._teleportMarker.material.opacity = 0.9;
    }

    clearTeleportAim() {
        this._teleportTargetValid = false;
        if (this._teleportArc) this._teleportArc.visible = false;
        this._teleportMarker.material.opacity = 0;
    }

    resetView() {
        this._worldGroup.position.set(0, 0, 0);
        this._worldGroup.rotation.set(0, 0, 0);
        this._worldGroup.scale.set(1, 1, 1);
        this._hasSpawned = false;
        if (this._cloudData) this._spawnOverview();
    }

    getCameraPositionWorld() {
        const cam = this.three.xr.isPresenting ? this.three.xr.getCamera(this.camera) : this.camera;
        const p = new THREE.Vector3(); cam.getWorldPosition(p); return p;
    }

    _rotateWorldAround(pivot, angle) {
        const dx = this._worldGroup.position.x - pivot.x, dz = this._worldGroup.position.z - pivot.z;
        const c = Math.cos(angle), s = Math.sin(angle);
        this._worldGroup.position.x = pivot.x + (c * dx + s * dz);
        this._worldGroup.position.z = pivot.z + (-s * dx + c * dz);
        this._worldGroup.rotation.y += angle;
    }

    // ---- live data ---------------------------------------------------------

    setVoxelMap(header, payloadArrayBuffer) {
        const n = header.n | 0;
        if (n === 0) return;
        const positions = new Float32Array(payloadArrayBuffer.slice(0, n * 12));
        const rgb = new Uint8Array(payloadArrayBuffer, n * 12, n * 3);
        const colors = new Float32Array(n * 3);
        for (let i = 0; i < n * 3; i++) colors[i] = rgb[i] / 255;
        this._cloudData = { n, positions, colors, voxelSize: header.voxel_size || POINT_SIZE };
        this._basePointSize = this._cloudData.voxelSize;
        this._cloudBounds = header.bounds || null;
        this._rebuildCloud();
        if (!this._hasSpawned) this._spawnOverview();
    }

    _rebuildCloud() {
        const d = this._cloudData;
        if (!d) return;
        if (this._cloudObj) {
            this._frameRotate.remove(this._cloudObj);
            this._cloudObj.geometry.dispose();
            if (this._cloudObj.material) this._cloudObj.material.dispose();
            this._cloudObj = null;
        }
        if (this._renderMode === 'cubes') {
            const box = new THREE.BoxGeometry(d.voxelSize, d.voxelSize, d.voxelSize);
            const mesh = new THREE.InstancedMesh(box, new THREE.MeshBasicMaterial(), d.n);
            mesh.instanceMatrix.setUsage(THREE.StaticDrawUsage);
            const dummy = new THREE.Object3D(); const col = new THREE.Color();
            for (let i = 0; i < d.n; i++) {
                dummy.position.set(d.positions[i * 3], d.positions[i * 3 + 1], d.positions[i * 3 + 2]);
                dummy.updateMatrix(); mesh.setMatrixAt(i, dummy.matrix);
                col.setRGB(d.colors[i * 3], d.colors[i * 3 + 1], d.colors[i * 3 + 2]);
                mesh.setColorAt(i, col);
            }
            mesh.instanceMatrix.needsUpdate = true;
            if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
            this._cloudObj = mesh;
        } else {
            const geom = new THREE.BufferGeometry();
            geom.setAttribute('position', new THREE.BufferAttribute(d.positions, 3));
            geom.setAttribute('color', new THREE.BufferAttribute(d.colors, 3));
            const size = d.voxelSize * (this._worldGroup.scale.x || 1);
            this._cloudObj = new THREE.Points(geom, new THREE.PointsMaterial({ size, vertexColors: true, sizeAttenuation: true }));
        }
        this._frameRotate.add(this._cloudObj);
    }

    toggleRenderMode() {
        this._renderMode = this._renderMode === 'cubes' ? 'points' : 'cubes';
        this._rebuildCloud();
    }

    setRobotPose(pose) {
        if (!pose || pose.length < 7) return;
        const [x, y, z, qx, qy, qz, qw] = pose;
        this._robot.position.set(x, y, (z || 0) + 0.05);
        this._robot.quaternion.set(qx, qy, qz, qw);
        // Grow trail (cap length).
        const last = this._trailPts.length ? this._trailPts[this._trailPts.length - 1] : null;
        if (!last || Math.hypot(last.x - x, last.y - y) > 0.05) {
            this._trailPts.push(new THREE.Vector3(x, y, 0.03));
            if (this._trailPts.length > 4000) this._trailPts.shift();
            this._trail.geometry.dispose();
            this._trail.geometry = new THREE.BufferGeometry().setFromPoints(this._trailPts);
        }
    }

    setCameraFrame(jpegArrayBuffer) {
        // Plain decode: CanvasTexture (flipY=true by default) already orients a
        // top-down canvas correctly. Do NOT pre-flip the bitmap or it inverts.
        const blob = new Blob([jpegArrayBuffer], { type: 'image/jpeg' });
        createImageBitmap(blob).then((bitmap) => {
            const ctx = this._camCanvas.getContext('2d');
            ctx.drawImage(bitmap, 0, 0, this._camCanvas.width, this._camCanvas.height);
            this._camTex.needsUpdate = true;
            bitmap.close && bitmap.close();
        }).catch(() => {});
    }

    _spawnOverview() {
        if (!this._cloudBounds) return;
        const b = this._cloudBounds;
        const cx = (b.x_min + b.x_max) / 2, cy = (b.y_min + b.y_max) / 2;
        // Shrink to a dollhouse and place the map a couple metres in front so
        // you start with a god view of the whole scene.
        const span = Math.max(b.x_max - b.x_min, b.y_max - b.y_min, 1);
        const s = Math.min(1, 3.0 / span);
        this._worldGroup.scale.setScalar(s);
        const head = this.getCameraPositionWorld();
        // After frame-rotate, robot (cx,cy,0) → three (cx,0,-cy); scaled by s.
        this._worldGroup.position.x = head.x - s * cx;
        this._worldGroup.position.z = (head.z - 2.0) - s * (-cy);
        this._hasSpawned = true;
        this.diag('spawned_overview', { cx, cy, scale: s });
    }
}
