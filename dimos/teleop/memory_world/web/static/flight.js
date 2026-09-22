import * as THREE from 'https://esm.sh/three@0.160.0';

// Camera flights: glide the viewer to an answer instead of teleporting it. The
// viewer moves the world group, not the camera, so a flight tweens the group's
// position plus the desktop look angles. The math mirrors focusOn and viewFrom
// in scene.js; this file only adds time.

const DEFAULT_DURATION_S = 1.3;
const DEFAULT_PITCH_RAD = -0.28;   // look slightly down at a target on the ground

function easeInOutCubic(t) {
    return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
}

function shortestAngle(from, to) {
    let d = (to - from) % (2 * Math.PI);
    if (d > Math.PI) d -= 2 * Math.PI;
    if (d < -Math.PI) d += 2 * Math.PI;
    return d;
}

export class Flight {
    constructor(scene) {
        this.scene = scene;
        this._active = null;
        this.onArrive = null;
    }

    get flying() {
        return this._active !== null;
    }

    /** Robot (x, y, z) to where three draws it: (x, z, -y) scaled like the world group. */
    _placed(position) {
        const [x, y, z] = position;
        return new THREE.Vector3(x, z, -y).multiplyScalar(this.scene._worldGroup.scale.x);
    }

    /** Put `target` (robot xyz) `distance` metres ahead of the viewer, seen from `yaw`. */
    lookAt(target, { distance = 4.0, yaw = null, pitch = DEFAULT_PITCH_RAD, duration = DEFAULT_DURATION_S } = {}) {
        const scene = this.scene;
        const xr = scene.three.xr.isPresenting;
        const headYaw = () => {
            const [fx, fz] = scene.getCameraForwardXZ();
            return Math.atan2(-fx, -fz);   // the camera looks down -z at yaw 0
        };
        const endYaw = xr ? headYaw() : (yaw === null ? scene._desktopYaw ?? 0 : yaw);
        const endPitch = xr ? 0 : pitch;
        const dir = new THREE.Vector3(
            -Math.sin(endYaw) * Math.cos(endPitch),
            Math.sin(endPitch),
            -Math.cos(endYaw) * Math.cos(endPitch),
        );
        const head = scene.getCameraPositionWorld();
        const scale = scene._worldGroup.scale.x;
        const placed = this._placed(target);
        const end = new THREE.Vector3(
            head.x + dir.x * distance * scale - placed.x,
            head.y + dir.y * distance * scale - placed.y,
            head.z + dir.z * distance * scale - placed.z,
        );
        if (xr) end.y = scene._worldGroup.position.y;   // in VR the floor stays put
        this._start(end, endYaw, endPitch, duration);
    }

    /** Stand where a hung frame's camera stood, looking the way it looked. */
    viewFrom(header, { duration = DEFAULT_DURATION_S } = {}) {
        const scene = this.scene;
        if (scene.three.xr.isPresenting) return false;
        const eye = this._placed(header.position);
        const f = new THREE.Vector3(...header.forward).normalize();
        const fwd = new THREE.Vector3(f.x, f.z, -f.y);
        const head = scene.getCameraPositionWorld();
        const end = new THREE.Vector3(head.x - eye.x, head.y - eye.y, head.z - eye.z);
        const yaw = Math.atan2(-fwd.x, -fwd.z);
        const pitch = Math.asin(Math.max(-1, Math.min(1, fwd.y)));
        this._start(end, yaw, pitch, duration);
        return true;
    }

    _start(endPosition, endYaw, endPitch, duration) {
        const scene = this.scene;
        const startYaw = scene._desktopYaw ?? 0;
        this._active = {
            t: 0,
            duration: Math.max(duration, 0.01),
            from: scene._worldGroup.position.clone(),
            to: endPosition,
            yaw0: startYaw,
            dyaw: shortestAngle(startYaw, endYaw),
            pitch0: scene._desktopPitch ?? 0,
            pitch1: endPitch,
        };
    }

    cancel() {
        this._active = null;
    }

    /** Advance by dt seconds; call once per frame. */
    tick(dt) {
        const a = this._active;
        if (!a) return;
        a.t = Math.min(a.t + dt, a.duration);
        const u = easeInOutCubic(a.t / a.duration);
        const scene = this.scene;
        scene._worldGroup.position.lerpVectors(a.from, a.to, u);
        if (!scene.three.xr.isPresenting && scene._desktopYaw !== undefined) {
            scene._desktopYaw = a.yaw0 + a.dyaw * u;
            scene._desktopPitch = a.pitch0 + (a.pitch1 - a.pitch0) * u;
            scene.camera.rotation.set(scene._desktopPitch, scene._desktopYaw, 0);
        }
        if (a.t >= a.duration) {
            this._active = null;
            if (this.onArrive) this.onArrive();
        }
    }
}
