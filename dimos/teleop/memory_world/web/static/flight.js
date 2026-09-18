// Camera flights: instead of teleporting the viewer to an answer, glide there.
//
// The viewer moves the world, not the camera, so a flight is a tween of the
// world group's position plus, on the desktop, the look yaw and pitch. The
// maths mirrors focusOn()/viewFrom() in scene.js; this file only adds time.

import * as THREE from 'https://esm.sh/three@0.160.0';
import { robotToWorldOffset } from '/static_mw/world_frame.js';

const DEFAULT_DURATION_S = 1.4;
const DEFAULT_PITCH_RAD = -0.28;     // look slightly down at a target on the floor

function easeInOutCubic(t) {
    return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
}

function shortestAngle(from, to) {
    let d = (to - from) % (2 * Math.PI);
    if (d > Math.PI) d -= 2 * Math.PI;
    if (d < -Math.PI) d += 2 * Math.PI;
    return d;
}

/** robot (x, y, z) -> three (x, z, -y), scaled like the world group. */
export class Flight {
    constructor(scene) {
        this.scene = scene;
        this._active = null;
        this.onArrive = null;
    }

    get flying() {
        return this._active !== null;
    }

    /** Frame `target` (robot xyz) `distance` metres ahead, looking at it from `yaw` (radians; current if null). */
    lookAt(target, { distance = 4.0, yaw = null, pitch = DEFAULT_PITCH_RAD, duration = DEFAULT_DURATION_S } = {}) {
        const scene = this.scene;
        const xr = scene.three.xr.isPresenting;
        // In XR the head IS the camera, so the caller's yaw cannot be honoured -- but the
        // direction we place the target in still has to be the one the person is facing.
        // This read `scene._desktopYaw ?? 0`, and `_desktopYaw` is only created by
        // startDesktop(), which an immersive session never calls: endYaw was always 0, so
        // `distance` metres "ahead" meant world -Z however the headset was turned.
        const headYaw = () => {
            const [fx, fz] = scene.getCameraForwardXZ();
            return Math.atan2(-fx, -fz);   // the camera looks down -z at yaw 0
        };
        const endYaw = xr ? headYaw() : (yaw === null ? scene._desktopYaw ?? 0 : yaw);
        const endPitch = xr ? 0 : pitch;
        // The camera looks down -z at yaw 0; pitch positive looks up.
        const dir = new THREE.Vector3(
            -Math.sin(endYaw) * Math.cos(endPitch),
            Math.sin(endPitch),
            -Math.cos(endYaw) * Math.cos(endPitch),
        );
        const head = scene.getCameraPositionWorld();
        const scale = scene._worldGroup.scale.x;
        // One implementation of "where three draws this point", shared with focusOn,
        // viewFrom and OrbitControl.apply -- which each had their own copy of the
        // unrotated version, and so each teleported you somewhere else the moment
        // anyone turned. The comment that used to live here is on the method.
        const placed = robotToWorldOffset(scene._worldGroup, target);
        const end = new THREE.Vector3(
            head.x + dir.x * distance * scale - placed.x,
            head.y + dir.y * distance * scale - placed.y,
            head.z + dir.z * distance * scale - placed.z,
        );
        if (xr) end.y = scene._worldGroup.position.y;   // in VR the floor stays put
        this._start(end, endYaw, endPitch, duration);
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
        if (scene._orbit && scene._orbit.active) scene.setOrbit(false);
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
