// Walking the recording's own path, as a camera move.
//
// The odom trail is where the robot went, and this flies the viewer along it at a
// steady pace looking where it was heading. Steady by ARC LENGTH, not by sample: the
// trail is sampled evenly in TIME, so a robot that stopped for a minute leaves a knot
// of samples in one spot, and stepping sample-by-sample would crawl through the knot
// and then sprint down the corridor after it.
//
// The viewer moves the world, not the camera, so each frame places `worldGroup` so
// that the point on the path lands where the head is -- the same trick `viewFrom`
// uses to stand you where a photograph was taken.

import { desktopLookAngles, robotToWorldOffset } from '/static_mw/world_frame.js';

const SPEED_MPS = 2.2;             // brisk walk; faster reads as a fairground ride
const LOOK_AHEAD_M = 3.5;          // aim at the path this far on, so corners lead
const EYE_LIFT_M = 0.25;           // a little above the path, so the floor is not in your face
const TURN_SMOOTHING_HZ = 2.5;     // how fast the look catches up to the direction of travel
const EASE_M = 3.0;                // metres of slow-in at the start and slow-out at the end
const MIN_PATH_M = 1.0;            // a path shorter than this is a standing robot

function shortestAngle(from, to) {
    let d = (to - from) % (2 * Math.PI);
    if (d > Math.PI) d -= 2 * Math.PI;
    if (d < -Math.PI) d += 2 * Math.PI;
    return d;
}

/** The path as points plus the distance along it at each one. */
function measure(positions) {
    const n = Math.floor(positions.length / 3);
    const points = [];
    const along = [];
    let travelled = 0;
    for (let i = 0; i < n; i++) {
        const point = [positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2] || 0];
        if (i > 0) {
            const previous = points[points.length - 1];
            const step = Math.hypot(point[0] - previous[0], point[1] - previous[1], point[2] - previous[2]);
            // Drop a repeat outright: two identical samples have no direction between
            // them, and a zero-length segment makes the tangent NaN.
            if (step < 1e-4) continue;
            travelled += step;
        }
        points.push(point);
        along.push(travelled);
    }
    return { points, along, length: travelled };
}

export class FlyThrough {
    constructor(scene) {
        this.scene = scene;
        this._path = null;
        this._at = 0;          // metres travelled
        this._yaw = 0;
        this._pitch = 0;
        this.onChange = null;  // told when it starts and when it stops
    }

    get flying() {
        return this._path !== null;
    }

    /** How far along, 0..1, or 0 when it is not running. */
    get progress() {
        return this._path && this._path.length > 0 ? this._at / this._path.length : 0;
    }

    /** Start at the beginning of the recording's path. False when there is no path. */
    start() {
        const positions = this.scene._trailPositions;
        if (!positions || positions.length < 6) return false;
        const path = measure(positions);
        if (path.length < MIN_PATH_M) return false;
        this._path = path;
        this._at = 0;
        // The look starts where the camera already is, so the first frame turns rather
        // than snapping -- the smoothing below does the rest.
        this._yaw = this.scene._desktopYaw ?? 0;
        this._pitch = this.scene._desktopPitch ?? 0;
        if (this.scene.isOrbiting && this.scene.isOrbiting()) this.scene.setOrbit(false);
        this.scene.diag('flythrough_start', { length_m: Number(path.length.toFixed(1)) });
        if (this.onChange) this.onChange();
        return true;
    }

    stop(reason = 'stopped') {
        if (!this._path) return;
        const done = this._at >= this._path.length;
        this._path = null;
        this.scene.diag('flythrough_stop', { reason, completed: done });
        if (this.onChange) this.onChange();
    }

    /** Where the path is `metres` along it, interpolated between samples. */
    _pointAt(metres) {
        const { points, along } = this._path;
        const target = Math.max(0, Math.min(metres, along[along.length - 1]));
        // Walk from the last index rather than searching: the cursor only moves forward.
        let i = this._cursor || 0;
        if (along[i] > target) i = 0;
        while (i < along.length - 2 && along[i + 1] < target) i += 1;
        this._cursor = i;
        const span = along[i + 1] - along[i];
        const u = span > 1e-6 ? (target - along[i]) / span : 0;
        const a = points[i];
        const b = points[i + 1];
        return [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u];
    }

    tick(dt) {
        if (!this._path || !Number.isFinite(dt)) return;
        const scene = this.scene;
        // A background tab delivers one enormous frame on wake; taking it at face value
        // teleports you most of the way down the corridor. Cap the step.
        const step = Math.min(dt, 0.1);
        // Slow in and out, so the start and the finish are not jerks. Both ends taper
        // over EASE_M, and never below a quarter speed or the ends take forever.
        const left = this._path.length - this._at;
        const taper = Math.min(this._at, left, EASE_M) / EASE_M;
        const speed = SPEED_MPS * Math.max(0.25, taper);
        this._at = Math.min(this._at + speed * step, this._path.length);

        const eye = this._pointAt(this._at);
        const ahead = this._pointAt(this._at + LOOK_AHEAD_M);
        const toward = [ahead[0] - eye[0], ahead[1] - eye[1], ahead[2] - eye[2]];
        const reach = Math.hypot(toward[0], toward[1], toward[2]);
        if (reach > 1e-3) {
            const angles = desktopLookAngles(scene._worldGroup, toward);
            // Exponential smoothing rather than a hard set: the trail is a real robot's
            // odometry and its tangent jitters, which reads as the camera shivering.
            const k = 1 - Math.exp(-TURN_SMOOTHING_HZ * step);
            this._yaw += shortestAngle(this._yaw, angles.yaw) * k;
            this._pitch += (angles.pitch - this._pitch) * k;
        }

        const placed = robotToWorldOffset(scene._worldGroup, [eye[0], eye[1], eye[2] + EYE_LIFT_M]);
        const head = scene.getCameraPositionWorld();
        scene._worldGroup.position.set(head.x - placed.x, head.y - placed.y, head.z - placed.z);
        if (!scene.three.xr.isPresenting && scene._desktopYaw !== undefined) {
            scene._desktopYaw = this._yaw;
            scene._desktopPitch = this._pitch;
            scene.camera.rotation.set(this._pitch, this._yaw, 0);
        }
        if (this._at >= this._path.length) this.stop('arrived');
    }
}
