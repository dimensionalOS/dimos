// Orbit mode: the eye circles a frame of the robot instead of walking.
//
// The viewer moves the world, not the camera, so orbiting means placing the
// world such that the target sits a fixed distance straight ahead of the eye
// along whatever direction the look controls have set. Dragging (which turns
// the camera) then swings the eye around the target; the wheel changes the
// distance; and the target itself follows the robot as the timeline moves.

import * as THREE from 'https://esm.sh/three@0.160.0';
import { robotToWorldOffset } from '/static_mw/world_frame.js';

const ORBIT_DISTANCE_M = 6.0;
const ORBIT_ZOOM_STEP = 1.12;
const ORBIT_MIN_M = 0.5;
const ORBIT_MAX_M = 60;

export class OrbitControl {
    constructor() {
        this.target = null;       // robot coords [x, y, z]
        this.distance = ORBIT_DISTANCE_M;
        this.active = false;
        this._marker = null;
    }

    /** Where the orbited frame is now; the eye follows it while orbiting. */
    setTarget(position, scene) {
        if (!position) return;
        this.target = [position[0], position[1], position[2]];
        if (this._marker) this._marker.position.set(...this.target);
        if (this.active) this.apply(scene);
    }

    enable(scene) {
        if (!this.target) {
            const trail = scene._odomTrailPoints;
            if (trail && trail.length) this.target = trail[trail.length - 1].slice();
            else if (scene._lastResultPoints.length) this.target = scene._lastResultPoints[0].position.slice();
            else this.target = [0, 0, 0];
        }
        if (!this._marker) {
            this._marker = new THREE.Mesh(
                new THREE.SphereGeometry(0.12, 12, 8),
                new THREE.MeshBasicMaterial({ color: 0xffb347, transparent: true, opacity: 0.85 }),
            );
            scene._frameRotate.add(this._marker);
        }
        this._marker.position.set(...this.target);
        this._marker.visible = true;
        this.active = true;
        this.apply(scene);
    }

    disable() {
        this.active = false;
        if (this._marker) this._marker.visible = false;
    }

    /** One wheel notch: in when `deltaY < 0`, out otherwise. */
    zoom(deltaY, scene) {
        const step = deltaY < 0 ? 1 / ORBIT_ZOOM_STEP : ORBIT_ZOOM_STEP;
        this.distance = Math.max(ORBIT_MIN_M, Math.min(ORBIT_MAX_M, this.distance * step));
        this.apply(scene);
    }

    /** Put the target `distance` metres straight ahead of the eye, along the current look direction. */
    apply(scene) {
        if (!this.active || !this.target) return;
        const scale = scene._worldGroup.scale.x;
        const local = robotToWorldOffset(scene._worldGroup, this.target);
        const head = scene.getCameraPositionWorld();
        const fwd = new THREE.Vector3();
        scene.camera.getWorldDirection(fwd);
        const d = this.distance * scale;
        scene._worldGroup.position.set(
            head.x + fwd.x * d - local.x,
            head.y + fwd.y * d - local.y,
            head.z + fwd.z * d - local.z,
        );
    }
}
