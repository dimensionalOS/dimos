// Between the robot's frame and the world the viewer stands in.
//
// The scene hangs everything the recording contains under `worldGroup > frameRotate`:
// `frameRotate` is R_x(-pi/2), which maps robot (x, y, z) to three (x, z, -y), and
// `worldGroup` then scales it, spins it about Y and moves it. Walking, flying and
// orbiting all work by moving `worldGroup` so that some point of the recording lands
// where the viewer should see it, which needs the forward map; the HUD needle and the
// server's idea of where the viewer is standing need the inverse.
//
// These lived as four private methods on Scene and three open-coded copies elsewhere,
// and the copies were the ones missing the spin.

import * as THREE from 'https://esm.sh/three@0.160.0';

/** R_y(rotation.y) applied to a worldGroup-local vector. x' = c*x + s*z, z' = -s*x + c*z. */
function spinIntoWorld(worldGroup, local) {
    const spin = worldGroup.rotation.y;
    const c = Math.cos(spin);
    const sn = Math.sin(spin);
    return new THREE.Vector3(c * local.x + sn * local.z, local.y, -sn * local.x + c * local.z);
}

/** Where three actually DRAWS a robot-frame point, as an offset from `worldGroup.position`.
 *
 *  three draws a child of worldGroup at `position + R_y(rotation.y) . local`, so anything
 *  that PLACES the world by subtracting a point's offset has to subtract the ROTATED one.
 *  Subtracting the bare `frameRotate(p) * scale` is right only at rotation.y === 0, and the
 *  right stick turns that every frame in VR: 1.637 m of error at 30 degrees, 4.472 at 90 and
 *  6.325 at 180 for a point framed 2.5 m ahead, measured against three's own matrixWorld.
 *  `resetView()` is the only thing that puts rotation.y back to 0, which is why a call site
 *  without this reads correct until someone turns and never reads correct again.
 */
export function robotToWorldOffset(worldGroup, position) {
    const local = new THREE.Vector3(position[0], position[2], -position[1])
        .multiplyScalar(worldGroup.scale.x);
    return spinIntoWorld(worldGroup, local);
}

/** A robot-frame DIRECTION in world space: the same spin, no scale and no offset. */
export function robotToWorldDir(worldGroup, direction) {
    return spinIntoWorld(
        worldGroup,
        new THREE.Vector3(direction[0], direction[2], -direction[1]),
    );
}

/** A world-space DIRECTION as robot (x, y). Position and scale drop out of a direction,
 *  so the HUD needle needs only this -- and needed it: it read the heading straight off
 *  the world-space forward, which is the robot's heading turned by `rotation.y`. The dot
 *  beside it goes through `worldPosToRobotXY` and does account for the spin, so after a
 *  turn the needle pointed one way while the dot it sits on had moved another. */
export function worldDirToRobotXY(worldGroup, dir) {
    // The inverse rotation, which for R_y is its transpose: x = c*x' - s*z', z = s*x' + c*z'.
    const c = Math.cos(worldGroup.rotation.y);
    const sn = Math.sin(worldGroup.rotation.y);
    // Un-apply frame-rotate: three (x, y, z) -> robot (x, -z, y).
    return [c * dir.x - sn * dir.z, -(sn * dir.x + c * dir.z)];
}

/** The inverse of `robotToWorldOffset`, as robot (x, y) -- the two the map needs. */
export function worldPosToRobotXY(worldGroup, worldPos) {
    const s = worldGroup.scale.x || 1;
    return worldDirToRobotXY(worldGroup, {
        x: (worldPos.x - worldGroup.position.x) / s,
        z: (worldPos.z - worldGroup.position.z) / s,
    });
}

/** The desktop camera's {yaw, pitch} for looking along a robot-frame `forward`.
 *
 *  `camera.rotation` is read in WORLD space, so the direction has to be spun into it
 *  first -- which is the whole of the difference, and it is exactly the world's current
 *  turn angle. Three call sites open-coded `atan2(-f[0], f[1])` off the raw robot
 *  forward; two of them (results.js's `_firstEvidence` and `_yawFromEvidence`, which is
 *  what Next, Prev and Navigate fly with) were still doing it, and landed the viewer
 *  facing 30 degrees wrong at a 30 degree spin, 90 at 90, 180 at 180.
 *
 *  Pitch needs no spin: robot z IS three y, and a turn about Y does not touch it.
 */
export function desktopLookAngles(worldGroup, forward) {
    const spun = robotToWorldDir(worldGroup, forward);
    return {
        // The desktop camera looks down -z at yaw 0; pitch is positive looking up.
        yaw: Math.atan2(-spun.x, -spun.z),
        pitch: Math.asin(Math.max(-1, Math.min(1, spun.y))),
    };
}
