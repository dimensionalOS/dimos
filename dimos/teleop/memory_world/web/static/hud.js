// The head-up panel: where it hangs relative to the head, and the little map on it.
//
// Lifted out of scene.js when that file hit the repo's 75 KB file limit. It is the
// largest piece of it that only reads the scene rather than being read by it, so it is
// the one that moves without dragging anything behind it. Same shape as evidence.js:
// a function over the scene, not a class.
import * as THREE from 'https://esm.sh/three@0.160.0';

export const HUD_PANEL_SIZE = 0.22;          // metres (square)
const HUD_DISTANCE = 0.55;            // metres in front of head
const HUD_OFFSET_DOWN = 0.25;
const HUD_OFFSET_LEFT = 0.32;
const HUD_FOLLOW_LERP = 0.18;         // damping per frame
export const ANSWER_PANEL_W = 0.62;          // metres; the canvas behind it is 4:1

/** Put the HUD where the head can see it, and move its map marker to the viewer. */
export function placeHud(scene) {
    // Place the HUD panel relative to the head: forward + down + left in
    // the head's yaw frame, kept upright (pitch ignored) so it doesn't
    // tumble when the user looks up.
    const cam = scene.three.xr.isPresenting
        ? scene.three.xr.getCamera(scene.camera)
        : scene.camera;
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
    if (!scene.three.xr.isPresenting) {
        const halfHeight = HUD_DISTANCE * Math.tan(THREE.MathUtils.degToRad(scene.camera.fov) / 2);
        const halfWidth = halfHeight * scene.camera.aspect;
        // The panel is turned toward the head, so its near edge projects wider than flat: keep a fat margin.
        offsetLeft = Math.max(0, Math.min(HUD_OFFSET_LEFT, halfWidth - ANSWER_PANEL_W / 2 - 0.12));
        // A portrait phone is narrower than the panel itself: shrink it to fit.
        const fit = Math.min(1, (2 * halfWidth - 0.08) / ANSWER_PANEL_W);
        scene._answerPanel.scale.setScalar(fit);
        scene._cameraPanel.scale.setScalar(fit);
    } else if (scene._answerPanel.scale.x !== 1) {
        scene._answerPanel.scale.setScalar(1);
        scene._cameraPanel.scale.setScalar(1);
    }
    const target = new THREE.Vector3()
        .copy(headPos)
        .addScaledVector(fwd, HUD_DISTANCE)
        .addScaledVector(right, -offsetLeft);
    target.y -= HUD_OFFSET_DOWN;
    // Tilt the panel slightly toward the user (downward tilt around X).
    scene._hudGroup.position.lerp(target, HUD_FOLLOW_LERP);
    // Face the user — look at head from panel position, then tilt up a bit.
    scene._hudGroup.lookAt(headPos);

    // Update marker dot position to where the camera is *in world*.
    // We need the camera's robot-frame XY. Camera is at headPos in three-world;
    // un-apply worldGroup transform + frameRotate to get robot frame.
    if (scene._topDownBounds) {
        const robotXY = scene._worldPosToRobotXY(headPos);
        if (robotXY) {
            const uv = scene._robotXYToHudUV(robotXY[0], robotXY[1]);
            // Panel is HUD_PANEL_SIZE wide centred at (0,0). Map u,v in [0,1]
            // to [-S/2, S/2].
            const s = HUD_PANEL_SIZE;
            scene._hudMarker.position.x = (uv[0] - 0.5) * s;
            scene._hudMarker.position.y = (0.5 - uv[1]) * s;
            scene._hudHeading.position.copy(scene._hudMarker.position);
            // The needle wants yaw in the map frame, not three's. After the
            // frame-rotate, robot +X is three +X and robot +Y is three -Z.
            const robotYaw = Math.atan2(fwd.x, -fwd.z);
            scene._hudHeading.rotation.z = -robotYaw;
        }
    }
}
