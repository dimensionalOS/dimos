// The head-up panel: where it hangs relative to the head, and the little map on it.
//
// Lifted out of scene.js when that file hit the repo's 75 KB file limit. It is the
// largest piece of it that only reads the scene rather than being read by it, so it is
// the one that moves without dragging anything behind it. Same shape as evidence.js:
// a function over the scene, not a class.
import * as THREE from 'https://esm.sh/three@0.160.0';
import { worldDirToRobotXY, worldPosToRobotXY } from '/static_mw/world_frame.js';

export const HUD_PANEL_SIZE = 0.22;          // metres (square)
const HUD_DISTANCE = 0.55;            // metres in front of head
const HUD_OFFSET_DOWN = 0.25;
const HUD_OFFSET_LEFT = 0.32;
const HUD_FOLLOW_LERP = 0.18;         // damping per frame
export const ANSWER_PANEL_W = 0.62;          // metres; the canvas behind it is 4:1
// How close to the edge of the view the panel's centre may be dragged. The panel is
// wider than it is tall, so this is the half-width that has to stay inside.
const HUD_EDGE_MARGIN = ANSWER_PANEL_W / 2;

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
    // Where the user dragged it to, if they have; the computed corner until then, so
    // the desktop auto-fit above still owns the position nobody has chosen.
    const placed = scene._hudOffset;
    const target = new THREE.Vector3()
        .copy(headPos)
        .addScaledVector(fwd, HUD_DISTANCE)
        .addScaledVector(right, placed ? placed.right : -offsetLeft);
    target.y -= placed ? placed.down : HUD_OFFSET_DOWN;
    // Tilt the panel slightly toward the user (downward tilt around X).
    // No damping mid-drag: at 0.18 the panel trails the cursor by enough that it reads
    // as the drag slipping rather than as smoothing.
    scene._hudGroup.position.lerp(target, scene._hudDragging ? 1 : HUD_FOLLOW_LERP);
    // Face the user — look at head from panel position, then tilt up a bit.
    scene._hudGroup.lookAt(headPos);

    // Update marker dot position to where the camera is *in world*.
    // We need the camera's robot-frame XY. Camera is at headPos in three-world;
    // un-apply worldGroup transform + frameRotate to get robot frame.
    if (scene._topDownBounds) {
        const robotXY = worldPosToRobotXY(scene._worldGroup, headPos);
        if (robotXY) {
            const uv = scene._robotXYToHudUV(robotXY[0], robotXY[1]);
            // Panel is HUD_PANEL_SIZE wide centred at (0,0). Map u,v in [0,1]
            // to [-S/2, S/2].
            const s = HUD_PANEL_SIZE;
            scene._hudMarker.position.x = (uv[0] - 0.5) * s;
            scene._hudMarker.position.y = (0.5 - uv[1]) * s;
            scene._hudHeading.position.copy(scene._hudMarker.position);
            // The needle wants yaw in the MAP's frame, not three's, and the two differ
            // by the world spin -- which is why this used to read `atan2(fwd.x, -fwd.z)`
            // straight off the world forward and point somewhere else than the dot it
            // sits on the moment anyone turned. That expression is what this returns at
            // rotation.y === 0.
            const [fwdRx, fwdRy] = worldDirToRobotXY(scene._worldGroup, fwd);
            const robotYaw = Math.atan2(fwdRx, fwdRy);
            scene._hudHeading.rotation.z = -robotYaw;
        }
    }
}


/** Half the view's width and height in metres at the distance the HUD hangs at. */
function viewHalfExtents(scene) {
    const halfHeight = HUD_DISTANCE * Math.tan(THREE.MathUtils.degToRad(scene.camera.fov) / 2);
    return [halfHeight * scene.camera.aspect, halfHeight];
}

/** Let the pointer pick the HUD up and put it somewhere else. Desktop only.
 *
 * The panel is head-locked 3D, not a DOM element, so "where it is" is an offset in the
 * head's yaw frame and a drag has to be measured in those metres rather than in pixels.
 * At HUD_DISTANCE the view is `2 * halfHeight` metres tall and `clientHeight` pixels
 * tall, and pixels are square, so one ratio converts both axes.
 *
 * The listeners go on `window` in the CAPTURE phase because the canvas already turns a
 * mousedown into look-drag; capturing on an ancestor is what runs first and lets
 * `stopPropagation` keep the world still while the panel moves. Registering on the
 * canvas instead would not have worked -- listeners on the same element fire in
 * registration order whatever their capture flag says, and look-drag is registered first.
 */
export function installHudDrag(scene, dom, signal) {
    const raycaster = new THREE.Raycaster();
    let last = null;

    const panelUnder = (event) => {
        if (scene.three.xr.isPresenting || !scene._hudGroup.visible) return false;
        const box = dom.getBoundingClientRect();
        raycaster.setFromCamera(
            new THREE.Vector2(
                ((event.clientX - box.left) / box.width) * 2 - 1,
                -((event.clientY - box.top) / box.height) * 2 + 1,
            ),
            scene.camera,
        );
        // The whole group, not just the map: the answer panel and the camera view hang
        // off the same corner, and picking one up should bring its neighbours.
        return raycaster.intersectObject(scene._hudGroup, true).length > 0;
    };

    const stop = (event) => {
        event.preventDefault();
        event.stopPropagation();
    };

    window.addEventListener('mousedown', (event) => {
        if (event.button !== 0 || !panelUnder(event)) return;
        // Start from where it IS, so the first drag does not jump it from the computed
        // corner to whatever the defaults say.
        const local = scene._hudGroup.position.clone().sub(scene.camera.position);
        const right = new THREE.Vector3().setFromMatrixColumn(scene.camera.matrixWorld, 0);
        right.y = 0;
        if (right.lengthSq() > 1e-6) right.normalize();
        scene._hudOffset = scene._hudOffset || { right: local.dot(right), down: -local.y };
        scene._hudDragging = true;
        last = { x: event.clientX, y: event.clientY };
        dom.style.cursor = 'move';
        stop(event);
    }, { capture: true, signal });

    window.addEventListener('mousemove', (event) => {
        if (!scene._hudDragging) return;
        const [halfWidth, halfHeight] = viewHalfExtents(scene);
        const metresPerPixel = (2 * halfHeight) / (dom.clientHeight || 1);
        const clamp = (v, limit) => Math.max(-limit, Math.min(limit, v));
        scene._hudOffset.right = clamp(
            scene._hudOffset.right + (event.clientX - last.x) * metresPerPixel,
            Math.max(0, halfWidth - HUD_EDGE_MARGIN),
        );
        scene._hudOffset.down = clamp(
            scene._hudOffset.down + (event.clientY - last.y) * metresPerPixel,
            Math.max(0, halfHeight - HUD_PANEL_SIZE / 2),
        );
        last = { x: event.clientX, y: event.clientY };
        stop(event);
    }, { capture: true, signal });

    window.addEventListener('mouseup', (event) => {
        if (!scene._hudDragging) return;
        scene._hudDragging = false;
        last = null;
        dom.style.cursor = 'grab';
        stop(event);
    }, { capture: true, signal });

    // Put it back. A drag has no other undo, and the canvas reads a double-click as
    // "give me pointer lock", which over the panel is never what was meant.
    window.addEventListener('dblclick', (event) => {
        if (!panelUnder(event)) return;
        scene._hudOffset = null;
        stop(event);
    }, { capture: true, signal });
}
