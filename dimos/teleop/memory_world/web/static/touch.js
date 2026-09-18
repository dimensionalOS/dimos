// Phone and tablet input for the desktop (non-WebXR) view: one finger looks,
// two fingers walk and pinch to scale. Split out of scene.js, which sits against
// the 75 KB pre-commit ceiling; it owns no state of its own beyond the stick it
// writes back to the scene each move.

const TOUCH_LOOK_SENSITIVITY = 0.006;         // radians per CSS pixel of one-finger drag
const TOUCH_WALK_GAIN = 40;                   // two-finger drag: a screen-height sweep = full stick x40
// Exported, not copied: scene.js clamps mouse-look with this same number, and two
// spellings of it would let touch-look and mouse-look stop at different pitches.
export const DESKTOP_PITCH_LIMIT = 1.45;   // just under 90deg, avoids gimbal flip

export function installTouch(scene, dom) {
    dom.style.touchAction = 'none';
    scene._touchStick = { x: 0, y: 0 };
    let last = null;   // {x, y} of one finger, or {x, y, spread} of two
    const centre = (touches) => {
        const points = Array.from(touches);
        const x = points.reduce((sum, t) => sum + t.clientX, 0) / points.length;
        const y = points.reduce((sum, t) => sum + t.clientY, 0) / points.length;
        const spread = points.length > 1
            ? Math.hypot(points[0].clientX - points[1].clientX, points[0].clientY - points[1].clientY)
            : 0;
        return { x, y, spread, count: points.length };
    };
    // `targetTouches`, not `touches`: `touches` is every finger on the SCREEN, and the
    // walk stick is a sibling of this canvas on the same page. A finger parked on the
    // stick made a one-finger look drag on the canvas count as two, so look went dead,
    // a phantom stick was written on top of the real one, and the world silently
    // rescaled on a "pinch" that was one finger and a joystick. `main.js` already uses
    // targetTouches for the stick, with a comment about exactly this.
    const begin = (event) => { last = centre(event.targetTouches); };
    dom.addEventListener('touchstart', (event) => { event.preventDefault(); begin(event); }, { passive: false });
    dom.addEventListener('touchmove', (event) => {
        event.preventDefault();
        const now = centre(event.targetTouches);
        if (!last || last.count !== now.count) { last = now; return; }
        if (now.count === 1) {
            scene._desktopYaw -= (now.x - last.x) * TOUCH_LOOK_SENSITIVITY;
            scene._desktopPitch -= (now.y - last.y) * TOUCH_LOOK_SENSITIVITY;
            scene._desktopPitch = Math.max(-DESKTOP_PITCH_LIMIT, Math.min(DESKTOP_PITCH_LIMIT, scene._desktopPitch));
            scene.camera.rotation.set(scene._desktopPitch, scene._desktopYaw, 0);
        } else {
            // Two fingers: drag walks (up = forward), pinch scales the world.
            const height = dom.clientHeight || 1;
            scene._touchStick = {
                x: Math.max(-1, Math.min(1, (now.x - last.x) / height * TOUCH_WALK_GAIN)),
                y: Math.max(-1, Math.min(1, (now.y - last.y) / height * TOUCH_WALK_GAIN)),
            };
            if (last.spread > 0 && now.spread > 0) scene.applyScale({ factor: now.spread / last.spread });
        }
        last = now;
    }, { passive: false });
    const end = (event) => {
        scene._touchStick = { x: 0, y: 0 };
        last = event.targetTouches.length ? centre(event.targetTouches) : null;
    };
    dom.addEventListener('touchend', end);
    dom.addEventListener('touchcancel', end);
}
