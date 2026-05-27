// Input gestures for VR World (live).
//
// Differs from memory_world: the LEFT stick DRIVES THE ROBOT (Twist) instead of
// walking the viewer. View navigation is on the right hand + grips.
//
// Emitted gestures:
//   { type: 'drive', x, yaw }                 every frame, left stick (x=fwd/back, yaw=turn)
//   { type: 'yaw', rate }                     every frame, right stick X — yaw the god-view
//   { type: 'teleport_aim'/'teleport_commit' } right trigger
//   { type: 'scale_delta', factor, pivotWorld } both grips/pinches, move apart/together
//   { type: 'reset_view' }                    left Y
//   { type: 'toggle_render' }                 right B

const STICK_DEADZONE = 0.15;
const YAW_RATE_MAX = Math.PI / 2;
const TRIGGER_PRESS = 0.5;
const PINCH_DISTANCE_M = 0.025, PINCH_RELEASE_M = 0.040;
const SCALE_RATE = 1.0;

export class InputAdapter {
    constructor(onGesture) {
        this.onGesture = onGesture;
        this._hand = {
            left: { pinching: false, wasPinching: false, pos: null },
            right: { pinching: false, wasPinching: false, pos: null },
        };
        this._scaleAnchor = null;
        this._teleportArmed = false;
        this._leftYWas = false;
        this._rightBWas = false;
    }

    onFrame(frame, xrRefSpace, nowMs, scene) {
        let driveX = 0, driveYaw = 0, yawRate = 0;
        let sawLeft = false, sawRight = false;

        for (const src of frame.session.inputSources) {
            const hand = src.handedness;
            if (hand !== 'left' && hand !== 'right') continue;

            // Hand-tracking path (pinch -> scale), only when no gamepad held.
            if (src.hand && src.hand.size > 0 && !src.gamepad) {
                this._processHand(hand, frame, xrRefSpace, src.hand);
                continue;
            }
            const gp = src.gamepad;
            if (!gp) continue;
            const ax = gp.axes[2] ?? 0, ay = gp.axes[3] ?? 0;

            // Grip position for controller-grip scaling.
            const gripSpace = src.gripSpace || src.targetRaySpace;
            const gripPose = gripSpace ? frame.getPose(gripSpace, xrRefSpace) : null;
            if (gripPose) {
                const p = gripPose.transform.position;
                this._hand[hand].pos = [p.x, p.y, p.z];
                this._hand[hand].pinching = (gp.buttons[1]?.value ?? 0) > 0.5;
                this._hand[hand].wasPinching = this._hand[hand].pinching;
            }

            if (hand === 'left') {
                sawLeft = true;
                // Drive the robot: stick up (ay<0) = forward; stick X = turn.
                driveX = Math.abs(ay) > STICK_DEADZONE ? -ay : 0;
                driveYaw = Math.abs(ax) > STICK_DEADZONE ? -ax : 0;  // right stick-x → turn right
                const yBtn = gp.buttons[5]?.pressed ?? false;
                if (yBtn && !this._leftYWas) this.onGesture({ type: 'reset_view' });
                this._leftYWas = yBtn;
            } else {
                sawRight = true;
                if (Math.abs(ax) > STICK_DEADZONE) {
                    const sign = ax >= 0 ? 1 : -1;
                    const mag = (Math.abs(ax) - STICK_DEADZONE) / (1 - STICK_DEADZONE);
                    yawRate = sign * mag * YAW_RATE_MAX;
                }
                const bBtn = gp.buttons[5]?.pressed ?? false;
                if (bBtn && !this._rightBWas) this.onGesture({ type: 'toggle_render' });
                this._rightBWas = bBtn;
                this._handleTeleport(src, frame, xrRefSpace, gp.buttons[0]?.value ?? 0);
            }
        }

        if (sawLeft) this.onGesture({ type: 'drive', x: driveX, yaw: driveYaw });
        if (sawRight) this.onGesture({ type: 'yaw', rate: yawRate });
        this._updateScale();
    }

    _handleTeleport(src, frame, xrRefSpace, triggerVal) {
        const rayPose = src.targetRaySpace ? frame.getPose(src.targetRaySpace, xrRefSpace) : null;
        if (triggerVal > TRIGGER_PRESS) {
            if (!rayPose) return;
            const p = rayPose.transform.position, o = rayPose.transform.orientation;
            const fx = -2 * (o.x * o.z + o.w * o.y);
            const fy = -2 * (o.y * o.z - o.w * o.x);
            const fz = -(1 - 2 * (o.x * o.x + o.y * o.y));
            this._teleportArmed = true;
            this.onGesture({ type: 'teleport_aim', originWorld: [p.x, p.y, p.z], dirWorld: [fx, fy, fz] });
        } else if (this._teleportArmed) {
            this._teleportArmed = false;
            this.onGesture({ type: 'teleport_commit' });
        }
    }

    _processHand(hand, frame, xrRefSpace, joints) {
        const thumb = joints.get('thumb-tip'), index = joints.get('index-finger-tip');
        if (!thumb || !index) return;
        const tp = frame.getJointPose(thumb, xrRefSpace), ip = frame.getJointPose(index, xrRefSpace);
        if (!tp || !ip) return;
        const tx = tp.transform.position, ix = ip.transform.position;
        const dist = Math.hypot(tx.x - ix.x, tx.y - ix.y, tx.z - ix.z);
        const st = this._hand[hand];
        st.pinching = st.wasPinching ? dist < PINCH_RELEASE_M : dist < PINCH_DISTANCE_M;
        st.wasPinching = st.pinching;
        st.pos = [(tx.x + ix.x) / 2, (tx.y + ix.y) / 2, (tx.z + ix.z) / 2];
    }

    _updateScale() {
        const L = this._hand.left, R = this._hand.right;
        if (L.pinching && R.pinching && L.pos && R.pos) {
            const dist = Math.hypot(L.pos[0] - R.pos[0], L.pos[1] - R.pos[1], L.pos[2] - R.pos[2]);
            const mid = [(L.pos[0] + R.pos[0]) / 2, (L.pos[1] + R.pos[1]) / 2, (L.pos[2] + R.pos[2]) / 2];
            if (this._scaleAnchor === null) {
                this._scaleAnchor = { dist };
            } else {
                const ratio = dist / Math.max(this._scaleAnchor.dist, 1e-3);
                const step = 1.0 + (ratio - 1.0) * SCALE_RATE;
                if (Math.abs(step - 1.0) > 0.005) {
                    this.onGesture({ type: 'scale_delta', factor: step, pivotWorld: mid });
                    this._scaleAnchor = { dist };
                }
            }
        } else {
            this._scaleAnchor = null;
        }
    }
}
