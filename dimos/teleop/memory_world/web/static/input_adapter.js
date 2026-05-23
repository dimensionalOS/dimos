// Gesture primitives for the memory-world VR walkthrough.
//
// Emitted gestures (passed to the `onGesture` callback):
//   { type: 'locomote',         stickX, stickY }                    // every frame, left controller stick
//   { type: 'yaw',              rate }                              // every frame, right controller stick X (rad/s)
//   { type: 'teleport_aim',     originWorld:[x,y,z], dirWorld:[x,y,z] }  // while right trigger held
//   { type: 'teleport_commit' }                                     // on right trigger release after aim
//   { type: 'scale_delta',      factor, pivotWorld:[x,y,z] }        // while both hands pinching
//   { type: 'reset_view' }                                          // Y button on left controller
//   { type: 'toggle_images' }                                       // X button on left controller
//   { type: 'toggle_render' }                                       // B button on right controller (cubes <-> points)
//
// Quest 3 gamepad button mapping (per WebXR spec / Meta docs):
//   buttons[0] = trigger
//   buttons[1] = grip
//   buttons[3] = thumbstick click
//   buttons[4] = A (right) / X (left)
//   buttons[5] = B (right) / Y (left)
//   axes[2,3]  = thumbstick X / Y

const STICK_DEADZONE = 0.18;
const YAW_RATE_MAX_RAD_PER_S = Math.PI / 2;    // 90°/s at full deflection
const TRIGGER_PRESS = 0.5;
const PINCH_DISTANCE_M = 0.025;        // 25 mm — WebXR-recommended pinch threshold
const PINCH_RELEASE_M = 0.040;
const SCALE_RATE = 1.0;                // factor change rate per metre of pinch-spread delta

export class InputAdapter {
    constructor(onGesture) {
        this.onGesture = onGesture;
        // Per-hand pinch state for scaling.
        this._hand = {
            left: { pinching: false, wasPinching: false, pos: null },
            right: { pinching: false, wasPinching: false, pos: null },
        };
        // Inter-hand reference distance captured when the second pinch begins.
        this._scaleAnchor = null;
        // Right-controller trigger state for teleport.
        this._teleportArmed = false;
    }

    onFrame(frame, xrRefSpace, nowMs, scene) {
        let stickX = 0, stickY = 0;
        let yawRate = 0;
        let sawLeftController = false;
        let sawRightController = false;

        for (const inputSource of frame.session.inputSources) {
            const hand = inputSource.handedness;
            if (hand !== 'left' && hand !== 'right') continue;

            // Hand-tracking branch (for pinch -> scale). Skip when a gamepad
            // is also exposed on this input source — that means the controller
            // is being held, so we want the grip-button path, not joints.
            if (inputSource.hand && inputSource.hand.size > 0 && !inputSource.gamepad) {
                this._processHandTracked(hand, frame, xrRefSpace, inputSource.hand);
                continue;
            }

            // Controller branch.
            const gp = inputSource.gamepad;
            if (!gp) continue;
            const axisX = gp.axes[2] ?? 0;
            const axisY = gp.axes[3] ?? 0;
            const triggerVal = gp.buttons[0]?.value ?? 0;
            const gripVal = gp.buttons[1]?.value ?? 0;
            const yButton = gp.buttons[5]?.pressed ?? false;

            // Track controller world position for grip-based scaling.
            const gripSpace = inputSource.gripSpace || inputSource.targetRaySpace;
            const gripPose = gripSpace ? frame.getPose(gripSpace, xrRefSpace) : null;
            if (gripPose) {
                const p = gripPose.transform.position;
                this._hand[hand].pos = [p.x, p.y, p.z];
                this._hand[hand].pinching = gripVal > 0.5;
                this._hand[hand].wasPinching = this._hand[hand].pinching;
            }

            if (hand === 'left') {
                sawLeftController = true;
                stickX = Math.abs(axisX) > STICK_DEADZONE ? axisX : 0;
                stickY = Math.abs(axisY) > STICK_DEADZONE ? axisY : 0;
                // Left Y button = reset view (press edge).
                if (yButton && !this._leftYWas) this.onGesture({ type: 'reset_view' });
                this._leftYWas = yButton;
                // Left X button (index 4) = toggle image thumbnails (press edge).
                const xButton = gp.buttons[4]?.pressed ?? false;
                if (xButton && !this._leftXWas) this.onGesture({ type: 'toggle_images' });
                this._leftXWas = xButton;
            } else if (hand === 'right') {
                sawRightController = true;
                // Smooth yaw: proportional to stick X with a deadzone.
                if (Math.abs(axisX) > STICK_DEADZONE) {
                    // Re-scale past the deadzone so small motion stays smooth.
                    const sign = axisX >= 0 ? 1 : -1;
                    const mag = (Math.abs(axisX) - STICK_DEADZONE) / (1 - STICK_DEADZONE);
                    yawRate = sign * mag * YAW_RATE_MAX_RAD_PER_S;
                }
                // Right B button (index 5) = toggle cubes <-> points (press edge).
                const bButton = gp.buttons[5]?.pressed ?? false;
                if (bButton && !this._rightBWas) this.onGesture({ type: 'toggle_render' });
                this._rightBWas = bButton;
                // Teleport: aim while trigger held, commit on release.
                this._handleTriggerTeleport(inputSource, frame, xrRefSpace, triggerVal, scene);
            }
        }

        // Always emit a locomote (zero stick = stop). Cheap text packet.
        if (sawLeftController) {
            this.onGesture({ type: 'locomote', stickX, stickY });
        }
        if (sawRightController) {
            this.onGesture({ type: 'yaw', rate: yawRate });
        }

        // Bimanual-pinch scaling.
        this._updateBimanualScale(scene);
    }

    _handleTriggerTeleport(inputSource, frame, xrRefSpace, triggerVal, scene) {
        const rayPose = inputSource.targetRaySpace
            ? frame.getPose(inputSource.targetRaySpace, xrRefSpace)
            : null;
        if (triggerVal > TRIGGER_PRESS) {
            // Held — emit an aim each frame.
            if (!rayPose) return;
            const p = rayPose.transform.position;
            const o = rayPose.transform.orientation;
            // Controller forward is -Z in its local frame; rotate (0,0,-1) by q.
            const fx = -2 * (o.x * o.z + o.w * o.y);
            const fy = -2 * (o.y * o.z - o.w * o.x);
            const fz = -(1 - 2 * (o.x * o.x + o.y * o.y));
            this._teleportArmed = true;
            this.onGesture({
                type: 'teleport_aim',
                originWorld: [p.x, p.y, p.z],
                dirWorld: [fx, fy, fz],
            });
        } else if (this._teleportArmed) {
            // Released — commit.
            this._teleportArmed = false;
            this.onGesture({ type: 'teleport_commit' });
        }
    }

    _processHandTracked(hand, frame, xrRefSpace, joints) {
        const wrist = joints.get('wrist');
        const thumb = joints.get('thumb-tip');
        const index = joints.get('index-finger-tip');
        if (!wrist || !thumb || !index) return;

        const wristPose = frame.getJointPose(wrist, xrRefSpace);
        const thumbPose = frame.getJointPose(thumb, xrRefSpace);
        const indexPose = frame.getJointPose(index, xrRefSpace);
        if (!wristPose || !thumbPose || !indexPose) return;

        const tx = thumbPose.transform.position;
        const ix = indexPose.transform.position;
        const dx = tx.x - ix.x, dy = tx.y - ix.y, dz = tx.z - ix.z;
        const dist = Math.hypot(dx, dy, dz);

        const st = this._hand[hand];
        if (st.wasPinching) {
            st.pinching = dist < PINCH_RELEASE_M;
        } else {
            st.pinching = dist < PINCH_DISTANCE_M;
        }
        st.wasPinching = st.pinching;
        // Midpoint between thumb and index is the natural pinch position.
        st.pos = [(tx.x + ix.x) / 2, (tx.y + ix.y) / 2, (tx.z + ix.z) / 2];
    }

    _updateBimanualScale(scene) {
        const L = this._hand.left, R = this._hand.right;
        const bothPinching = L.pinching && R.pinching && L.pos && R.pos;

        if (bothPinching) {
            const dist = Math.hypot(
                L.pos[0] - R.pos[0],
                L.pos[1] - R.pos[1],
                L.pos[2] - R.pos[2],
            );
            const mid = [
                (L.pos[0] + R.pos[0]) / 2,
                (L.pos[1] + R.pos[1]) / 2,
                (L.pos[2] + R.pos[2]) / 2,
            ];
            if (this._scaleAnchor === null) {
                this._scaleAnchor = { dist, mid };
            } else {
                // factor relative to anchor — fingers further apart = world grows.
                const ratio = dist / Math.max(this._scaleAnchor.dist, 1e-3);
                // Apply incrementally: only the per-frame delta. Then re-anchor.
                const stepFactor = 1.0 + (ratio - 1.0) * SCALE_RATE;
                if (Math.abs(stepFactor - 1.0) > 0.005) {
                    this.onGesture({
                        type: 'scale_delta',
                        factor: stepFactor,
                        pivotWorld: mid,
                    });
                    // Re-anchor so the next frame measures from current state.
                    this._scaleAnchor = { dist, mid };
                }
            }
        } else {
            this._scaleAnchor = null;
        }
    }
}
