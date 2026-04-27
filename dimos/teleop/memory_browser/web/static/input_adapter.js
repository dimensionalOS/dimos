// Gesture primitives extracted from raw WebXR input.
//
// v1 (this file): controllers via `inputSource.gripSpace` + gamepad. Emits
// the same gesture events that v2 (XRHand joints) will eventually emit, so
// the rest of the app doesn't care which one is feeding it.
//
// Emitted gestures (passed to the `onGesture` callback):
//   { type: 'engage' }                                    grip pressed
//   { type: 'disengage' }                                 grip released
//   { type: 'palm_roll_delta', value: <radians signed> }  while engaged
//   { type: 'swipe', hand: 'left'|'right', value: <m/s> } sustained lateral burst

const SWIPE_VELOCITY_THRESHOLD = 1.5;          // m/s — minimum horizontal speed
const SWIPE_AXIS_RATIO = 1.5;                  // |vx| must dominate |vy|+|vz| by this much
const SWIPE_DEBOUNCE_MS = 500;
const VELOCITY_BUFFER_SIZE = 5;                // rolling window for velocity smoothing
const GRIP_THRESHOLD = 0.5;                    // analog grip press cutoff
// Hand-tracking: pinch = thumb-tip ↔ index-tip distance below this threshold.
// 25 mm matches the WebXR spec's recommended pinch threshold.
const PINCH_DISTANCE_M = 0.025;
const PINCH_RELEASE_M = 0.040;                 // hysteresis so flicker at 25mm doesn't toggle

export class InputAdapter {
    constructor(onGesture) {
        this.onGesture = onGesture;
        // Per-hand state. The right hand drives roll; either hand can swipe.
        this._state = {
            left: this._freshHandState(),
            right: this._freshHandState(),
        };
    }

    _freshHandState() {
        return {
            engaged: false,
            // Forward-axis reference captured at engage time, used to measure roll.
            engagedForward: null,  // unit Vector3 in ref space
            engagedUp: null,
            lastRoll: 0,           // radians, integrated since engage
            posBuffer: [],         // [{t, p:[x,y,z]}] for velocity
            lastSwipeMs: 0,
            // Tracks whether the right hand was last seen pinching, used for
            // hysteresis so micro-jitter at the threshold doesn't toggle
            // engage/disengage every frame.
            wasPinching: false,
        };
    }

    onFrame(frame, xrRefSpace, nowMs) {
        let seenLeft = false, seenRight = false;
        let usedHands = false;
        for (const inputSource of frame.session.inputSources) {
            const handedness = inputSource.handedness;
            if (handedness !== 'left' && handedness !== 'right') continue;

            // Prefer hand tracking when joints are exposed for this source.
            if (inputSource.hand && inputSource.hand.size > 0) {
                if (this._processHandTracked(handedness, frame, xrRefSpace, inputSource.hand, nowMs)) {
                    if (handedness === 'left') seenLeft = true;
                    if (handedness === 'right') seenRight = true;
                    usedHands = true;
                    continue;
                }
            }

            // Fall back to controller (gripSpace + gamepad button[1] = grip).
            const grip = inputSource.gripSpace || inputSource.targetRaySpace;
            if (!grip) continue;
            const pose = frame.getPose(grip, xrRefSpace);
            if (!pose) continue;

            if (handedness === 'left') seenLeft = true;
            if (handedness === 'right') seenRight = true;

            const gp = inputSource.gamepad;
            const gripVal = gp ? (gp.buttons[1]?.value ?? 0) : 0;
            this._processHand(handedness, pose, gripVal, nowMs);
        }

        if (!this._seenAnyController && (seenLeft || seenRight)) {
            this._seenAnyController = true;
            if (typeof window !== 'undefined' && window.app && window.app.diag) {
                window.app.diag('first_input', { left: seenLeft, right: seenRight, hands: usedHands });
            }
        }
        if (usedHands && !this._seenHandTracking) {
            this._seenHandTracking = true;
            if (typeof window !== 'undefined' && window.app && window.app.diag) {
                window.app.diag('hand_tracking_active');
            }
        }
    }

    _processHandTracked(hand, frame, xrRefSpace, joints, nowMs) {
        const wrist = joints.get('wrist');
        const thumb = joints.get('thumb-tip');
        const index = joints.get('index-finger-tip');
        if (!wrist || !thumb || !index) return false;

        const wristPose = frame.getJointPose(wrist, xrRefSpace);
        const thumbPose = frame.getJointPose(thumb, xrRefSpace);
        const indexPose = frame.getJointPose(index, xrRefSpace);
        if (!wristPose || !thumbPose || !indexPose) return false;

        const tx = thumbPose.transform.position;
        const ix = indexPose.transform.position;
        const dx = tx.x - ix.x, dy = tx.y - ix.y, dz = tx.z - ix.z;
        const dist = Math.hypot(dx, dy, dz);

        // Hysteresis: only flip from "pinching=true" back to false once the
        // fingers are clearly apart. Cuts engage/disengage chatter at the
        // threshold.
        const st = this._state[hand];
        let pinching;
        if (st.wasPinching) {
            pinching = dist < PINCH_RELEASE_M;
        } else {
            pinching = dist < PINCH_DISTANCE_M;
        }
        st.wasPinching = pinching;

        // Reuse the controller path: wrist transform stands in for grip pose,
        // pinch stands in for grip-pressed.
        this._processHand(hand, wristPose, pinching ? 1.0 : 0.0, nowMs);
        return true;
    }

    _processHand(hand, pose, gripVal, nowMs) {
        const st = this._state[hand];
        const pos = pose.transform.position;
        const ori = pose.transform.orientation;

        // Velocity buffer (used for swipe detection on either hand).
        st.posBuffer.push({ t: nowMs, p: [pos.x, pos.y, pos.z] });
        if (st.posBuffer.length > VELOCITY_BUFFER_SIZE) st.posBuffer.shift();

        if (st.posBuffer.length >= 2) {
            const a = st.posBuffer[0];
            const b = st.posBuffer[st.posBuffer.length - 1];
            const dt = Math.max((b.t - a.t) / 1000, 1e-3);
            const vx = (b.p[0] - a.p[0]) / dt;
            const vy = (b.p[1] - a.p[1]) / dt;
            const vz = (b.p[2] - a.p[2]) / dt;
            const speedX = Math.abs(vx);
            const cross = Math.abs(vy) + Math.abs(vz);
            if (
                speedX > SWIPE_VELOCITY_THRESHOLD &&
                speedX > cross * SWIPE_AXIS_RATIO &&
                nowMs - st.lastSwipeMs > SWIPE_DEBOUNCE_MS
            ) {
                st.lastSwipeMs = nowMs;
                this.onGesture({ type: 'swipe', hand, value: vx });
            }
        }

        // Engage / disengage on grip transitions.
        const wantEngaged = gripVal > GRIP_THRESHOLD && hand === 'right';
        if (wantEngaged && !st.engaged) {
            st.engaged = true;
            // Capture initial wrist axes — we measure roll relative to these so
            // small wrist movements at engage-time don't kick the cursor.
            const { forward, up } = quatBasis(ori);
            st.engagedForward = forward;
            st.engagedUp = up;
            st.lastRoll = 0;
            this.onGesture({ type: 'engage' });
        } else if (!wantEngaged && st.engaged) {
            st.engaged = false;
            st.engagedForward = null;
            st.engagedUp = null;
            this.onGesture({ type: 'disengage' });
        }

        // While engaged, emit roll deltas relative to the captured forward.
        if (st.engaged && st.engagedForward) {
            const { forward, up } = quatBasis(ori);
            // Signed angle around the engaged forward axis between engagedUp and current up.
            const roll = signedAngleAroundAxis(st.engagedUp, up, st.engagedForward);
            const delta = wrapPi(roll - st.lastRoll);
            st.lastRoll = roll;
            if (Math.abs(delta) > 1e-4) {
                this.onGesture({ type: 'palm_roll_delta', value: delta });
            }
        }
    }
}

// Quaternion -> orthonormal basis. Returns the controller's forward (-Z) and up (+Y).
function quatBasis(q) {
    const x = q.x, y = q.y, z = q.z, w = q.w;
    // Rotate (0, 0, -1)
    const fx = -2 * (x * z + w * y);
    const fy = -2 * (y * z - w * x);
    const fz = -(1 - 2 * (x * x + y * y));
    // Rotate (0, 1, 0)
    const ux = 2 * (x * y - w * z);
    const uy = 1 - 2 * (x * x + z * z);
    const uz = 2 * (y * z + w * x);
    return { forward: normalize3([fx, fy, fz]), up: normalize3([ux, uy, uz]) };
}

function normalize3(v) {
    const n = Math.hypot(v[0], v[1], v[2]) || 1;
    return [v[0] / n, v[1] / n, v[2] / n];
}

function dot3(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

function cross3(a, b) {
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
}

// Signed angle from `from` to `to` around `axis`, all unit vectors.
// Positive when rotation is right-handed about `axis`.
function signedAngleAroundAxis(from, to, axis) {
    // Project onto plane perpendicular to axis.
    const fp = projectOntoPlane(from, axis);
    const tp = projectOntoPlane(to, axis);
    const fpN = normalize3(fp);
    const tpN = normalize3(tp);
    const cosA = Math.max(-1, Math.min(1, dot3(fpN, tpN)));
    const angle = Math.acos(cosA);
    const sign = dot3(cross3(fpN, tpN), axis) >= 0 ? 1 : -1;
    return angle * sign;
}

function projectOntoPlane(v, axis) {
    const d = dot3(v, axis);
    return [v[0] - axis[0] * d, v[1] - axis[1] * d, v[2] - axis[2] * d];
}

function wrapPi(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
}
