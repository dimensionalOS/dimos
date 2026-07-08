// Shared xArm (manipulation) command builders — the protocol/policy data for
// the hosted-teleop arm, kept here so the VR cockpit (vrarm.js / vrarmui.js)
// and any future DOM cockpit can't drift. Parallel to go2cmd.js.
//
// Two planes, matching the robot side (ArmHostedConnection):
//   • cmd_unreliable (state.cmdChannel): LCM PoseStamped + Joy, per controller
//     per frame. The operator sends RAW WebXR gripSpace poses — the robot's
//     webxr_to_robot owns the frame conversion and per-hand alignment, and
//     re-bases on engage, so we send ABSOLUTE poses (no delta math here).
//   • state_reliable (state.stateChannel): JSON control plane (estop / clear /
//     camera_select), acked on state_reliable_back.

import { geometry_msgs, sensor_msgs, std_msgs } from 'https://esm.sh/jsr/@dimos/msgs@0.1.4';

// nowMs is robot-clock time (Date.now() + state.clockOffsetMs). It only feeds
// the robot's command-plane latency stats — not IK — but stamping it right
// makes the operator HUD's measured latency meaningful.
function stamp(nowMs) {
    return new std_msgs.Time({ sec: Math.floor(nowMs / 1000), nsec: (nowMs % 1000) * 1_000_000 });
}

// Raw WebXR controller pose → PoseStamped. frame_id carries the handedness
// ("left"/"right"), which the robot uses to route to a hand and pick the L/R
// alignment — it is NOT a coordinate frame. Pass pose.transform.position /
// .orientation straight through (no conversion).
export function buildPoseStamped(handedness, position, orientation, nowMs) {
    return new geometry_msgs.PoseStamped({
        header: new std_msgs.Header({ stamp: stamp(nowMs), frame_id: handedness }),
        pose: new geometry_msgs.Pose({
            position: new geometry_msgs.Point({ x: position.x, y: position.y, z: position.z }),
            orientation: new geometry_msgs.Quaternion({
                x: orientation.x, y: orientation.y, z: orientation.z, w: orientation.w,
            }),
        }),
    });
}

// Gamepad → Joy. The robot requires axes ≥4 and buttons ≥7 or it drops the
// frame. axes = [stickX, stickY, triggerAnalog, gripAnalog]; engage = primary
// button (index 4 = X/A); gripper = trigger analog (axes[2]).
export function buildJoy(handedness, gamepad, nowMs) {
    // xr-standard exposes the thumbstick on axes[2]/[3]; older mappings on [0]/[1].
    const xr = gamepad.mapping === 'xr-standard';
    const stickX = (xr ? gamepad.axes[2] : gamepad.axes[0]) ?? 0.0;
    const stickY = (xr ? gamepad.axes[3] : gamepad.axes[1]) ?? 0.0;
    const axes = [
        stickX,
        stickY,
        gamepad.buttons[0]?.value ?? 0.0,  // trigger analog → gripper
        gamepad.buttons[1]?.value ?? 0.0,  // grip analog
    ];
    // Digital 0/1 for every button the pad exposes; the robot reads indices
    // 0-6 (trigger, grip, touchpad, thumbstick, X/A, Y/B, menu).
    const buttons = [];
    for (let i = 0; i < gamepad.buttons.length; i++) {
        buttons.push(gamepad.buttons[i]?.pressed ? 1 : 0);
    }
    return new sensor_msgs.Joy({
        header: new std_msgs.Header({ stamp: stamp(nowMs), frame_id: handedness }),
        axes_length: axes.length,
        buttons_length: buttons.length,
        axes,
        buttons,
    });
}

function chanReady(chan) {
    return chan && chan.readyState === 'open';
}

// ── JSON control plane (state_reliable) ──────────────────────────────

// E-STOP latch. Unlike the Go2 (which also sends a legacy Damp), the arm robot
// latches on the dedicated estop type alone and freezes the coordinator target.
// nonce() bumps and returns the caller's id so the cmd_ack can be matched.
export function sendEstop(chan, nonce) {
    if (!chanReady(chan)) return;
    chan.send(JSON.stringify({ type: 'estop', nonce: nonce() }));
}

// Clear the latch. Does NOT resume motion — hands stay disengaged until the
// operator releases and re-presses engage (robot recaptures the baseline pose).
export function sendEstopClear(chan, nonce) {
    if (!chanReady(chan)) return;
    chan.send(JSON.stringify({ type: 'estop_clear', nonce: nonce() }));
}

// Pick which camera(s) the robot muxes into the video track. Names filtered to
// known ("cam1"/"cam2") robot-side; empty falls back to the first cam.
export function sendCameraSelect(chan, cams) {
    if (!chanReady(chan)) return;
    chan.send(JSON.stringify({ type: 'camera_select', cams }));
}
