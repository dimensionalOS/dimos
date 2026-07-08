// Shared Go2 command constants — the protocol/policy data that MUST stay
// identical between the DOM cockpit (views/go2.js) and the VR cockpit
// (vrui.js). Kept here so the two can't drift (a retuned Rage scale or a
// changed confirm-set in one but not the other would be an invisible bug).
//
// Display-catalog arrays (POSTURE/ACTIONS/CAMS) stay local to each view —
// their `name`/`id` values match but the labels differ intentionally (VR
// uses tighter strings for its panels).

// Speed bar. normal/high = browser-side velocity scale; rage additionally
// flips the firmware Rage FSM (set_mode). Envelope note: firmware widens to
// ~2.5 m/s only under harder stick push, so rage lin is pushed past 1.0 to
// actually exploit it (buildTwist's Shift can add ×2 on top).
export const SPEEDS = [
    { mode: 'normal', label: 'Normal', scale: { lin: 0.5, ang: 0.5 } },
    { mode: 'high', label: 'High', scale: { lin: 1.0, ang: 1.0 } },
    { mode: 'rage', label: 'Rage', scale: { lin: 2.0, ang: 1.5 } },
];

// Acrobatic actions — the robot leaps, so they're confirm-gated before firing.
// Mirrors the robot-side allow-list.
export const CONFIRM_ACTIONS = new Set(['FrontPounce', 'FrontJump']);

// Sport-command name → latched posture the UI reflects on a confirmed ack.
// Superset across both cockpits (PoseStand only reachable from the DOM view).
export const POSTURE_STATE = {
    StandReady: 'StandReady', PoseStand: 'PoseStand', StandDown: 'StandDown',
    RecoveryStand: 'RecoveryStand', Sit: 'Sit',
};

// E-STOP send, shared by all three cockpits (DOM/VR-panel/VR-button). Latch via
// the dedicated estop type (new robots) + a legacy Damp for older ones; fire-
// and-forget on the reliable channel. `nonce` bumps and returns the view's id.
export function sendEstop(chan, nonce) {
    if (!chan || chan.readyState !== 'open') return;
    chan.send(JSON.stringify({ type: 'estop', nonce: nonce() }));
    chan.send(JSON.stringify({ type: 'sport_cmd', name: 'Damp', nonce: nonce() }));
}
export function sendEstopClear(chan, nonce) {
    if (!chan || chan.readyState !== 'open') return;
    chan.send(JSON.stringify({ type: 'estop_clear', nonce: nonce() }));
}
