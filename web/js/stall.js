// Video-freshness drive gate (HARDENING_PLAN A1).
//
// Policy: once video has played ("armed"), if the frame clock stops advancing
// for STALL_MS the operator is driving blind — block drive and show the
// overlay. When frames resume, auto-clear the stall but keep drive blocked
// until the operator releases all drive keys (neutral gate), so a held W
// can't lunge the robot the instant the picture unfreezes.
//
// Liveness signal is video.currentTime progression — NOT the e2e latency
// stamp (benchmark-only: reads 0 unless the robot runs latency_stamp) and NOT
// getStats (absent on the LiveKit path). currentTime works on both transports
// and is a cheap property read at tick rate.
//
// Pure logic lives in createStallGate so node:test can drive it with fake
// clocks; the keyboard loop feeds it real <video> readings.

export const STALL_MS = 1000;

export function createStallGate({ stallMs = STALL_MS } = {}) {
    let lastMediaTime = -1;      // furthest video.currentTime seen
    let lastProgressMs = null;   // wall time of the last advance
    let armed = false;           // no lockout before first frame (no-video robots drive as before)
    let stalled = false;
    let resumePending = false;   // frames are back but drive keys still held

    return {
        /**
         * Call once per drive tick.
         * @param mediaTime  video.currentTime, or -1 when no video element/frame
         * @param nowMs      monotonic clock (performance.now())
         * @param keysHeld   true while any drive input is engaged
         * @returns {{stalled: boolean, blocked: boolean, armed: boolean}}
         *   stalled → show the overlay; blocked → suppress drive sends.
         */
        sample(mediaTime, nowMs, keysHeld) {
            if (mediaTime >= 0 && mediaTime > lastMediaTime) {
                lastMediaTime = mediaTime;
                lastProgressMs = nowMs;
                armed = true;
                if (stalled) {
                    stalled = false;
                    resumePending = true;  // auto-resume, gated on neutral below
                }
            }
            if (armed && !stalled && lastProgressMs !== null &&
                nowMs - lastProgressMs > stallMs) {
                stalled = true;
                resumePending = false;
            }
            if (resumePending && !keysHeld) resumePending = false;
            return { stalled, blocked: stalled || resumePending, armed };
        },
    };
}

// Read the liveness signal off a <video>; -1 when it can't have a frame yet.
export function videoMediaTime(v) {
    return v && v.readyState >= 2 ? v.currentTime : -1;
}
