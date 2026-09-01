import assert from "node:assert/strict";
import test from "node:test";

import { scheduleTrackingFrame } from "./static/tracking_timing.mjs";

function countTrackingFrames(displayHz, seconds) {
    let deadline = null;
    let count = 0;
    for (let frame = 0; frame < displayHz * seconds; frame++) {
        const result = scheduleTrackingFrame(frame * 1000 / displayHz, deadline);
        deadline = result.nextDeadline;
        if (result.send) count++;
    }
    return count;
}

test("phase pacing averages 50 Hz at common headset refresh rates", () => {
    assert.equal(countTrackingFrames(72, 10), 500);
    assert.equal(countTrackingFrames(90, 10), 500);
});

test("a render stall schedules one current frame without a catch-up burst", () => {
    const first = scheduleTrackingFrame(0, null);
    const afterStall = scheduleTrackingFrame(1000, first.nextDeadline);
    const nextRender = scheduleTrackingFrame(1001, afterStall.nextDeadline);

    assert.equal(first.send, true);
    assert.equal(afterStall.send, true);
    assert.equal(afterStall.nextDeadline, 1020);
    assert.equal(nextRender.send, false);
});
