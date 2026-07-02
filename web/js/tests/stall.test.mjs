// node --test web/js/tests/
// Drive-gate state machine (HARDENING_PLAN A1): arm → stall → auto-resume
// with neutral gate. Pure fake-clock tests, no DOM.

import assert from 'node:assert/strict';
import { test } from 'node:test';

import { createStallGate } from '../stall.js';

const STALL = 1000;

test('not armed before first frame — no lockout on no-video robots', () => {
    const g = createStallGate({ stallMs: STALL });
    // 10s of ticks with no video at all: never blocked.
    for (let t = 0; t <= 10_000; t += 100) {
        const s = g.sample(-1, t, true);
        assert.equal(s.armed, false);
        assert.equal(s.blocked, false);
        assert.equal(s.stalled, false);
    }
});

test('arms on first frame, stalls after threshold', () => {
    const g = createStallGate({ stallMs: STALL });
    assert.equal(g.sample(0.033, 0, false).armed, true);
    // frames flow to t=500
    assert.equal(g.sample(0.5, 500, false).stalled, false);
    // frozen frame clock: not yet over threshold at +1000
    assert.equal(g.sample(0.5, 1500, false).stalled, false);
    // over threshold → stalled + blocked
    const s = g.sample(0.5, 1600, false);
    assert.equal(s.stalled, true);
    assert.equal(s.blocked, true);
});

test('auto-resumes when frames return, but blocks until keys released', () => {
    const g = createStallGate({ stallMs: STALL });
    g.sample(0.1, 0, true);
    g.sample(0.1, 1200, true);            // → stalled while holding W
    assert.equal(g.sample(0.1, 1300, true).stalled, true);

    // frames resume while W still held: overlay clears, drive still blocked
    let s = g.sample(0.2, 1400, true);
    assert.equal(s.stalled, false);
    assert.equal(s.blocked, true);

    // still holding → still blocked
    s = g.sample(0.3, 1500, true);
    assert.equal(s.blocked, true);

    // release keys → neutral gate passes, drive unblocked
    s = g.sample(0.4, 1600, false);
    assert.equal(s.blocked, false);
});

test('resume with keys already released unblocks immediately', () => {
    const g = createStallGate({ stallMs: STALL });
    g.sample(0.1, 0, false);
    g.sample(0.1, 1200, false);           // stalled, hands off
    const s = g.sample(0.2, 1300, false); // frames back, no keys held
    assert.equal(s.stalled, false);
    assert.equal(s.blocked, false);
});

test('re-stalls after a resume if frames freeze again', () => {
    const g = createStallGate({ stallMs: STALL });
    g.sample(0.1, 0, false);
    g.sample(0.1, 1200, false);           // stall #1
    g.sample(0.2, 1300, false);           // resume
    g.sample(0.3, 1400, false);
    const s = g.sample(0.3, 2600, false); // frozen ≥1000 again
    assert.equal(s.stalled, true);
    assert.equal(s.blocked, true);
});

test('paused-at-start video never arms (readyState guard maps to -1)', () => {
    const g = createStallGate({ stallMs: STALL });
    const s = g.sample(-1, 5000, true);
    assert.equal(s.armed, false);
    assert.equal(s.blocked, false);
});
