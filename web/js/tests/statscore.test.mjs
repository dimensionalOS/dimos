// node --test web/js/tests/
// Video-stats delta math + ICE-path resolution (HARDENING_PLAN E2).

import assert from 'node:assert/strict';
import { test } from 'node:test';

import { computeVideoStats, findVideoInbound, selectedIceType } from '../statscore.js';

function sample(overrides = {}) {
    return {
        timestamp: 0,
        framesDecoded: 0,
        bytesReceived: 0,
        packetsLost: 0,
        packetsReceived: 0,
        totalDecodeTime: 0,
        jitter: 0,
        frameWidth: 1280,
        frameHeight: 720,
        framesDropped: 0,
        freezeCount: 0,
        jitterBufferDelay: 0,
        jitterBufferEmittedCount: 0,
        ...overrides,
    };
}

test('null on first sample and on non-advancing stats clock', () => {
    const a = sample({ timestamp: 1000 });
    assert.equal(computeVideoStats(null, a), null);
    assert.equal(computeVideoStats(a, sample({ timestamp: 1000 })), null); // dt=0
    assert.equal(computeVideoStats(a, sample({ timestamp: 900 })), null);  // dt<0
});

test('rates are per-second deltas', () => {
    const prev = sample({ timestamp: 0, framesDecoded: 100, bytesReceived: 0 });
    const cur = sample({
        timestamp: 2000,  // 2s window
        framesDecoded: 160,          // +60 → 30 fps
        bytesReceived: 500_000,      // 4Mb/2s → 2000 kbps
        packetsLost: 5,
        packetsReceived: 495,        // 5/(5+495) → 1%
        totalDecodeTime: 0.6,        // 0.6s/60f → 10ms
        jitter: 0.012,
        jitterBufferDelay: 30,
        jitterBufferEmittedCount: 300,  // → 100ms
    });
    const s = computeVideoStats(prev, cur, 123.4);
    assert.equal(s.fps, 30);
    assert.equal(s.kbps, 2000);
    assert.equal(s.loss_pct, 1);
    assert.equal(s.decode_ms, 10);
    assert.equal(s.jitter_ms, 12);
    assert.equal(s.jitter_buffer_ms, 100);
    assert.equal(s.e2e_latency_ms, 123.4);
    assert.equal(s.width, 1280);
    assert.equal(s.type, 'video_stats');
});

test('handles missing counters (older browsers) as zeros', () => {
    const prev = { timestamp: 0 };
    const cur = { timestamp: 1000 };
    const s = computeVideoStats(prev, cur);
    assert.equal(s.fps, 0);
    assert.equal(s.loss_pct, 0);
    assert.equal(s.jitter_buffer_ms, 0);
    assert.equal(s.decode_ms, 0);
});

// RTCStatsReport is Map-like: forEach + get.
function fakeReport(entries) {
    const m = new Map(entries.map((e) => [e.id, e]));
    return { forEach: (fn) => m.forEach(fn), get: (id) => m.get(id) };
}

test('selectedIceType maps candidate types (selected flag, Chrome)', () => {
    for (const [candidateType, want] of [
        ['relay', 'turn'], ['srflx', 'stun'], ['prflx', 'stun'], ['host', 'direct'],
    ]) {
        const r = fakeReport([
            { id: 'p', type: 'candidate-pair', selected: true, localCandidateId: 'c' },
            { id: 'c', type: 'local-candidate', candidateType },
        ]);
        assert.equal(selectedIceType(r), want, candidateType);
    }
});

test('selectedIceType falls back to nominated+succeeded (spec)', () => {
    const r = fakeReport([
        { id: 'p', type: 'candidate-pair', nominated: true, state: 'succeeded', localCandidateId: 'c' },
        { id: 'c', type: 'local-candidate', candidateType: 'relay' },
    ]);
    assert.equal(selectedIceType(r), 'turn');
});

test('selectedIceType null when no active pair or dangling candidate', () => {
    assert.equal(selectedIceType(fakeReport([])), null);
    const dangling = fakeReport([
        { id: 'p', type: 'candidate-pair', selected: true, localCandidateId: 'missing' },
    ]);
    assert.equal(selectedIceType(dangling), null);
});

test('findVideoInbound picks the video inbound-rtp entry only', () => {
    const inbound = { id: 'v', type: 'inbound-rtp', kind: 'video', timestamp: 1 };
    const r = fakeReport([
        { id: 'a', type: 'inbound-rtp', kind: 'audio' },
        inbound,
        { id: 'o', type: 'outbound-rtp', kind: 'video' },
    ]);
    assert.equal(findVideoInbound(r), inbound);
    assert.equal(findVideoInbound(fakeReport([])), null);
});
