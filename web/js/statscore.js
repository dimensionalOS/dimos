// Pure video-stats math shared by the Cloudflare path (webrtc.js, pc.getStats)
// and the LiveKit path (livekit.js, receiver.getStats) — HARDENING_PLAN E2.
// No DOM, no timers: node:test drives it with fake stats objects.

// Resolve the in-use ICE path from a getStats() report (Map-like): find the
// active candidate-pair, look up its local candidate, map candidateType.
//   host        → 'direct'
//   srflx/prflx → 'stun'
//   relay       → 'turn'
export function selectedIceType(report) {
    let pair = null;
    report.forEach((r) => {
        if (r.type !== 'candidate-pair') return;
        // `selected` (Chrome) or nominated+succeeded (spec) marks the active pair.
        if (r.selected || (r.nominated && r.state === 'succeeded')) pair = r;
    });
    if (!pair) return null;
    const local = report.get(pair.localCandidateId);
    if (!local) return null;
    return local.candidateType === 'relay' ? 'turn'
        : local.candidateType === 'srflx' || local.candidateType === 'prflx' ? 'stun'
        : 'direct';  // host
}

// Find the video inbound-rtp entry in a stats report; null when absent.
export function findVideoInbound(report) {
    let inbound = null;
    report.forEach((r) => {
        if (r.type === 'inbound-rtp' && r.kind === 'video') inbound = r;
    });
    return inbound;
}

// Delta two consecutive inbound-rtp samples into the video_stats payload the
// robot logs and the HUD renders. Returns null when a delta isn't computable
// yet (first sample, or non-advancing stats clock — callers just skip a tick).
export function computeVideoStats(prev, inbound, e2eLatencyMs = 0) {
    if (!prev || !inbound) return null;
    const dt = (inbound.timestamp - prev.timestamp) / 1000;
    if (dt <= 0) return null;

    const dFrames = (inbound.framesDecoded ?? 0) - (prev.framesDecoded ?? 0);
    const dBytes = (inbound.bytesReceived ?? 0) - (prev.bytesReceived ?? 0);
    const dLost = (inbound.packetsLost ?? 0) - (prev.packetsLost ?? 0);
    const dRecv = (inbound.packetsReceived ?? 0) - (prev.packetsReceived ?? 0);
    const lossDen = dLost + dRecv;
    // Avg decode time per frame over the window — latency component.
    const dDecode = (inbound.totalDecodeTime ?? 0) - (prev.totalDecodeTime ?? 0);
    const decodeMs = dFrames > 0 ? +((dDecode / dFrames) * 1000).toFixed(1) : 0;

    return {
        type: 'video_stats',
        fps: +(dFrames / dt).toFixed(1),
        kbps: +((dBytes * 8) / dt / 1000).toFixed(1),
        width: inbound.frameWidth ?? 0,
        height: inbound.frameHeight ?? 0,
        loss_pct: lossDen > 0 ? +((dLost / lossDen) * 100).toFixed(2) : 0,
        jitter_ms: +((inbound.jitter ?? 0) * 1000).toFixed(1),
        frames_dropped: inbound.framesDropped ?? 0,
        freezes: inbound.freezeCount ?? 0,
        // Receive-side latency (network RTT lives in clock-sync, not here).
        jitter_buffer_ms:
            inbound.jitterBufferEmittedCount
                ? +((inbound.jitterBufferDelay / inbound.jitterBufferEmittedCount) * 1000).toFixed(1)
                : 0,
        decode_ms: decodeMs,
        e2e_latency_ms: e2eLatencyMs,  // glass-to-glass, 0 if not stamping
    };
}
