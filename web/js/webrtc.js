// WebRTC dial-out + clock sync + video-stats reporter + state-channel dispatch.

import { api } from './api.js';
import { ensureRobotCam, setStatus } from './dom.js';
import {
    CLOCK_SYNC_BURST_COUNT,
    CLOCK_SYNC_BURST_INTERVAL_MS,
    CLOCK_SYNC_DRIFT_INTERVAL_MS,
    VIDEO_STATS_INTERVAL_MS,
    state,
} from './state.js';

export async function setupWebRTC(sessionId) {
    setStatus('Negotiating WebRTC...');
    state.pc = new RTCPeerConnection({
        iceServers: [{ urls: 'stun:stun.cloudflare.com:3478' }]
    });
    const pc = state.pc;

    const sctpPlaceholder = pc.createDataChannel('_sctp_init');

    // recvonly transceiver gives the offer a video m-section to bind to.
    pc.addTransceiver('video', { direction: 'recvonly' });
    pc.ontrack = (e) => {
        if (e.track.kind !== 'video') return;
        // Keyboard view has a static <video>; VR has none and uses a hidden
        // one as a GL texture source. Only the keyboard one is shown.
        const existed = !!document.getElementById('robot-cam');
        const v = ensureRobotCam();
        v.srcObject = e.streams[0] || new MediaStream([e.track]);
        if (existed) v.style.display = 'block';
        v.play?.().catch(() => {});  // immersive: no user-gesture; nudge autoplay
    };

    const iceFailed = new Promise((_, reject) => {
        pc.oniceconnectionstatechange = () => {
            if (pc.iceConnectionState === 'failed' ||
                pc.iceConnectionState === 'disconnected') {
                reject(new Error('ICE ' + pc.iceConnectionState));
            }
        };
    });

    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);

    // Non-trickle ICE.
    await new Promise(resolve => {
        if (pc.iceGatheringState === 'complete') return resolve();
        pc.onicegatheringstatechange = () => {
            if (pc.iceGatheringState === 'complete') resolve();
        };
    });

    const data = await api('POST', `/sessions/${sessionId}/join`, {
        role: 'operator',
        sdp_offer: pc.localDescription.sdp,
    });
    await pc.setRemoteDescription({ type: 'answer', sdp: data.sdp_answer });

    // CF rejects /datachannels/new until the PC is 'connected'.
    await Promise.race([
        new Promise((resolve, reject) => {
            const check = () => {
                if (pc.connectionState === 'connected') resolve();
                else if (pc.connectionState === 'failed' ||
                         pc.connectionState === 'closed') {
                    reject(new Error('PC ' + pc.connectionState));
                }
            };
            check();
            pc.addEventListener('connectionstatechange', check);
        }),
        iceFailed,
    ]);

    try { sctpPlaceholder.close(); } catch (_) {}

    const bridge = await api('POST', `/sessions/${sessionId}/bridge-datachannel`);
    if (bridge.cmd_channel_id == null) {
        throw new Error('Broker did not return cmd_channel_id');
    }

    state.cmdChannel = pc.createDataChannel('cmd_unreliable', {
        negotiated: true,
        id: bridge.cmd_channel_id,
        ordered: false,
        maxRetransmits: 0,
    });
    state.cmdChannel.binaryType = 'arraybuffer';

    await Promise.race([
        new Promise((resolve, reject) => {
            if (state.cmdChannel.readyState === 'open') return resolve();
            state.cmdChannel.onopen = () => resolve();
            state.cmdChannel.onerror = (e) => reject(e);
        }),
        iceFailed,
    ]);

    // state_reliable — best-effort. Older brokers omit state_channel_id and
    // clock sync just doesn't run; cmdChannel is unaffected.
    if (bridge.state_channel_id != null) {
        state.stateChannel = pc.createDataChannel('state_reliable', {
            negotiated: true,
            id: bridge.state_channel_id,
            ordered: true,
        });
        state.stateChannel.onopen = () => {
            startClockSync(state.stateChannel);
            startVideoStats(state.stateChannel);
        };
        // Fallback for old brokers w/o state_back_channel_id (no-op on new ones).
        state.stateChannel.onmessage = (e) => handleStateMessage(e.data);
        state.stateChannel.onerror = (e) => console.warn('[state-channel] error', e);
        state.stateChannel.onclose = () => console.info('[state-channel] closed');
    }

    // CF datachannels are one-way; this is the only robot → operator path.
    if (bridge.state_back_channel_id != null) {
        state.stateBackChannel = pc.createDataChannel('state_reliable_back', {
            negotiated: true,
            id: bridge.state_back_channel_id,
            ordered: true,
        });
        state.stateBackChannel.onmessage = (e) => handleStateMessage(e.data);
        state.stateBackChannel.onerror = (e) => console.warn('[state-back-channel] error', e);
        state.stateBackChannel.onclose = () => console.info('[state-back-channel] closed');
    }

    // Apply the broker's video-pull renegotiation offer so ontrack fires.
    // Best-effort: failure leaves commands + clock intact.
    if (bridge.video_offer) {
        try {
            await pc.setRemoteDescription({ type: 'offer', sdp: bridge.video_offer });
            const answer = await pc.createAnswer();
            await pc.setLocalDescription(answer);
            await api('POST', `/sessions/${sessionId}/renegotiate-answer`, {
                sdp_answer: pc.localDescription.sdp,
            });
            console.info('[video] renegotiation complete — awaiting frames');
        } catch (err) {
            console.warn('[video] renegotiation failed (commands unaffected):', err);
        }
    } else {
        console.info('[video] no video_offer — broker video_status:', bridge.video_status || '(none)');
    }
}

// ─── Clock sync ──────────────────────────────────────────────────────────
// Burst of N pings to converge fast, then one every 30s to track drift.
export function startClockSync(channel) {
    let sent = 0;
    const sendPing = () => {
        if (!channel || channel.readyState !== 'open') return;
        channel.send(JSON.stringify({ type: 'ping', client_ts: Date.now() / 1000 }));
    };
    sendPing();
    sent = 1;
    state.clockSyncBurstTimer = setInterval(() => {
        sendPing();
        sent += 1;
        if (sent >= CLOCK_SYNC_BURST_COUNT) {
            clearInterval(state.clockSyncBurstTimer);
            state.clockSyncBurstTimer = null;
            state.clockSyncDriftTimer = setInterval(sendPing, CLOCK_SYNC_DRIFT_INTERVAL_MS);
        }
    }, CLOCK_SYNC_BURST_INTERVAL_MS);
}

export function stopClockSync() {
    if (state.clockSyncBurstTimer) { clearInterval(state.clockSyncBurstTimer); state.clockSyncBurstTimer = null; }
    if (state.clockSyncDriftTimer) { clearInterval(state.clockSyncDriftTimer); state.clockSyncDriftTimer = null; }
}

// ─── Video stats ─────────────────────────────────────────────────────────
// getStats() lives only in the browser, so the operator samples the inbound
// track and reports health to the robot (rate/bitrate/loss = deltas between
// consecutive 1s samples). Robot folds it into report.md.
export function startVideoStats(channel) {
    state.videoStatsPrev = null;
    state.videoStatsTimer = setInterval(async () => {
        if (!channel || channel.readyState !== 'open' || !state.pc) return;
        let inbound = null;
        try {
            const stats = await state.pc.getStats();
            stats.forEach((r) => {
                if (r.type === 'inbound-rtp' && r.kind === 'video') inbound = r;
            });
        } catch (_) { return; }
        if (!inbound) return;

        const now = inbound.timestamp;  // ms, getStats clock
        const prev = state.videoStatsPrev;
        state.videoStatsPrev = inbound;
        if (!prev) return;  // need two samples for deltas

        const dt = (now - prev.timestamp) / 1000;
        if (dt <= 0) return;
        const dFrames = (inbound.framesDecoded ?? 0) - (prev.framesDecoded ?? 0);
        const dBytes = (inbound.bytesReceived ?? 0) - (prev.bytesReceived ?? 0);
        const dLost = (inbound.packetsLost ?? 0) - (prev.packetsLost ?? 0);
        const dRecv = (inbound.packetsReceived ?? 0) - (prev.packetsReceived ?? 0);
        const lossDen = dLost + dRecv;
        // Avg decode time per frame over the window — latency component.
        const dDecode = (inbound.totalDecodeTime ?? 0) - (prev.totalDecodeTime ?? 0);
        const decodeMs = dFrames > 0 ? +((dDecode / dFrames) * 1000).toFixed(1) : 0;

        const payload = {
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
        };
        try { channel.send(JSON.stringify(payload)); } catch (_) {}
        state.liveStats.video = payload;  // latest sample for the HUD/VR quad
    }, VIDEO_STATS_INTERVAL_MS);
}

export function stopVideoStats() {
    if (state.videoStatsTimer) { clearInterval(state.videoStatsTimer); state.videoStatsTimer = null; }
    state.videoStatsPrev = null;
}

function handleStateMessage(data) {
    let msg;
    try { msg = JSON.parse(data); } catch (_) { return; }
    if (msg.type === 'pong') applyPong(msg);
    // Robot-measured command-plane health (latency/jitter/loss) — what
    // actually arrived, which the operator can't see from its send side.
    else if (msg.type === 'robot_telemetry') state.liveStats.cmd = msg.cmd;
}

// NTP-style min-RTT: accept only samples below the running best, ignore
// clearly-bogus ones.
function applyPong(pong) {
    const recvTsSec = Date.now() / 1000;
    const rttMs = (recvTsSec - pong.client_ts) * 1000;
    if (rttMs < 0 || rttMs > 5000) return;
    if (rttMs >= state.bestRttMs) return;
    state.bestRttMs = rttMs;
    const offsetSec =
        ((pong.robot_ts - pong.client_ts) + (pong.robot_ts - recvTsSec)) / 2;
    state.clockOffsetMs = offsetSec * 1000;
    state.liveStats.rttMs = rttMs;
    state.liveStats.offsetMs = state.clockOffsetMs;
    console.log(`[clock-sync] rtt=${rttMs.toFixed(1)}ms offset=${state.clockOffsetMs.toFixed(1)}ms`);
    // Report back so the robot can log it (can't derive from one-way pings).
    if (state.stateChannel && state.stateChannel.readyState === 'open') {
        state.stateChannel.send(JSON.stringify({
            type: 'clock_report',
            rtt_ms: +rttMs.toFixed(1),
            offset_ms: +state.clockOffsetMs.toFixed(1),
        }));
    }
}

export function send(bytes) {
    if (state.cmdChannel && state.cmdChannel.readyState === 'open') {
        state.cmdChannel.send(bytes);
        state.cmdSendCount++;  // sampled into liveStats.cmdHz once per second
    }
}
