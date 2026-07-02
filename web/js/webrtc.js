// WebRTC dial-out + clock sync + video-stats reporter + state-channel dispatch.

import { api } from './api.js';
import { ensureRobotCam, setStatus } from './dom.js';
import {
    CLOCK_SYNC_BURST_COUNT,
    CLOCK_SYNC_BURST_INTERVAL_MS,
    CLOCK_SYNC_DRIFT_INTERVAL_MS,
    OP_HEARTBEAT_INTERVAL_MS,
    VIDEO_STATS_INTERVAL_MS,
    state,
} from './state.js';
import { computeVideoStats, findVideoInbound, selectedIceType } from './statscore.js';

const STUN_ONLY = [{ urls: 'stun:stun.cloudflare.com:3478' }];

// Connection waits hang forever on networks that silently drop UDP (ICE sits
// in 'checking'); cap them so the operator gets an error instead.
const CONNECT_TIMEOUT_MS = 20000;
const CHANNEL_OPEN_TIMEOUT_MS = 10000;
const GATHER_TIMEOUT_MS = 10000;
// Overall ceiling so a hung api() call (fetch has no default timeout) or any
// missed per-await guard can't wedge setupInProgress forever. Generous: sum
// of per-step caps is ~60s; this gives headroom without being mistaken for
// progress.
const SETUP_TIMEOUT_MS = 90000;
// After the first usable (srflx/relay) candidate, wait this long to scoop up
// sibling candidates (the relay leg lands ~80ms after srflx) before proceeding.
const GATHER_SETTLE_MS = 400;

export function timeout(ms, label) {
    return new Promise((_, reject) =>
        setTimeout(() => reject(new Error(label)), ms));
}

export async function setupWebRTC(sessionId) {
    // Re-entry would overwrite state.pc and leak the prior PC; the second
    // caller is rejected, the first finishes (or its finally clears the flag).
    if (state.setupInProgress) {
        throw new Error('Connect already in progress — disconnect first to retry');
    }
    state.setupInProgress = true;
    let timerId;
    const timer = new Promise((_, reject) => {
        timerId = setTimeout(() => reject(new Error('Connect timed out')), SETUP_TIMEOUT_MS);
    });
    try {
        return await Promise.race([_setupWebRTCInner(sessionId), timer]);
    } catch (err) {
        // Close partial PC so a failure (timeout or otherwise) doesn't leak
        // it past the next setupWebRTC entry.
        if (state.pc) { try { state.pc.close(); } catch (_) {} state.pc = null; }
        throw err;
    } finally {
        clearTimeout(timerId);
        state.setupInProgress = false;
    }
}

async function _setupWebRTCInner(sessionId) {
    setStatus('Negotiating WebRTC...');
    // TURN must be in the PC's config at construction for relay candidates
    // to gather with the offer. Best-effort: a broker without TURN
    // configured returns STUN-only, and a failed fetch degrades to it.
    let iceServers = STUN_ONLY;
    const tFetch = performance.now();
    try {
        const turn = await api('GET', '/sessions/turn-credentials');
        if (turn.ice_servers?.length) iceServers = turn.ice_servers;
    } catch (err) {
        // api() already logged out + navigated to auth — don't continue setup.
        if (err.message === 'Unauthorized' ||
            err.message === 'Session expired — log in again') {
            throw err;
        }
        console.warn('[turn] credential fetch failed — STUN only:', err);
    }
    const relayCount = iceServers.filter(s =>
        [].concat(s.urls || []).some(u => u.startsWith('turn'))).length;
    console.info(`[ice] credentials ${(performance.now() - tFetch).toFixed(0)}ms ` +
        `(${relayCount} TURN server(s))`);
    state.pc = new RTCPeerConnection({ iceServers });
    const pc = state.pc;

    const sctpPlaceholder = pc.createDataChannel('_sctp_init');

    // recvonly transceiver gives the offer a video m-section to bind to.
    pc.addTransceiver('video', { direction: 'recvonly' });
    pc.ontrack = (e) => {
        if (e.track.kind !== 'video') return;
        // Keyboard has a static <video>; VR uses a hidden one as a GL source.
        const existed = !!document.getElementById('robot-cam');
        const v = ensureRobotCam();
        // Stop the prior stream so renegotiation track-swaps don't leak.
        const prior = v.srcObject;
        if (prior && prior.getTracks) {
            for (const t of prior.getTracks()) t.stop();
        }
        v.srcObject = e.streams[0] || new MediaStream([e.track]);
        if (existed) v.style.display = 'block';
        v.play?.().catch(() => {});  // immersive has no user-gesture; nudge autoplay
    };

    // Resolve gather as soon as we have a routable (srflx/relay) candidate plus
    // a short settle window — don't wait for iceGatheringState='complete'. With
    // TURN configured, CF/Chrome keep the gatherer open probing extra relay
    // permutations long after every usable candidate exists, so 'complete' lags
    // ~10s behind a connection that was ready in <400ms.
    let onUsableCandidate = null;  // set by the gather phase below
    const candTypes = {};  // type → count, for a one-line gather summary
    pc.onicecandidate = (e) => {
        if (!e.candidate) return;
        const c = e.candidate;
        candTypes[c.type] = (candTypes[c.type] || 0) + 1;
        if ((c.type === 'srflx' || c.type === 'relay') && onUsableCandidate) onUsableCandidate();
    };
    pc.onicecandidateerror = (e) => {
        // 701 = TURN/STUN server unreachable; 401 = bad creds; 300/600 = misc.
        console.warn(`[ice] cand ERROR code=${e.errorCode} ${e.errorText || ''} ` +
            `url=${e.url || ''} host=${e.address || ''}:${e.port || ''}`);
    };

    // 'disconnected' is transient (network blips recover in ~1s); only
    // 'failed' is terminal.
    const iceFailed = new Promise((_, reject) => {
        pc.oniceconnectionstatechange = () => {
            if (pc.iceConnectionState === 'failed') {
                reject(new Error('ICE failed'));
            }
        };
    });

    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);

    // Non-trickle ICE: proceed once we have a usable (srflx/relay) candidate,
    // after a brief settle to scoop up siblings (e.g. the relay leg ~80ms after
    // srflx). Falls back to iceGatheringState='complete' or the hard cap so a
    // truly stalled gather still can't hang forever.
    const tGather = performance.now();
    let how = 'cap';
    await new Promise(resolve => {
        let settleTimer = null;
        const done = (reason) => { how = reason; clearTimeout(settleTimer); resolve(); };
        if (pc.iceGatheringState === 'complete') return done('complete');
        // First usable candidate → wait GATHER_SETTLE_MS for siblings, then go.
        onUsableCandidate = () => {
            if (settleTimer) return;
            settleTimer = setTimeout(() => done('usable+settle'), GATHER_SETTLE_MS);
        };
        pc.onicegatheringstatechange = () => {
            if (pc.iceGatheringState === 'complete') done('complete');
        };
        setTimeout(() => done('cap'), GATHER_TIMEOUT_MS);
    });
    onUsableCandidate = null;  // stop settling on late candidates
    const candSummary = Object.entries(candTypes).map(([t, n]) => `${t}×${n}`).join(' ') || 'none';
    console.info(`[ice] gather ${(performance.now() - tGather).toFixed(0)}ms (${how}) — ${candSummary}`);

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
        timeout(CONNECT_TIMEOUT_MS,
            'Timed out connecting — your network may block WebRTC (UDP)'),
    ]);

    try { sctpPlaceholder.close(); } catch (_) {}

    const bridge = await api('POST', `/sessions/${sessionId}/bridge-datachannel`);
    if (bridge.cmd_channel_id == null) {
        throw new Error('Broker did not return cmd_channel_id');
    }
    startOpHeartbeat(sessionId);

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
        timeout(CHANNEL_OPEN_TIMEOUT_MS, 'Command channel never opened'),
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
        state.stateBackChannel.onerror = (e) => console.warn('[state-back] error', e);
        state.stateBackChannel.onclose = () => console.info('[state-back] closed');
    } else {
        console.warn('[state-back] no state_back_channel_id from broker — cmd latency/SOC unavailable');
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
    _lastBestUpdateMs = 0;  // reset decay state for the new session
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
// Delta math + ICE-path resolution live in statscore.js (shared with the
// LiveKit path, unit-tested under web/js/tests/).

// Glass-to-glass latency: read the robot's capture stamp from a strip APPENDED
// below the video (robot draws it as extra rows, not over content). Constants
// MUST match _stamp() in hosted_connection.py.
const STAMP_CELL_PX = 16;
const STAMP_STRIP_PX = 16;  // height of the appended strip, in rows
const STAMP_SYNC = [1, 0, 1, 0];
const STAMP_TIME_BITS = 44;
const STAMP_CELLS = STAMP_SYNC.length + STAMP_TIME_BITS;
const _stampCanvas = document.createElement('canvas');

// Latency in ms, or 0 when the stamp is missing/unreadable (robot not stamping).
function readLatencyStamp() {
    const v = document.getElementById('robot-cam');
    if (!v || !v.videoWidth) return 0;
    const s = STAMP_CELL_PX;
    const w = STAMP_CELLS * s;
    if (v.videoWidth < w || v.videoHeight < STAMP_STRIP_PX) return 0;
    _stampCanvas.width = w;
    _stampCanvas.height = STAMP_STRIP_PX;
    const ctx = _stampCanvas.getContext('2d', { willReadFrequently: true });
    let px;
    try {
        // The strip is the bottom STAMP_STRIP_PX rows of the frame.
        const srcY = v.videoHeight - STAMP_STRIP_PX;
        ctx.drawImage(v, 0, srcY, w, STAMP_STRIP_PX, 0, 0, w, STAMP_STRIP_PX);
        px = ctx.getImageData(0, 0, w, STAMP_STRIP_PX).data;  // RGBA
    } catch (_) {
        return 0;  // tainted canvas / not ready
    }
    const cy = (STAMP_STRIP_PX / 2) | 0;
    const bitAt = (i) => {
        const cx = (i * s + s / 2) | 0;
        const o = (cy * w + cx) * 4;
        const luma = 0.299 * px[o] + 0.587 * px[o + 1] + 0.114 * px[o + 2];
        return luma >= 128 ? 1 : 0;
    };
    for (let i = 0; i < STAMP_SYNC.length; i++) {
        if (bitAt(i) !== STAMP_SYNC[i]) return 0;  // no stamp → not benchmarking
    }
    let ms = 0;
    for (let i = 0; i < STAMP_TIME_BITS; i++) {
        ms = ms * 2 + bitAt(STAMP_SYNC.length + i);
    }
    // ms is robot-clock; offset (robot − operator) brings it into operator time.
    const e2e = Date.now() - ms + state.clockOffsetMs;
    if (e2e < 0 || e2e > 5000) return 0;
    return +e2e.toFixed(1);
}

// getStats() lives only in the browser, so the operator samples the inbound
// track and reports health to the robot (rate/bitrate/loss = deltas between
// consecutive 1s samples). Robot folds it into report.json.
//
// getReport supplies the RTCStatsReport: defaults to the Cloudflare path's
// state.pc.getStats(); the LiveKit path passes its track receiver's getStats.
export function startVideoStats(channel, getReport = null) {
    state.videoStatsPrev = null;
    // Skip overlapping ticks — if getStats() blocks >1s, two concurrent
    // bodies race on videoStatsPrev and emit nonsense deltas.
    let inFlight = false;
    state.videoStatsTimer = setInterval(async () => {
        if (inFlight) return;
        if (!channel || channel.readyState !== 'open') return;
        if (!getReport && !state.pc) return;  // CF path needs the PC
        inFlight = true;
        let report = null;
        try {
            report = await (getReport ? getReport() : state.pc.getStats());
        } catch (_) {
            inFlight = false;
            return;
        }
        state.liveStats.iceType = selectedIceType(report);  // direct/stun/turn
        const inbound = findVideoInbound(report);
        if (!inbound) { inFlight = false; return; }

        const prev = state.videoStatsPrev;
        state.videoStatsPrev = inbound;
        // Stamp decode draws the video to a canvas — skip it on ticks that
        // can't produce a payload anyway (first sample).
        const payload = prev ? computeVideoStats(prev, inbound, readLatencyStamp()) : null;
        if (payload) {
            try { channel.send(JSON.stringify(payload)); } catch (_) {}
            state.liveStats.video = payload;  // latest sample for the HUD/VR quad
        }
        inFlight = false;
    }, VIDEO_STATS_INTERVAL_MS);
}

export function stopVideoStats() {
    if (state.videoStatsTimer) { clearInterval(state.videoStatsTimer); state.videoStatsTimer = null; }
    state.videoStatsPrev = null;
}

// ─── Operator liveness heartbeat ─────────────────────────────────────────
// Keeps broker's last_operator_heartbeat fresh so the reaper doesn't evict
// us. Covers silent drops (browser crash, iOS backgrounding) that pagehide
// misses. Stops on terminal auth/not-found — nothing to be gained by spam.
export function startOpHeartbeat(sessionId) {
    stopOpHeartbeat();
    // In-flight guard: fetch has no timeout, so a hung request would stack a
    // new POST every interval behind it.
    let inFlight = false;
    state.opHeartbeatTimer = setInterval(async () => {
        if (inFlight) return;
        inFlight = true;
        try {
            await api('POST', `/sessions/${sessionId}/op-heartbeat`);
        } catch (err) {
            const msg = err.message || '';
            if (msg === 'Unauthorized' || msg.startsWith('HTTP 4') || /Not the bound|Session not found/.test(msg)) {
                console.warn('[op-heartbeat] terminal:', msg);
                stopOpHeartbeat();
            }
        } finally {
            inFlight = false;
        }
    }, OP_HEARTBEAT_INTERVAL_MS);
}

export function stopOpHeartbeat() {
    if (state.opHeartbeatTimer) {
        clearInterval(state.opHeartbeatTimer);
        state.opHeartbeatTimer = null;
    }
}

export function handleStateMessage(data) {
    // Messages arrive as a string (pong, sent as str) OR an ArrayBuffer
    // (robot_telemetry/cmd_ack, published as bytes → CF delivers binary).
    // Decode binary to text before parsing, else JSON.parse throws and the
    // message is silently dropped — which is why battery never showed.
    if (data instanceof ArrayBuffer) {
        data = new TextDecoder().decode(data);
    } else if (ArrayBuffer.isView(data)) {
        data = new TextDecoder().decode(data.buffer);
    }
    let msg;
    try { msg = JSON.parse(data); } catch (_) { return; }
    if (msg.type === 'pong') applyPong(msg);
    // Robot-measured command-plane health (latency/jitter/loss) — what
    // actually arrived, which the operator can't see from its send side.
    else if (msg.type === 'robot_telemetry') {
        state.liveStats.cmd = msg.cmd;
        // Battery SOC rides robot_telemetry (state_reliable_back). null until
        // the robot's first lowstate; views read state.liveStats.soc.
        if (msg.soc != null) state.liveStats.soc = msg.soc;
        // Robot-authoritative UI state (posture/rage/OA/cams/estop) — the
        // active view registers state.onRobotState to reconcile its controls.
        // Absent on older robots; the hook is a no-op then.
        if (msg.state) state.onRobotState?.(msg.state);
    }
    // Command ack for a nonce'd command (body_height, sport_cmd, ...). The
    // active view registers state.onCmdAck to resolve its pending button/slider.
    else if (msg.type === 'cmd_ack') state.onCmdAck?.(msg);
}

// NTP-style min-RTT. Decay the floor ~1ms/s so a stale outlier doesn't pin
// clockOffsetMs forever — let the offset track link drift over minutes.
const BEST_RTT_DECAY_MS_PER_SEC = 1;
let _lastBestUpdateMs = 0;

function applyPong(pong) {
    const recvTsSec = Date.now() / 1000;
    const rttMs = (recvTsSec - pong.client_ts) * 1000;
    if (rttMs < 0 || rttMs > 5000) return;
    const nowMs = Date.now();
    if (_lastBestUpdateMs && Number.isFinite(state.bestRttMs)) {
        const dtSec = (nowMs - _lastBestUpdateMs) / 1000;
        state.bestRttMs += dtSec * BEST_RTT_DECAY_MS_PER_SEC;
    }
    if (rttMs >= state.bestRttMs) return;
    state.bestRttMs = rttMs;
    _lastBestUpdateMs = nowMs;
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
