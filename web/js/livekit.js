// LiveKit transport — operator side. Drop-in alternative to setupWebRTC for
// sessions whose robot connected with transport="livekit". Produces the same
// state.* surface (state.cmdChannel / state.stateChannel shims, robot-cam video)
// the rest of the app already drives, so keyboard.js / vr.js / hud.js / send()
// and clock-sync work unchanged.

import { api } from './api.js';
import { ensureRobotCam, setStatus } from './dom.js';
import { state } from './state.js';
import {
    handleStateMessage, startClockSync, startOpHeartbeat,
    startVideoStats, stopVideoStats, timeout,
} from './webrtc.js';

const CMD_TOPIC = 'cmd_unreliable';
const STATE_TOPIC = 'state_reliable';
const STATE_BACK_TOPIC = 'state_reliable_back';
const CONNECT_TIMEOUT_MS = 20000;

// Reused per packet on the hot data path — don't allocate per message.
const DEC = new TextDecoder();
const ENC = new TextEncoder();

function toU8(bytes) {
    if (bytes instanceof Uint8Array) return bytes;
    if (bytes instanceof ArrayBuffer) return new Uint8Array(bytes);
    if (ArrayBuffer.isView(bytes)) return new Uint8Array(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    return new Uint8Array(bytes);
}

export async function setupLiveKit(sessionId) {
    // Same re-entry guard as setupWebRTC — a double-click Connect would
    // otherwise spin up two Rooms that fight over state.*.
    if (state.setupInProgress) {
        throw new Error('Connect already in progress — disconnect first to retry');
    }
    state.setupInProgress = true;
    try {
        return await _setupLiveKitInner(sessionId);
    } finally {
        state.setupInProgress = false;
    }
}

async function _setupLiveKitInner(sessionId) {
    const LK = window.LivekitClient;
    if (!LK) throw new Error('LiveKit client SDK not loaded');
    setStatus('Connecting (LiveKit)...');

    // The broker mints a room-scoped token; LiveKit handles ICE/TURN itself, so
    // there is no SDP exchange or turn-credentials fetch here (unlike WebRTC).
    const data = await api('POST', `/sessions/${sessionId}/join`, { role: 'operator' });
    if (!data.url || !data.token) throw new Error('Broker did not return LiveKit url/token');

    // adaptiveStream downgrades video whose attached element is hidden/zero-size
    // — exactly the VR GL-texture <video>. Teleop wants full rate, so keep off.
    const room = new LK.Room({ adaptiveStream: false });
    state.room = room;

    // Robot camera track → the shared <video> element (same one the WebRTC path
    // feeds via pc.ontrack), so keyboard view + VR GL texture pick it up as-is.
    // The track's RTCRtpReceiver also feeds the video-stats sampler — the SDK
    // owns the PeerConnection, so state.pc.getStats() doesn't exist here.
    let videoTrack = null;
    const maybeStartStats = () => {
        const receiver = videoTrack?.receiver;
        if (!receiver || !state.stateChannel) return;
        stopVideoStats();
        startVideoStats(state.stateChannel, () => receiver.getStats());
    };
    room.on(LK.RoomEvent.TrackSubscribed, (track) => {
        if (track.kind !== 'video') return;
        const existed = !!document.getElementById('robot-cam');
        const v = ensureRobotCam();
        // Detach the prior track before rebinding — on a robot republish this
        // fires again, and the old track + its receiver (still held by the
        // stats sampler) would otherwise leak.
        if (videoTrack && videoTrack !== track) {
            try { videoTrack.detach(v); } catch (_) {}
        }
        track.attach(v);
        if (existed) v.style.display = 'block';
        v.play?.().catch(() => {});
        videoTrack = track;
        maybeStartStats();  // restarts the sampler against the new receiver
    });

    // Robot replies on the back channel by protocol; LiveKit never echoes our
    // own published data, so the forward topic carries nothing inbound.
    room.on(LK.RoomEvent.DataReceived, (payload, _participant, _kind, topic) => {
        if (topic === STATE_BACK_TOPIC) {
            handleStateMessage(DEC.decode(payload));
        }
    });

    // Drop refs on terminal disconnect so send() doesn't fire into a dead
    // Room and a fresh setupLiveKit doesn't skip teardown.
    room.on(LK.RoomEvent.Disconnected, () => {
        console.info('[livekit] room disconnected');
        if (state.room === room) {
            stopVideoStats();  // sampler holds this room's track receiver
            state.room = null;
            state.cmdChannel = null;
            state.stateChannel = null;
        }
    });

    // On connect failure/timeout, tear down the half-open Room — otherwise it
    // keeps reconnecting in the background and a retry spins up a second Room.
    try {
        await Promise.race([
            room.connect(data.url, data.token),
            timeout(CONNECT_TIMEOUT_MS, 'Timed out connecting to LiveKit'),
        ]);
    } catch (err) {
        await room.disconnect().catch(() => {});
        if (state.room === room) state.room = null;
        throw err;
    }

    // Shim the outbound DataChannels onto LiveKit topics: send()/startClockSync
    // only touch .readyState + .send(). readyState tracks room.state so callers
    // stop sending mid-reconnect; .close() is a no-op (disconnect tears down);
    // publishData rejections are swallowed (fire-and-forget).
    const lp = room.localParticipant;
    const publish = (bytes, opts) => lp.publishData(bytes, opts).catch(() => {});
    const isOpen = () => (room.state === 'connected' ? 'open' : 'closed');
    state.cmdChannel = {
        get readyState() { return isOpen(); },
        send: (bytes) => publish(toU8(bytes), { reliable: false, topic: CMD_TOPIC }),
        close: () => {},
    };
    state.stateChannel = {
        get readyState() { return isOpen(); },
        send: (txt) => publish(ENC.encode(txt), { reliable: true, topic: STATE_TOPIC }),
        close: () => {},
    };

    startClockSync(state.stateChannel);
    startOpHeartbeat(sessionId);
    // Video stats: sampled off the subscribed track's RTCRtpReceiver (the SDK
    // owns the PC). TrackSubscribed may fire before or after this point —
    // whichever side runs second actually starts the sampler.
    maybeStartStats();
}
