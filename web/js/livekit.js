// LiveKit transport — operator side. Drop-in alternative to setupWebRTC for
// sessions whose robot connected with transport="livekit". Produces the same
// state.* surface (state.cmdChannel / state.stateChannel shims, robot-cam video)
// the rest of the app already drives, so keyboard.js / vr.js / hud.js / send()
// and clock-sync work unchanged.

import { api } from './api.js';
import { ensureRobotCam, setStatus } from './dom.js';
import { state } from './state.js';
import { startClockSync, handleStateMessage } from './webrtc.js';

const CMD_TOPIC = 'cmd_unreliable';
const STATE_TOPIC = 'state_reliable';
const STATE_BACK_TOPIC = 'state_reliable_back';
const CONNECT_TIMEOUT_MS = 20000;

function timeout(ms, label) {
    return new Promise((_, reject) => setTimeout(() => reject(new Error(label)), ms));
}

function toU8(bytes) {
    if (bytes instanceof Uint8Array) return bytes;
    if (bytes instanceof ArrayBuffer) return new Uint8Array(bytes);
    if (ArrayBuffer.isView(bytes)) return new Uint8Array(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    return new Uint8Array(bytes);
}

export async function setupLiveKit(sessionId) {
    const LK = window.LivekitClient;
    if (!LK) throw new Error('LiveKit client SDK not loaded');
    setStatus('Connecting (LiveKit)...');

    // The broker mints a room-scoped token; LiveKit handles ICE/TURN itself, so
    // there is no SDP exchange or turn-credentials fetch here (unlike WebRTC).
    const data = await api('POST', `/sessions/${sessionId}/join`, { role: 'operator' });
    if (!data.url || !data.token) throw new Error('Broker did not return LiveKit url/token');

    const room = new LK.Room({ adaptiveStream: true, dynacast: true });
    state.room = room;

    // Robot camera track → the shared <video> element (same one the WebRTC path
    // feeds via pc.ontrack), so keyboard view + VR GL texture pick it up as-is.
    room.on(LK.RoomEvent.TrackSubscribed, (track) => {
        if (track.kind !== 'video') return;
        const existed = !!document.getElementById('robot-cam');
        const v = ensureRobotCam();
        track.attach(v);
        if (existed) v.style.display = 'block';
        v.play?.().catch(() => {});
    });

    // Robot → operator messages arrive as topic-tagged data packets; route the
    // reverse channel into the same JSON handler the WebRTC path uses.
    room.on(LK.RoomEvent.DataReceived, (payload, _participant, _kind, topic) => {
        if (topic === STATE_BACK_TOPIC) {
            handleStateMessage(new TextDecoder().decode(payload));
        }
    });

    room.on(LK.RoomEvent.Disconnected, () => console.info('[livekit] room disconnected'));

    await Promise.race([
        room.connect(data.url, data.token),
        timeout(CONNECT_TIMEOUT_MS, 'Timed out connecting to LiveKit'),
    ]);

    // Shim the two outbound DataChannels onto LiveKit topics. send() (webrtc.js)
    // and startClockSync only touch .readyState + .send(), so these stand in
    // transparently; .close() is a no-op (room teardown happens in disconnect).
    const lp = room.localParticipant;
    state.cmdChannel = {
        readyState: 'open',
        send: (bytes) => lp.publishData(toU8(bytes), { reliable: false, topic: CMD_TOPIC }),
        close: () => {},
    };
    state.stateChannel = {
        readyState: 'open',
        send: (txt) => lp.publishData(new TextEncoder().encode(txt), { reliable: true, topic: STATE_TOPIC }),
        close: () => {},
    };

    startClockSync(state.stateChannel);
    // Video-stats reporter is skipped on LiveKit: it samples state.pc.getStats(),
    // which the SDK owns internally. (HUD video health is a follow-up.)
}
