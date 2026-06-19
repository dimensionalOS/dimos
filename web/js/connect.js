// Connect handlers — called from the Connect button (must run inside a user
// gesture; that's why VR session creation is started immediately, not after
// the WebRTC round-trip).

import { setStatus } from './dom.js';
import { mountHud } from './hud.js';
import { setupLiveKit } from './livekit.js';
import { navigate } from './router.js';
import { state } from './state.js';
import { startKeyboardLoop } from './views/keyboard.js';
import { startVR } from './vr.js';
import { setupWebRTC } from './webrtc.js';

// Operator transport follows what the robot connected with (broker surfaces it).
function setupTransport(sessionId, transport) {
    return transport === 'livekit' ? setupLiveKit(sessionId) : setupWebRTC(sessionId);
}

export async function connectToRobot(sessionId, robotName, transport) {
    state.activeRobot = { session_id: sessionId, robot_name: robotName, transport: transport || 'cloudflare' };
    try {
        if (!navigator.xr) throw new Error('WebXR not supported. Use Quest 3 browser.');

        navigate('teleop');
        // VR FIRST — gesture activation must be alive when requestSession() runs.
        await startVR();

        try {
            await setupTransport(sessionId, transport);
        } catch (rtcError) {
            if (state.xrSession) { await state.xrSession.end().catch(() => {}); state.xrSession = null; }
            throw rtcError;
        }
        setStatus(`Connected — ${robotName}`);
    } catch (e) {
        console.error(e);
        setStatus(`Connection failed: ${e.message}`);
        setTimeout(() => navigate('dashboard'), 3000);
    }
}

export async function connectKeyboard(sessionId, robotName, transport) {
    state.activeRobot = { session_id: sessionId, robot_name: robotName, transport: transport || 'cloudflare' };
    try {
        navigate('keyboard');
        await setupTransport(sessionId, transport);
        setStatus(`Connected — ${robotName}`);
        startKeyboardLoop();
        mountHud();  // always-on metrics pill (browser view)
    } catch (e) {
        console.error(e);
        setStatus(`Connection failed: ${e.message}`);
        setTimeout(() => navigate('dashboard'), 3000);
    }
}

// Go2 cockpit view. Same transport/drive path as connectKeyboard — the go2
// view starts the keyboard loop itself on render, so we don't start it here.
export async function connectGo2(sessionId, robotName, transport) {
    state.activeRobot = { session_id: sessionId, robot_name: robotName, transport: transport || 'cloudflare' };
    try {
        navigate('go2');  // renderGo2() runs startKeyboardLoop()
        await setupTransport(sessionId, transport);
        setStatus(`Connected — ${robotName}`);
    } catch (e) {
        console.error(e);
        setStatus(`Connection failed: ${e.message}`);
        setTimeout(() => navigate('dashboard'), 3000);
    }
}
