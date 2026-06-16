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

// Pick the operator transport by what the robot connected with. The robot's
// blueprint chose it; the broker surfaces it on the session (SessionInfo /
// join response). The two setups expose the same state.* surface downstream.
function setupTransport(sessionId, transport) {
    return transport === 'livekit' ? setupLiveKit(sessionId) : setupWebRTC(sessionId);
}

export async function connectToRobot(sessionId, robotName, transport) {
    state.activeRobot = { session_id: sessionId, robot_name: robotName };
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
    state.activeRobot = { session_id: sessionId, robot_name: robotName };
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
