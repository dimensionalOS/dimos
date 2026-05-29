// Connect handlers — called from the Connect button (must run inside a user
// gesture; that's why VR session creation is started immediately, not after
// the WebRTC round-trip).

import { setStatus } from './dom.js';
import { mountHud } from './hud.js';
import { navigate } from './router.js';
import { state } from './state.js';
import { startKeyboardLoop } from './views/keyboard.js';
import { startVR } from './vr.js';
import { setupWebRTC } from './webrtc.js';

export async function connectToRobot(sessionId, robotName) {
    state.activeRobot = { session_id: sessionId, robot_name: robotName };
    try {
        if (!navigator.xr) throw new Error('WebXR not supported. Use Quest 3 browser.');

        navigate('teleop');
        // VR FIRST — gesture activation must be alive when requestSession() runs.
        await startVR();

        try {
            await setupWebRTC(sessionId);
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

export async function connectKeyboard(sessionId, robotName) {
    state.activeRobot = { session_id: sessionId, robot_name: robotName };
    try {
        navigate('keyboard');
        await setupWebRTC(sessionId);
        setStatus(`Connected — ${robotName}`);
        startKeyboardLoop();
        mountHud();  // always-on metrics pill (browser view)
    } catch (e) {
        console.error(e);
        setStatus(`Connection failed: ${e.message}`);
        setTimeout(() => navigate('dashboard'), 3000);
    }
}
