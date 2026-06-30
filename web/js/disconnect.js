// Single teardown path — wraps every connect-time side effect.

import { api, brokerOrigin } from './api.js';
import { unmountHud } from './hud.js';
import { navigate } from './router.js';
import { state } from './state.js';
import { stopTick } from './views/go2.js';
import { stopKeyboardLoop } from './views/keyboard.js';
import { stopClockSync, stopVideoStats } from './webrtc.js';

export async function disconnect() {
    // Await leave so the broker clears operator_id before any re-connect —
    // fire-and-forget raced the next join into a 409 "already has operator".
    // Best-effort: still tear down locally if it fails.
    if (state.activeRobot && state.token) {
        try {
            await api('POST', `/sessions/${state.activeRobot.session_id}/leave`,
                { reason: 'user_initiated' });
        } catch (_) {}
    }

    stopKeyboardLoop();
    stopTick();  // go2 cockpit: clear telemetry tick + cmd-ack hook
    stopClockSync();
    stopVideoStats();
    unmountHud();
    if (state.xrSession) { await state.xrSession.end().catch(() => {}); state.xrSession = null; }
    state.xrRefSpace = null;  // belongs to the ended XR session; can't be reused
    if (state.cmdChannel) { try { state.cmdChannel.close(); } catch (_) {} state.cmdChannel = null; }
    if (state.stateChannel) { try { state.stateChannel.close(); } catch (_) {} state.stateChannel = null; }
    if (state.stateBackChannel) { try { state.stateBackChannel.close(); } catch (_) {} state.stateBackChannel = null; }
    const v = document.getElementById('robot-cam');
    if (v) {
        v.srcObject = null;
        // Remove the dynamically-created (VR) element; leave the keyboard
        // view's static one in place (it's re-rendered anyway).
        if (!v.parentElement || v.parentElement === document.body) v.remove();
        else v.style.display = 'none';
    }
    if (state.pc) { try { state.pc.close(); } catch (_) {} state.pc = null; }
    if (state.room) { try { state.room.disconnect(); } catch (_) {} state.room = null; }
    state.clockOffsetMs = 0;
    state.bestRttMs = Infinity;
    state.liveStats.video = null;
    state.liveStats.rttMs = null;
    state.liveStats.offsetMs = 0;
    state.liveStats.cmdHz = 0;
    state.liveStats.cmd = null;
    state.cmdSendCount = 0;

    const canvas = document.getElementById('canvas');
    canvas.style.display = 'none';
    state.activeRobot = null;
    navigate('dashboard');
}

// Tab close / reload bypasses the Disconnect button, so operator_id used to
// stick on the row forever (next dashboard visit showed it as "Busy" against
// our own session). Fire a best-effort /leave on pagehide using fetch with
// `keepalive: true` — the browser lets the request complete after the page
// unloads. sendBeacon would be simpler but can't set Authorization headers.
let _pagehideInstalled = false;

export function installPagehideLeave() {
    if (_pagehideInstalled) return;
    _pagehideInstalled = true;
    window.addEventListener('pagehide', () => {
        if (!state.activeRobot || !state.token) return;
        const url = `${brokerOrigin()}/api/v1/sessions/${state.activeRobot.session_id}/leave`;
        try {
            fetch(url, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'Authorization': `Bearer ${state.token}`,
                },
                body: JSON.stringify({ reason: 'pagehide' }),
                keepalive: true,
            }).catch(() => {});
        } catch (_) {
            // Some browsers throw synchronously on keepalive misuse — swallow.
        }
    });
}
