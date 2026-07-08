// Entry point: register views with the router, pick the initial route, and
// wire the DevTools preview hook.

import { installPagehideLeave } from './disconnect.js';
import { navigate, register } from './router.js';
import { state } from './state.js';
import { renderAuth } from './views/auth.js';
import { renderDashboard } from './views/dashboard.js';
import { renderGo2 } from './views/go2.js';
import { renderKeyboard } from './views/keyboard.js';
import { renderTeleop } from './views/teleop.js';
import { renderVRPreview } from './vrpreview.js';

register('auth', renderAuth);
register('dashboard', renderDashboard);
register('go2', renderGo2);
register('keyboard', renderKeyboard);
register('teleop', renderTeleop);
register('vrpreview', renderVRPreview);

installPagehideLeave();

// #vrpreview: headset UI check with faked data — no auth, no broker, no robot.
if (location.hash === '#vrpreview') navigate('vrpreview');
else if (state.token) navigate('dashboard');
else navigate('auth');

// DevTools-only preview hooks — no broker required. (VR preview lives at the
// #vrpreview route, which fakes channels + map + video; see vrpreview.js.)
window._teleopDev = {
    previewKeyboard() {
        state.cmdChannel = { readyState: 'open', send: () => {} };
        state.activeRobot = { session_id: 'preview', robot_name: 'Preview Bot' };
        navigate('keyboard');
    },
    previewGo2() {
        state.activeRobot = { session_id: 'preview', robot_name: 'go2-preview' };
        navigate('go2');
    },
    async previewXArm() {
        // Immersive arm cockpit against fake channels — no broker/robot. Must be
        // called from a click (WebXR requestSession needs a user gesture).
        const { startArmVR } = await import('./vrarm.js');
        state.activeRobot = { session_id: 'preview', robot_name: 'xarm-preview', transport: 'cloudflare' };
        state.cmdChannel = { readyState: 'open', send: () => {} };
        state.stateChannel = { readyState: 'open', send: () => {} };
        navigate('teleop');
        await startArmVR();
    },
    navigate,
};
