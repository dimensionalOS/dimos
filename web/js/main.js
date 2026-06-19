// Entry point: register views with the router, pick the initial route, and
// wire the DevTools preview hook.

import { navigate, register } from './router.js';
import { state } from './state.js';
import { renderAuth } from './views/auth.js';
import { renderDashboard } from './views/dashboard.js';
import { renderGo2 } from './views/go2.js';
import { renderKeyboard } from './views/keyboard.js';
import { renderTeleop } from './views/teleop.js';

register('auth', renderAuth);
register('dashboard', renderDashboard);
register('go2', renderGo2);
register('keyboard', renderKeyboard);
register('teleop', renderTeleop);

if (state.token) navigate('dashboard');
else navigate('auth');

// DevTools-only preview hook — no broker required.
window._teleopDev = {
    previewKeyboard() {
        state.cmdChannel = { readyState: 'open', send: () => {} };
        state.activeRobot = { session_id: 'preview', robot_name: 'Preview Bot' };
        navigate('keyboard');
    },
    previewVR() {
        state.activeRobot = { session_id: 'preview', robot_name: 'Preview Bot' };
        navigate('teleop');
    },
    previewGo2() {
        state.activeRobot = { session_id: 'preview', robot_name: 'go2-preview' };
        navigate('go2');
    },
    navigate,
};
