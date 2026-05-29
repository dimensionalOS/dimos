// VR landing view — the actual scene renders into <canvas> via WebXR; this
// is just the loading screen + Disconnect button.

import { disconnect } from '../disconnect.js';
import { escHtml, state } from '../state.js';

export function renderTeleop(c) {
    c.innerHTML = `
    <div class="min-h-screen flex flex-col items-center justify-center p-6 fade-in">
        <div class="w-full max-w-md text-center">
            <h1 class="text-3xl font-bold text-white mb-2">${escHtml(state.activeRobot?.robot_name || 'Teleop')}</h1>
            <div id="teleop-status" class="text-lg text-gray-300 px-4 py-3 bg-bg-950 border border-[#2a2a2a] rounded-lg my-4">
                Negotiating...
            </div>
            <button id="disconnectBtn" class="mt-4 px-6 py-2.5 bg-[#2a2a2a] hover:bg-[#3a3a3a] text-white text-sm font-medium rounded-lg transition-colors">
                Disconnect
            </button>
        </div>
    </div>`;
    document.getElementById('disconnectBtn').onclick = disconnect;
}
