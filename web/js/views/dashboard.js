// Dashboard: API key management + available robots list.

import { api, brokerOrigin, logout } from '../api.js';
import { connectKeyboard, connectToRobot } from '../connect.js';
import { escHtml, state, timeAgo, xrDetection } from '../state.js';

export async function renderDashboard(c) {
    c.innerHTML = `
    <div class="max-w-4xl mx-auto p-6 fade-in">
        <header class="flex items-center justify-between mb-8">
            <div>
                <h1 class="text-2xl font-bold text-white">Dashboard</h1>
                <p class="text-gray-400 text-sm">${escHtml(state.userEmail)}</p>
            </div>
            <button id="logoutBtn" class="px-4 py-2 text-sm text-gray-400 hover:text-white border border-[#2a2a2a] rounded-lg hover:border-[#3a3a3a] transition-colors">
                Log Out
            </button>
        </header>

        <!-- API Keys -->
        <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl p-6 mb-6">
            <div class="flex items-center justify-between mb-4">
                <div>
                    <h2 class="text-lg font-semibold text-white">API Keys</h2>
                    <p class="text-gray-400 text-sm">Authenticate your robots with the teleop service</p>
                </div>
                <button id="newKeyBtn" class="px-4 py-2 bg-dim-500 hover:bg-dim-600 text-bg-950 text-sm font-medium rounded-lg transition-colors">
                    + New Key
                </button>
            </div>

            <div id="create-key-form" class="hidden mb-4 p-4 bg-[#1f1f1f] rounded-lg border border-[#2a2a2a]">
                <div class="grid grid-cols-1 md:grid-cols-2 gap-3 mb-3">
                    <input id="key-name" type="text" placeholder="Key name (e.g. Lab Robot 01)"
                        class="px-3 py-2 bg-bg-950 border border-[#2a2a2a] rounded-lg text-white placeholder-gray-500 text-sm focus:outline-none focus:ring-2 focus:ring-dim-400">
                    <input id="key-robot-id" type="text" placeholder="Robot ID (optional)"
                        class="px-3 py-2 bg-bg-950 border border-[#2a2a2a] rounded-lg text-white placeholder-gray-500 text-sm focus:outline-none focus:ring-2 focus:ring-dim-400">
                </div>
                <div class="flex gap-2">
                    <button id="generateKeyBtn" class="px-4 py-2 bg-dim-500 hover:bg-dim-600 text-bg-950 text-sm rounded-lg transition-colors">Generate Key</button>
                    <button id="cancelKeyBtn" class="px-4 py-2 text-gray-400 hover:text-white text-sm rounded-lg transition-colors">Cancel</button>
                </div>
            </div>

            <div id="new-key-reveal" class="hidden mb-4 p-4 bg-green-900/20 border border-green-800 rounded-lg">
                <p class="text-green-400 text-sm font-medium mb-2">🔑 Key created! Copy it now — it won't be shown again.</p>
                <div class="flex items-center gap-2">
                    <code id="new-key-value" class="key-reveal flex-1 px-3 py-2 bg-bg-950 rounded text-green-300 text-sm"></code>
                    <button id="copyKeyBtn" class="px-3 py-2 bg-[#2a2a2a] hover:bg-[#3a3a3a] rounded text-sm text-white transition-colors">Copy</button>
                </div>
                <p class="text-gray-500 text-xs mt-2">Use it as <code>broker_api_key</code> when starting <code>HostedTeleopModule</code>.</p>
            </div>

            <div id="keys-list" class="space-y-2">
                <div class="text-gray-500 text-sm py-4 text-center">Loading...</div>
            </div>
        </section>

        <!-- Connected robots -->
        <section class="bg-bg-950 border border-[#2a2a2a] rounded-xl p-6">
            <div class="flex items-center justify-between mb-4">
                <div>
                    <h2 class="text-lg font-semibold text-white">Available Robots</h2>
                    <p class="text-gray-400 text-sm">Robots online — click Connect to teleoperate</p>
                </div>
                <button id="refreshRobotsBtn" class="px-4 py-2 text-sm text-gray-300 border border-[#2a2a2a] rounded-lg hover:bg-[#1f1f1f] transition-colors">
                    Refresh
                </button>
            </div>
            <div id="robots-list" class="space-y-2">
                <div class="text-gray-500 text-sm py-4 text-center">Loading...</div>
            </div>
        </section>

        <p class="text-center text-xs text-gray-600 mt-6">
            Broker: <code>${escHtml(brokerOrigin())}</code>
        </p>
    </div>`;

    document.getElementById('logoutBtn').onclick = logout;
    document.getElementById('newKeyBtn').onclick = () => document.getElementById('create-key-form').classList.remove('hidden');
    document.getElementById('cancelKeyBtn').onclick = () => document.getElementById('create-key-form').classList.add('hidden');
    document.getElementById('generateKeyBtn').onclick = createKey;
    document.getElementById('copyKeyBtn').onclick = copyKey;
    document.getElementById('refreshRobotsBtn').onclick = loadRobots;

    await Promise.all([loadKeys(), loadRobots()]);
}

async function createKey() {
    const name = document.getElementById('key-name').value.trim();
    const robotId = document.getElementById('key-robot-id').value.trim() || null;
    if (!name) { alert('Please provide a key name'); return; }
    try {
        const data = await api('POST', '/keys', { name, robot_id: robotId });
        document.getElementById('new-key-value').textContent = data.api_key;
        document.getElementById('new-key-reveal').classList.remove('hidden');
        document.getElementById('create-key-form').classList.add('hidden');
        document.getElementById('key-name').value = '';
        document.getElementById('key-robot-id').value = '';
        await loadKeys();
    } catch (err) {
        alert('Failed to create key: ' + err.message);
    }
}

function copyKey() {
    const key = document.getElementById('new-key-value').textContent;
    navigator.clipboard.writeText(key);
    const btn = document.getElementById('copyKeyBtn');
    btn.textContent = 'Copied!';
    setTimeout(() => { btn.textContent = 'Copy'; }, 2000);
}

async function loadKeys() {
    const listEl = document.getElementById('keys-list');
    try {
        const data = await api('GET', '/keys');
        if (!data.keys || data.keys.length === 0) {
            listEl.innerHTML = '<p class="text-gray-500 text-sm py-4 text-center">No API keys yet. Create one to connect a robot.</p>';
            return;
        }
        listEl.innerHTML = data.keys.map(k => `
            <div class="flex items-center justify-between p-3 bg-[#1f1f1f] rounded-lg">
                <div class="flex-1 min-w-0">
                    <div class="flex items-center gap-2">
                        <span class="text-white text-sm font-medium">${escHtml(k.name)}</span>
                        ${k.robot_id ? `<span class="text-xs px-2 py-0.5 bg-[#2a2a2a] rounded text-gray-300">${escHtml(k.robot_id)}</span>` : ''}
                    </div>
                    <div class="flex items-center gap-3 mt-1">
                        <code class="text-gray-400 text-xs">${escHtml(k.key_prefix)}...</code>
                        <span class="text-gray-600 text-xs">${k.last_used_at ? 'Last used ' + timeAgo(k.last_used_at) : 'Never used'}</span>
                    </div>
                </div>
                <button data-id="${k.id}" data-name="${escHtml(k.name)}"
                    class="revoke-btn px-3 py-1.5 text-xs text-red-400 hover:text-red-300 hover:bg-red-900/20 rounded transition-colors">
                    Revoke
                </button>
            </div>
        `).join('');
        listEl.querySelectorAll('.revoke-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                revokeKey(e.target.dataset.id, e.target.dataset.name);
            });
        });
    } catch (err) {
        listEl.innerHTML = `<p class="text-red-400 text-sm">${escHtml(err.message)}</p>`;
    }
}

async function revokeKey(id, name) {
    if (!confirm(`Revoke key "${name}"? This cannot be undone.`)) return;
    try {
        await api('DELETE', `/keys/${id}`);
        document.getElementById('new-key-reveal').classList.add('hidden');
        await loadKeys();
    } catch (err) {
        alert('Failed to revoke: ' + err.message);
    }
}

async function loadRobots() {
    const listEl = document.getElementById('robots-list');
    await xrDetection;
    try {
        const robots = await api('GET', '/sessions');
        // Broker returns a flat list per sessions.py:list_sessions response_model.
        const arr = Array.isArray(robots) ? robots : (robots.sessions || []);
        if (arr.length === 0) {
            listEl.innerHTML = '<p class="text-gray-500 text-sm py-4 text-center">No robots online. Start a robot with a registered API key.</p>';
            return;
        }
        listEl.innerHTML = arr.map(s => `
            <div class="flex items-center justify-between p-3 bg-[#1f1f1f] rounded-lg">
                <div class="flex items-center gap-3">
                    <div class="w-2 h-2 rounded-full ${s.state === 'active' ? 'bg-green-400' : s.state === 'idle' ? 'bg-yellow-400' : 'bg-gray-500'}"></div>
                    <div>
                        <span class="text-white text-sm font-medium">${escHtml(s.robot_name)}</span>
                        <span class="text-gray-500 text-xs ml-2">${escHtml(s.robot_id)}</span>
                        ${s.rtt_ms ? `<span class="text-gray-500 text-xs ml-2">${Math.round(s.rtt_ms)}ms</span>` : ''}
                    </div>
                </div>
                <button data-id="${s.session_id}" data-name="${escHtml(s.robot_name)}"
                    class="connect-btn px-4 py-2 bg-dim-500 hover:bg-dim-600 text-bg-950 text-sm rounded-lg transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                    ${s.state === 'active' ? 'disabled' : ''}>
                    ${s.state === 'active' ? 'Busy' : 'Connect'}
                </button>
            </div>
        `).join('');
        const handler = state.xrSupported ? connectToRobot : connectKeyboard;
        listEl.querySelectorAll('.connect-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                handler(e.target.dataset.id, e.target.dataset.name);
            });
        });
    } catch (err) {
        listEl.innerHTML = `<p class="text-red-400 text-sm py-4 text-center">${escHtml(err.message)}</p>`;
    }
}
