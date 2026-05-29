// Login / register view.

import { api } from '../api.js';
import { navigate } from '../router.js';
import { escHtml, state } from '../state.js';

let authMode = 'login';

export function renderAuth(c) {
    c.innerHTML = `
    <div class="min-h-screen flex items-center justify-center p-4">
        <div class="w-full max-w-md">
            <div class="text-center mb-8">
                <h1 class="text-3xl font-bold text-white mb-2">DimOS Teleop</h1>
                <p class="text-gray-400">Remote robot teleoperation platform</p>
            </div>
            <div class="bg-bg-950 border border-[#2a2a2a] rounded-xl p-6 shadow-xl">
                <div class="flex mb-6 bg-[#1f1f1f] rounded-lg p-1">
                    <button id="tab-login" class="flex-1 py-2 px-4 rounded-md text-sm font-medium bg-dim-500 text-bg-950">Log In</button>
                    <button id="tab-register" class="flex-1 py-2 px-4 rounded-md text-sm font-medium text-gray-400 hover:text-white">Sign Up</button>
                </div>
                <form id="auth-form">
                    <div class="space-y-4">
                        <div>
                            <label class="block text-sm font-medium text-gray-300 mb-1">Email</label>
                            <input id="email" type="email" required value="${escHtml(state.userEmail)}"
                                class="w-full px-4 py-2.5 bg-[#1f1f1f] border border-[#2a2a2a] rounded-lg text-white placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-dim-400"
                                placeholder="you@company.com">
                        </div>
                        <div>
                            <label class="block text-sm font-medium text-gray-300 mb-1">Password</label>
                            <input id="password" type="password" required
                                class="w-full px-4 py-2.5 bg-[#1f1f1f] border border-[#2a2a2a] rounded-lg text-white placeholder-gray-500 focus:outline-none focus:ring-2 focus:ring-dim-400"
                                placeholder="••••••••">
                        </div>
                        <div id="auth-error" class="text-red-400 text-sm hidden"></div>
                        <button type="submit" class="w-full py-2.5 bg-dim-500 hover:bg-dim-600 text-bg-950 font-medium rounded-lg transition-colors">
                            <span id="auth-btn-text">Log In</span>
                        </button>
                    </div>
                </form>
            </div>
            <p class="text-center text-gray-500 text-sm mt-6">
                Powered by <a href="https://dimensionalos.com" class="text-dim-500 hover:text-dim-400">DimensionalOS</a>
            </p>
        </div>
    </div>`;

    document.getElementById('tab-login').onclick = () => showAuthTab('login');
    document.getElementById('tab-register').onclick = () => showAuthTab('register');
    document.getElementById('auth-form').onsubmit = handleAuth;
}

function showAuthTab(mode) {
    authMode = mode;
    const active = 'flex-1 py-2 px-4 rounded-md text-sm font-medium bg-dim-500 text-bg-950';
    const inactive = 'flex-1 py-2 px-4 rounded-md text-sm font-medium text-gray-400 hover:text-white';
    document.getElementById('tab-login').className = mode === 'login' ? active : inactive;
    document.getElementById('tab-register').className = mode === 'register' ? active : inactive;
    document.getElementById('auth-btn-text').textContent = mode === 'login' ? 'Log In' : 'Create Account';
}

async function handleAuth(e) {
    e.preventDefault();
    const email = document.getElementById('email').value.trim();
    const password = document.getElementById('password').value;
    const errEl = document.getElementById('auth-error');
    errEl.classList.add('hidden');
    try {
        const endpoint = authMode === 'login' ? '/auth/login' : '/auth/register';
        const data = await api('POST', endpoint, { email, password });
        state.token = data.token;
        state.userEmail = data.user_id;
        localStorage.setItem('teleop_token', state.token);
        localStorage.setItem('teleop_email', state.userEmail);
        navigate('dashboard');
    } catch (err) {
        errEl.textContent = err.message;
        errEl.classList.remove('hidden');
    }
}
