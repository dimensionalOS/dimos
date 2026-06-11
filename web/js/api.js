// REST helpers + auth lifecycle.

import { state } from './state.js';
import { navigate } from './router.js';
import { refreshTokens, tokenExpired } from './cognito.js';

export function brokerOrigin() {
    return state.brokerOverride || window.location.origin;
}

let refreshInFlight = null;

// Refresh the ID token if it's expired/near expiry. Single-flight so a burst
// of parallel api() calls doesn't fire N refresh requests.
async function ensureFreshToken() {
    if (!state.token || !state.refreshToken) return;
    if (!tokenExpired(state.token)) return;
    refreshInFlight ??= (async () => {
        try {
            const result = await refreshTokens(state.refreshToken);
            state.token = result.IdToken;
            localStorage.setItem('teleop_token', state.token);
        } catch {
            logout();
            throw new Error('Session expired — log in again');
        } finally {
            refreshInFlight = null;
        }
    })();
    await refreshInFlight;
}

export async function api(method, path, body = null) {
    await ensureFreshToken();
    const opts = { method, headers: { 'Content-Type': 'application/json' } };
    if (state.token) opts.headers['Authorization'] = `Bearer ${state.token}`;
    if (body) opts.body = JSON.stringify(body);
    const res = await fetch(`${brokerOrigin()}/api/v1${path}`, opts);
    if (res.status === 401) { logout(); throw new Error('Unauthorized'); }
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.detail || `HTTP ${res.status}`);
    return data;
}

export function setSession(idToken, refreshToken, email) {
    state.token = idToken;
    state.refreshToken = refreshToken || '';
    state.userEmail = email;
    localStorage.setItem('teleop_token', state.token);
    localStorage.setItem('teleop_refresh', state.refreshToken);
    localStorage.setItem('teleop_email', state.userEmail);
}

export function logout() {
    localStorage.removeItem('teleop_token');
    localStorage.removeItem('teleop_refresh');
    localStorage.removeItem('teleop_email');
    state.token = '';
    state.refreshToken = '';
    state.userEmail = '';
    navigate('auth');
}
