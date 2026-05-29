// REST helpers + auth lifecycle.

import { state } from './state.js';
import { navigate } from './router.js';

export function brokerOrigin() {
    return state.brokerOverride || window.location.origin;
}

export async function api(method, path, body = null) {
    const opts = { method, headers: { 'Content-Type': 'application/json' } };
    if (state.token) opts.headers['Authorization'] = `Bearer ${state.token}`;
    if (body) opts.body = JSON.stringify(body);
    const res = await fetch(`${brokerOrigin()}/api/v1${path}`, opts);
    if (res.status === 401) { logout(); throw new Error('Unauthorized'); }
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.detail || `HTTP ${res.status}`);
    return data;
}

export function logout() {
    localStorage.removeItem('teleop_token');
    localStorage.removeItem('teleop_email');
    state.token = '';
    state.userEmail = '';
    navigate('auth');
}
