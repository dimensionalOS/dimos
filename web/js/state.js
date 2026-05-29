// Shared mutable state — every other module imports from here.

export const state = {
    // Auth
    token: localStorage.getItem('teleop_token') || '',
    userEmail: localStorage.getItem('teleop_email') || '',
    brokerOverride: new URLSearchParams(window.location.search).get('broker') || '',

    // WebRTC + WebXR
    pc: null,
    cmdChannel: null,
    stateChannel: null,
    stateBackChannel: null,
    xrSession: null,
    xrRefSpace: null,
    gl: null,
    lastSendTime: 0,
    activeRobot: null,

    // Clock sync
    clockOffsetMs: 0,
    bestRttMs: Infinity,
    clockSyncBurstTimer: null,
    clockSyncDriftTimer: null,

    // Video stats
    videoStatsTimer: null,
    videoStatsPrev: null,

    // Keyboard
    kbInterval: null,
    kbKeys: new Set(),

    // Live metrics — single source of truth for the browser HUD + VR quad.
    // Nothing here is sent anywhere; pure local display state.
    liveStats: {
        video: null,        // latest video_stats payload
        rttMs: null,        // best clock-sync RTT (ms)
        offsetMs: 0,
        cmdHz: 0,           // command-send rate (twists for kb, poses for VR)
        cmd: null,          // robot-measured: {latency_ms, jitter_ms, loss_pct, rate_hz}
    },
    cmdSendCount: 0,        // rolling counter; sampled into cmdHz once/sec
    hudTimer: null,

    // VR stats quad (canvas → texture).
    statsCanvas: null,
    statsCtx: null,
    statsTex: null,
    statsBuf: null,

    // Video quad (WebGL).
    quadProgram: null,
    quadBuf: null,
    quadTex: null,
    quadUniforms: null,

    xrSupported: false,
};

// Dev-only: behave like a logged-in user on localhost.
const isLocalDev = ['localhost', '127.0.0.1', '0.0.0.0'].includes(window.location.hostname);
if (isLocalDev && !state.token) {
    state.token = 'dev-token';
    state.userEmail = 'dev@local';
}

export const sendInterval = 1000 / 80;
export const CLOCK_SYNC_BURST_COUNT = 5;
export const CLOCK_SYNC_BURST_INTERVAL_MS = 400;
export const CLOCK_SYNC_DRIFT_INTERVAL_MS = 30000;  // post-burst drift cadence
export const VIDEO_STATS_INTERVAL_MS = 1000;

// Probed once at boot; awaited by loadRobots() to pick the right Connect handler.
export const xrDetection = (async () => {
    if (!navigator.xr) return false;
    const ar = await navigator.xr.isSessionSupported('immersive-ar').catch(() => false);
    if (ar) { state.xrSupported = true; return true; }
    const vr = await navigator.xr.isSessionSupported('immersive-vr').catch(() => false);
    state.xrSupported = vr;
    return vr;
})();

export function escHtml(s) {
    return s ? String(s).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c])) : '';
}

export function timeAgo(iso) {
    const diff = Date.now() - new Date(iso).getTime();
    const mins = Math.floor(diff / 60000);
    if (mins < 1) return 'just now';
    if (mins < 60) return `${mins}m ago`;
    const hrs = Math.floor(mins / 60);
    if (hrs < 24) return `${hrs}h ago`;
    return `${Math.floor(hrs/24)}d ago`;
}
