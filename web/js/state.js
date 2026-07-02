// Shared mutable state — every other module imports from here.

// ?broker= is dev-only — in prod it would let a phishing link redirect every
// api() call (and the Cognito bearer token) to an attacker-controlled origin.
const isLocalDev = ['localhost', '127.0.0.1', '0.0.0.0'].includes(window.location.hostname);
const brokerParam = new URLSearchParams(window.location.search).get('broker') || '';

export const state = {
    // Auth (Cognito ID + refresh tokens)
    token: localStorage.getItem('teleop_token') || '',
    refreshToken: localStorage.getItem('teleop_refresh') || '',
    userEmail: localStorage.getItem('teleop_email') || '',
    brokerOverride: isLocalDev ? brokerParam : '',

    // WebRTC + WebXR
    setupInProgress: false,  // setupWebRTC re-entry guard
    pc: null,
    room: null,          // LiveKit Room (livekit transport only; null for cloudflare)
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

    // Operator liveness heartbeat to broker
    opHeartbeatTimer: null,

    // Keyboard
    kbInterval: null,
    kbKeys: new Set(),

    // Speed bar (go2 cockpit): {lin, ang} multipliers applied in buildTwist.
    // Normal 0.5 / High 1.0 m/s envelope; Rage uses firmware mode + full scale.
    speedScale: { lin: 0.5, ang: 0.5 },

    // Live metrics — single source of truth for the browser HUD + VR quad.
    // Nothing here is sent anywhere; pure local display state.
    liveStats: {
        video: null,        // latest video_stats payload
        rttMs: null,        // best clock-sync RTT (ms)
        offsetMs: 0,
        cmdHz: 0,           // command-send rate (twists for kb, poses for VR)
        cmd: null,          // robot-measured: {latency_ms, jitter_ms, rate_hz, throughput_bps}
        soc: null,          // robot battery state-of-charge (%), from robot_telemetry
        iceType: null,      // selected ICE path: 'direct' | 'stun' | 'turn' | null
    },
    onCmdAck: null,         // optional view hook: (msg) => void for {type:cmd_ack,nonce,ok}
    driveEnabled: true,     // gates WASD; go2 cockpit sets false until Stand/Drive
    // Video-freshness drive gate (stall.js): stalled drives the overlay/HUD,
    // blocked suppresses twist sends (stall OR post-stall neutral gate).
    videoStall: { stalled: false, blocked: false, armed: false },
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
if (isLocalDev && !state.token) {
    state.token = 'dev-token';
    state.userEmail = 'dev@local';
}

export const sendInterval = 1000 / 80;
export const CLOCK_SYNC_BURST_COUNT = 5;
export const CLOCK_SYNC_BURST_INTERVAL_MS = 400;
export const CLOCK_SYNC_DRIFT_INTERVAL_MS = 30000;  // post-burst drift cadence
export const VIDEO_STATS_INTERVAL_MS = 1000;
export const OP_HEARTBEAT_INTERVAL_MS = 5000;  // matches broker's ~20s reap window

// Probed once at boot; awaited by loadRobots() to pick the right Connect handler.
// immersive-vr only: headsets (Quest) support it, phones don't
export const xrDetection = (async () => {
    if (!navigator.xr) return false;
    state.xrSupported = await navigator.xr.isSessionSupported('immersive-vr').catch(() => false);
    return state.xrSupported;
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
