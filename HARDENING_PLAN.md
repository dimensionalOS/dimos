# Hosted Teleop — Issues & Execution Plan

Full-stack audit of hosted teleop (this repo + `dimos` robot side), 2026-07-01.
Companion to `BUG_AUDIT.md` (control-plane fix cycle, mostly ✅) — this file
covers what's still open across **safety, robustness, security, infra, and
video quality**, and how to execute the fixes.

**Legend:** severity `P0` (safety/security, do now) · `P1` (next) · `P2` (later/scale)
· effort `S` (<½ day) · `M` (1–2 days) · `L` (multi-day).
Repo tags: `[teleop]` = this repo, `[dimos]` = robot side.

Verified context that frames severity:

- The base-velocity path **already has a deadman**: every `move()` re-arms a
  200 ms auto-stop timer (`dimos/robot/unitree/connection.py:202`,
  `cmd_vel_timeout = 0.2`). Link drop while driving → base halts in ≤200 ms.
- The browser stamps twists in the robot's clock frame (clock-sync offset), and
  the robot drops stale (>0.5 s) / out-of-order twists
  (`hosted_connection.py:327`).

So the open safety work is about what the watchdog *doesn't* cover: frozen
video, in-flight sport actions, and truthful state after reconnect.

---

## Workstream A — Safety supervisor (P0)

Matches `dimos` roadmap Phases 2–3 (`myprojects/hosted/README.md`).

### A1. Wire the video-freshness drive lockout — `[teleop]` `S` — ✅ implemented
(auto-resume + neutral gate; liveness = video.currentTime progression, 1s
threshold; overlay + DRIVE pill + health=bad; one zero-twist on transition;
unit-tested in web/js/tests/stall.test.mjs. VR pose streaming not gated —
tracked separately.)
- **Issue:** The cockpit's `#video-lost` "video stalled — drive disabled"
  overlay (`web/js/views/go2.js:109`) is dead markup — nothing toggles it, and
  `state.driveEnabled` gates only on posture/E-STOP (`go2.js:420`). An operator
  can keep driving on a frozen last frame.
- **Plan:** Track video liveness in the 1 Hz tick: freshness =
  `framesDecoded` delta from `state.liveStats.video` (already sampled) or
  `video.currentTime` progression as fallback (covers LiveKit, which has no
  stats yet). If stale > N ms (start with 1000): show `#video-lost`, force
  `state.driveEnabled = false`, and have the keyboard loop emit one zero-twist
  before going quiet. Auto-clear when frames resume; require posture re-arm
  (press Stand/Drive) to resume drive.
- **Accept:** kill the robot-side video (leave datachannels up) → overlay
  appears ≤1.5 s, WASD stops sending, HUD health goes `bad`; frames resume →
  overlay clears, drive resumes only after Stand/Drive.

### A2. Robot-side E-STOP latch + stop-on-disconnect — `[dimos]` `M`
- **Issue:** No `estop` message type exists robot-side ("Damp" is just an
  allow-listed sport cmd). Browser E-STOP is fire-and-forget over the link —
  if the link is down it never arrives. Channel close is silent
  (`providers/broker.py:353`); an **in-flight sport action** (Jump/Pounce
  thread) runs to completion regardless of link state.
- **Plan:**
  1. Add `{type:"estop", nonce}` handled in `_on_state_json` *before* any
     dispatch: latch `self._estopped = True`, call `Damp`, ack. While latched,
     `move()` returns False and sport cmds are refused (ack `ok:false`,
     `reason:"estopped"`). Add `{type:"estop_clear"}` to re-arm.
  2. Stop-on-disconnect: in `BrokerProvider._heartbeat_once`, when
     `state_reliable`'s SCTP id transitions set→None (operator left), invoke a
     provider-level `on_operator_lost` callback; `Go2HostedConnection` wires it
     to `stop_movement()` + (config-gated) `Damp`. Mirror on LiveKit via
     `ParticipantDisconnected`.
  3. Surface `estopped` in telemetry (see A3) so the browser latch reflects
     robot truth, not local assumption.
- **Accept:** pull operator network mid-Jump → robot damps on operator-loss;
  E-STOP pressed with healthy link → ack ≤300 ms and subsequent WASD/actions
  refused until re-arm.

### A3. State snapshot on operator join + richer telemetry — `[dimos]` + `[teleop]` `M`
- **Issue:** `robot_telemetry` carries only `{cmd stats, soc, robot_ts}`
  (`hosted_connection.py:361`). Reconnecting operators get an optimistic UI:
  posture defaults to `StandReady`, obstacle-avoidance assumed ON, rage/cams
  unknown. (Cockpit `ui.posture`/`ui.obstacleAvoid` are local guesses.)
- **Plan:** Extend the 3 Hz telemetry payload with
  `state: {posture, rage, obstacle_avoidance, cams, estopped}` (robot-side
  authoritative values — `_rage_active`, `_cam_selected`, new estop latch,
  last-confirmed posture from the sport-cmd path). On the browser, seed
  `ui.*` from the first `robot_telemetry` after connect instead of constants,
  and reconcile on every message. This removes the need for a separate
  "greeting" message and survives missed packets (it's periodic).
- **Accept:** toggle OA / select Rage / sit the robot, hard-refresh the
  browser, reconnect → cockpit shows the true OA/rage/posture within 1 s.

---

## Workstream B — Robot-side hardening `[dimos]` (P1)

### B1. Bound command execution — `S`
- **Issue:** Every `sport_cmd`/`set_mode`/`obstacle_avoidance`/`StandReady`
  spawns an unbounded daemon thread (`hosted_connection.py:246,268,295,312`).
  Exposure is low (single authenticated operator, UI disables pending
  buttons), but a stuck RPC + spam stacks threads.
- **Plan:** One `threading.BoundedSemaphore(2)` around all runners; on
  contention ack `ok:false, reason:"busy"` immediately. Optional: serialize
  sport cmds through a single worker + queue(maxsize=1).

### B2. Nonce idempotency — `S`
- **Issue:** `nonce` is echoed but never deduped (`hosted_connection.py:223`).
  Not a replay *vulnerability* (channel is DTLS-authenticated,
  ordered-reliable), but double-click/browser-retry double-executes.
- **Plan:** `deque(maxlen=64)` of seen nonces; duplicate → re-ack last result,
  don't re-execute.

### B3. `_rage_active` race — `S`
- **Issue:** Read in `_handle_set_mode` (line 280), written in the spawned
  thread (line 289), no lock — rapid toggles can double-fire `set_rage_mode`.
- **Plan:** Guard with a small `threading.Lock` (or fold into B1's
  serialization, which fixes it for free).

### B4. Stale watchdog comment — `S` (trivial, do with B1)
- `connection.py:201` says "Auto-stop after 0.5 seconds"; the constant is
  `cmd_vel_timeout = 0.2`. Fix the comment — it's the safety-critical number.

### B5. LiveKit heartbeat terminal condition — `S`
- **Issue:** CF provider stops after 5 consecutive 401/404; LiveKit's loop
  (`livekit_broker.py:288`) retries/logs forever on revoked key/deleted
  session.
- **Plan:** Port the 5-strike terminal logic.

---

## Workstream C — Broker & deployment security `[teleop]` (P0/P1)

### C1. Network exposure — `S` **P0** — ✅ fixed (needs `terraform apply`)
- **Issue:** Security group opens **SSH 22** and **app 8450** to
  `0.0.0.0/0` (`terraform/main.tf`); `config.py:42` defaults
  `host="0.0.0.0"` (prod is safe only because systemd passes
  `--host 127.0.0.1`).
- **Plan:** Drop the 8450 ingress rule entirely (Caddy is the only public
  entry); restrict 22 to an admin CIDR var (or SSM Session Manager and drop
  22). Change `config.py` default to `127.0.0.1`.

### C2. Rate limiting — `M` **P1** (BUG_AUDIT N5)
- **Issue:** No limits on `/keys` (mint), `/join`, `/turn-credentials`
  (TURN quota burn), auth-failure probing.
- **Plan:** `slowapi` (or a 20-line token bucket keyed on
  `owner_id|sub|IP`) at the router level: writes 10/min, TURN 6/min,
  heartbeats exempt. Return 429 with `Retry-After`.

### C3. Signup gating — `S` **P1**
- **Issue:** Cognito self-signup is open with auto-verified email
  (`terraform/cognito.tf`) — anyone can register and mint robot API keys.
  Tenant isolation contains them, but it's an open surface.
- **Plan:** Least effort: a Cognito **pre-signup Lambda** allowlisting
  `@dimensionalos.com` (+ invited domains); or flip to admin-create-only
  (`allow_admin_create_user_only`) while the pool is small.

### C4. CSP + token storage — `M` **P2** (BUG_AUDIT N1/N2, acknowledged) — ✅ CSP report-only + SRI shipped; enforce after validation; cookie auth still tracked
- **Plan (incremental, don't boil the ocean):** add
  `Content-Security-Policy-Report-Only` in the Caddyfile covering the known
  origins (self, `cdn.tailwindcss.com`, `cdn.jsdelivr.net`, `esm.sh`,
  `fonts.g*`, Cognito endpoint, `wss:` for LiveKit), watch console for a week,
  then enforce. Pin the LiveKit script to an exact version + SRI hash and
  replace the Tailwind *runtime CDN* (explicitly not-for-production) with a
  built CSS file — that also removes the biggest inline-script obstacle to a
  strict CSP. httpOnly-cookie auth stays a separate tracked item (architectural).

### C5. DEPLOY.md Caddy drift — `S` **P1** — ✅ fixed (deploy.sh now ships web/ + Caddyfile too)
- **Issue:** DEPLOY.md Step 4 tells you to overwrite the Caddyfile with a bare
  `reverse_proxy 127.0.0.1:8450` — that proxies *everything* to uvicorn and
  breaks static SPA serving, contradicting the committed `Caddyfile`.
- **Plan:** Replace Step 4 with "copy the repo `Caddyfile`" (single source of
  truth), and have `scripts/deploy.sh` rsync it + reload caddy.

---

## Workstream D — Scale & operability `[teleop]` (P2, plan now)

### D1. Single-instance SPOF — `L`
- **Issue:** One t3.small + SQLite; Litestream is one-way backup, not
  failover. Broker holds in-memory state (`_robot_channel_ids`,
  `_session_locks`, `_pending_video_renegotiations`,
  `routers/sessions.py:38-50`) so it cannot scale past one process — running
  uvicorn with `--workers >1` silently breaks channel-id delivery today.
- **Plan (ordered):**
  1. Guard: assert single-worker at startup (cheap insurance now).
  2. Move `_robot_channel_ids` + pending-renegotiation flags into
     `TeleopSession` columns (`state_back_channel_id` already lives there —
     finish the job; the maps are tiny and change at human cadence).
  3. Postgres via existing `DATABASE_URL`; swap per-session asyncio locks for
     row-level `SELECT … FOR UPDATE`.
  4. Then 2× instances behind an ALB + health checks; or, cheaper stopgap: an
     ASG of 1 with EIP re-attach for auto-recovery.
- **Accept:** kill -9 the broker mid-session → robot heartbeat re-learns
  channel ids from the replacement instance; drive resumes without robot
  restart.

### D2. Observability — `M`
- **Issue:** journald-only logs, no metrics, no alerting; `report.json` exists
  robot-side but nothing aggregates fleet health.
- **Plan:** `/metrics` (prometheus-fastapi-instrumentator) + CloudWatch agent;
  alarms on: health-check fail, robot-heartbeat 5xx rate, reaper eviction
  spikes, disk >80 %. Log the per-session quality columns (`rtt_ms`, loss)
  on eviction for postmortems.

### D3. Terraform drift — `S`
- `ignore_changes = [ami, user_data]` means rotated CF/Cognito secrets in
  tfvars never reach the box. Move runtime secrets out of user_data into SSM
  Parameter Store (instance role already exists); `.env` rendered by a small
  fetch script at boot → rotation = restart, no re-pave.

---

## Workstream E — Video quality & frontend fixes (P1)

### E1. Codec/bitrate control on the robot track — `M/L` `[dimos]`
- **Issue:** aiortc defaults (VP8, source-rate, native resolution): no target
  bitrate, no degradation preference — congestion shows up as drops/freezes
  instead of graceful downscale. This is the single biggest lever on perceived
  latency.
- **Plan:** Prefer H.264 in the SDP munge; set encoder target bitrate
  (aiortc `RTCRtpSender.setParameters` / encoder config) from a blueprint knob;
  cap publish resolution/FPS in the mux (`_composite`). Stretch: close the
  loop — operator's `video_stats` (loss/freezes, already delivered to the
  robot) nudges the encoder bitrate up/down.

### E2. LiveKit HUD stats parity — `S/M` `[teleop]`
- **Issue:** `startVideoStats` is skipped on LiveKit (`livekit.js:106`) — HUD
  video panel is blank, health classifier runs degraded.
- **Plan:** Sample the subscribed track's `RTCStatsReport` via the LiveKit SDK
  (`track.receiver.getStats()` or SDK stats events) and feed the same
  `video_stats` payload shape.

### E3. Verified frontend bugs (small, batch into one PR) — `S` `[teleop]` — ✅ fixed
1. **`videoStats` wedge:** `webrtc.js:385` — `if (dt <= 0) return;` leaks
   `inFlight = true` permanently (stats + e2e readout die for the session).
   Reset `inFlight` before that return.
2. **Stale battery/path across sessions:** `disconnect.js` resets
   `liveStats.{video,rttMs,offsetMs,cmdHz,cmd}` but not `soc` / `iceType` —
   next robot briefly shows the previous robot's battery and ICE path.
3. **`speedScale` mismatch:** `state.js:47` initializes `{lin:0.5, ang:0.5}`
   but `keyboard.js:120`'s comment claims the standalone keyboard view
   defaults to 1.0 (the `||` fallback never fires — object is truthy). The
   standalone view silently drives at half speed. Decide intended default;
   reset `state.speedScale` in `disconnect()`.
4. **Failed connect leaks loops:** `connect.js` catch paths navigate away
   without `stopKeyboardLoop()` / `stopTick()` — intervals survive (guards
   make them no-ops, but they stack across retries). Route failures through
   a `teardown()` that mirrors `disconnect()`'s local half.
5. **`setupLiveKit` re-entry:** no `setupInProgress` guard (unlike
   `setupWebRTC`) — double-click Connect can create two Rooms. Reuse the same
   flag.
6. **`op-heartbeat` overlap:** async `setInterval` without in-flight guard and
   `fetch` without timeout — a hung request stacks. Add the same in-flight
   bool used by `videoStats`.

---

## Execution order

| Phase | Items | Rationale |
|---|---|---|
| **Now (P0)** | A1, C1, B4 | Highest safety-per-effort (A1 is browser-only), close the network exposure, fix the misleading safety comment. |
| **Sprint 1 (P1)** | A2, A3, E3 | Safety supervisor robot-side + truthful reconnect UI + the verified frontend bug batch (E3 rides along with A1/A3 UI work). |
| **Sprint 2 (P1)** | B1–B3, B5, C2, C3, C5, E2 | Hardening + limits + LiveKit parity. |
| **Later (P2)** | E1, C4, D1–D3 | Video pipeline upgrade, CSP/token migration, scale-out — start D1 planning when >~5 concurrent robots or first paying tenant. |

Cross-repo note: A2/A3/B* land in `dimos` (robot), A1/C*/D*/E2/E3 in
`dimensional-teleop`. A3 spans both — ship the robot-side telemetry extension
first (browser ignores unknown fields today, so it's backward-compatible),
then the UI seeding.

## Explicitly out of scope / already handled

- Operator-takeover TOCTOU, bridge partial-failure leaks, reaper coverage,
  JWKS refresh, setup timeouts — fixed; see `BUG_AUDIT.md` ✅ entries.
- Base-velocity deadman — **exists** (200 ms watchdog); do not re-report.
- Nonce "replay attack" — downgraded to idempotency nit (B2); channel is
  authenticated and ordered.
- `latency_stamp` per-frame copy cost — benchmark-only flag, default off.
