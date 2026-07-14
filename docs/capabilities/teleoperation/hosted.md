---
title: "Hosted Teleop"
---

Operate a DimOS robot remotely from any browser or Quest headset over WebRTC.
The robot dials out to a hosted broker
([teleop.dimensionalos.com](https://teleop.dimensionalos.com)) — Cloudflare
Realtime or LiveKit SFU — so you don't need to open any inbound ports on the
robot's network. It works behind a home router, on Wi-Fi, wired LAN, or
cellular.

![The hosted teleop operator console: live video, minimap, command bar, and the metrics HUD](assets/operator_ui.png)

## How it works

There are three pieces: the **robot**, the **broker**, and the **operator's
browser**. You never connect to the robot directly.

1. **The robot dials out.** When you run a `teleop-hosted-go2-*` blueprint, the
   robot opens an outbound WebRTC session to the broker and registers itself.
   Because the robot initiates the connection, no inbound ports or port
   forwarding are needed — it works from behind any NAT.
2. **The broker bridges the session.** It sits between the robot and the
   operator, relaying video, the minimap, telemetry, and your commands. It also
   handles login and decides which operators may connect to which robots.
3. **You connect from the browser.** Open the console, pick your robot, and
   click **Connect**. The browser pulls the robot's video track and opens the
   command/telemetry data channels back to it.

Once connected, four streams flow continuously:

| Stream | Direction | Carries |
|--------|-----------|---------|
| Video | robot → you | The selected camera, composited into one live track |
| Minimap | robot → you | Occupancy grid + robot pose for click-to-navigate |
| Telemetry | robot → you | Battery, posture, link latency/rate for the HUD |
| Commands | you → robot | Drive input, sport commands, nav goals, E-STOP |

Everything runs in one robot-side process sharing a single broker session, so
there's exactly one video track and one control plane per robot — see
[How it connects](#how-it-connects) for the channel-level detail.

## Quick Start

```bash
TRANSPORTS__BROKER__API_KEY=dtk_live_... \
dimos run teleop-hosted-go2-transport
```

The robot registers with the broker. Open
[teleop.dimensionalos.com](https://teleop.dimensionalos.com), log in, and your
robot appears under **Available Robots**. Click **Connect** and you're driving.

The API key alone is enough — the broker derives the robot identity from it.
`TRANSPORTS__BROKER__ROBOT_ID` / `TRANSPORTS__BROKER__ROBOT_NAME` are optional
overrides. All broker settings can also be passed on the CLI, e.g.
`-o transports.broker.api_key=dtk_live_...`.

## Get an API key

1. Visit [teleop.dimensionalos.com](https://teleop.dimensionalos.com) and sign up.
2. On the dashboard, **API Keys → + New Key**.
3. Copy the key (shown once) and pass it as `TRANSPORTS__BROKER__API_KEY`.

## Available blueprints

| Blueprint | Backend | Notes |
|-----------|---------|-------|
| `teleop-hosted-go2-transport` | Cloudflare | Drive + camera + minimap + click-to-nav (recommended) |
| `teleop-hosted-go2-multicam` | Cloudflare | Adds a second RealSense, operator-selectable, mux'd into one video track |
| `teleop-hosted-go2-livekit` | LiveKit | Drive + camera + state; no minimap/click-nav yet |
| `teleop-hosted-go2-livekit-multicam` | LiveKit | LiveKit backend with the second RealSense |

The transport blueprints bind `Cloudflare*` / `LiveKit*` transports directly to
the streams of several small, per-concern modules that all run in one process
(`n_workers=1`) so they share a single broker session: `Go2CommandModule`
(command / E-STOP / drive guard), `CameraMuxModule` (camera → video track),
`MapCompressModule` (costmap → minimap), `HostedStatsModule` (telemetry + acks),
and the `GO2Connection` driver itself. LiveKit uses the same
`transports.broker.*` config key as Cloudflare.

Enable the glass-to-glass latency benchmark with
`-o cameramuxmodule.latency_stamp=true` (adds a timestamp strip the operator
reads then crops).

## Operating the robot

Once the robot is running and you've clicked **Connect**, here's how to fly it.

### 1. Connect

Open [teleop.dimensionalos.com](https://teleop.dimensionalos.com), find your
robot under **Available Robots**, and click **Connect**. Video appears within a
second or two; the metrics HUD starts populating once telemetry arrives. If the
robot doesn't show up, confirm the blueprint is still running and that the API
key matches the one the robot registered with.

### 2. Drive

Use **WASD** on a desktop keyboard (or the on-screen buttons on a phone) to
drive: `W`/`S` forward/back, `A`/`D` strafe, and turn with the yaw controls.
Hold **Shift** for 2× speed, **Ctrl** for half speed. Drive input streams
continuously and stops the instant you release — the robot treats a released
key as "stop," so it never keeps coasting on a dropped packet.

### 3. Navigate with the minimap

The minimap shows the robot's costmap and live pose. **Click any point** on it
to send a navigation goal — the robot plans a path and drives there on its own,
avoiding obstacles. Give a manual drive command at any time and it takes over
immediately, cancelling the plan. There's also a **cancel** control to stop
navigating without driving manually.

### 4. Postures and commands

The command bar exposes the robot's allow-listed actions:

- **Stand / sit / recover** — `RecoveryStand`, `StandDown`, `Sit`, and `Damp`
  (relax the joints).
- **Greetings / stretch** — `Hello`, `Stretch`.
- **Acrobatics** — `FrontJump`, `FrontPounce` — only available when the robot
  was launched with `-o go2commandmodule.allow_acrobatics=true`.
- **Obstacle avoidance** — toggle the onboard avoidance layer on or off.
- **Head LED** — set the head light brightness.
- **Camera** — on multicam robots, pick which camera (or side-by-side view) the
  video track shows.

Each command is acknowledged, so the UI reflects what the robot actually did,
not just what you clicked.

### 5. E-STOP

The **E-STOP** control is always available. It immediately stops all motion,
cancels any active navigation, and damps the robot — and it takes priority over
everything else in flight. Clear it with **estop_clear** (or the equivalent
control) when you're ready to resume; the robot won't move again until you do.

> **Operator loss is a stop, too.** If your connection drops, the robot sees the
> operator leave and halts on its own — it never keeps executing your last
> command into a dead link.

## Operator inputs

The browser is modality-agnostic — it streams whatever the device gives it, and
the robot blueprint decides what to do with it.

| Device | Input | Maps to |
|--------|-------|---------|
| Desktop browser | **WASD** keyboard | `TwistStamped` → `cmd_vel` |
| Phone | **On-screen WASD** | same path as keyboard |
| Quest 3 (Go2) | **Left thumbstick** Y → fwd/back, X → strafe; **Right thumbstick** X → yaw | `Joy` → derived twist on the robot |
| Quest 3 (xArm7) | **Controller poses** + analog triggers | `PoseStamped` → coordinator IK task; triggers → gripper |

Shift = 2× speed, Ctrl = ½×. The operator can also send allow-listed sport
commands (StandDown, RecoveryStand, Sit, Damp, Hello, Stretch, and — gated
behind `allow_acrobatics` — FrontJump, FrontPounce),
toggle obstacle avoidance / rage mode / the head LED, pick the camera, E-STOP,
and click the minimap to navigate.

## Live metrics HUD

While connected, the operator sees a metrics overlay, color-coded on video and
command-plane health:

| Metric | Source |
|--------|--------|
| `fps`, `bitrate`, `loss`, `jitter buffer`, `decode time`, `freezes` | Operator's `getStats()` on the inbound video track |
| `e2e latency` (glass-to-glass) | Frame-embedded capture-time stamp, decoded operator-side (needs `latency_stamp=true`) |
| `RTT` | NTP-style min-RTT clock sync over the reliable datachannel |
| `cmd latency`, `jitter`, `rate` | Robot-measured over the inbound command wire, sent back on `state_reliable_back` |

Pair with the recorder to log a session and emit a stats report:

```bash
dimos run teleop-hosted-go2-transport teleop-recorder
```

This writes `recording_teleop_<ts>.db` + a `report_<ts>.json` on disconnect;
regenerate from an old .db with `python -m dimos.teleop.utils.report path.db`.

## Configuration

Each concern is its own module, so its commonly-tuned fields live under that
module's config key (module class name, lowercased). Pass with `-o`, e.g.
`-o hostedstatsmodule.telemetry_hz=5`.

`hostedstatsmodule` — `HostedStatsModule`:

| Field | Default | Notes |
|-------|---------|-------|
| `telemetry_hz` | `3.0` | Robot → operator HUD push rate |

`go2commandmodule` — `Go2CommandModule`:

| Field | Default | Notes |
|-------|---------|-------|
| `cmd_stale_after_sec` | `0.5` | Drop `cmd_vel` older than this |
| `max_linear_mps` / `max_angular_rps` | `1.5` / `2.0` | Robot-side clamp on operator drive |
| `max_nav_goal_m` | `100.0` | Reject click-to-nav goals farther than this |
| `allow_acrobatics` | `false` | Gate FrontJump / FrontPounce etc. |
| `damp_on_operator_lost` | `false` | Damp the robot when the operator link drops |

`cameramuxmodule` — `CameraMuxModule`:

| Field | Default | Notes |
|-------|---------|-------|
| `latency_stamp` | `false` | Paint the glass-to-glass timestamp strip |
| `video_max_width` / `video_max_fps` | `0` (source) | Publish-side caps for constrained uplinks |
| `cameras` | `["cam1","cam2"]` | Named inputs; first is the boot default view |

`mapcompressmodule` — `MapCompressModule`:

| Field | Default | Notes |
|-------|---------|-------|
| `map_hz` / `odom_hz` | `2.0` / `15.0` | Minimap grid + robot-pose push rates (`0` = off) |
| `map_min_resolution` | `0.1` | Coarsen finer occupancy grids to this m/cell before encoding for the minimap |

Broker settings live under `transports.broker.*`: `api_key` (required),
`broker_url`, `robot_id`, `robot_name` (`"robot"` default), `stun_url`,
`video_codec` (e.g. `h264`/`vp8`), `video_max_bitrate_bps` (`0` = aiortc's
default 3 Mbps ceiling; raise for crisper video on a good uplink), and — for the
operator→robot mic — `audio_in` (opt-in). With broker `audio_in=true` the offer
carries a recvonly audio transceiver and the broker pulls the operator's mic
onto the session; set `go2connection.audio_in=true` as well and the Go2 plays it
on the dog's speaker. Operator→robot audio is Cloudflare-only.

## How it connects

The per-process `BrokerProvider` (Cloudflare) / `LiveKitBrokerProvider` owns the
session; blueprint transports bind to it. Channels:

```text
robot                          broker (Cloudflare / LiveKit)      operator browser/Quest
─────                          ─────────────────────────────      ──────────────────────
  POST /api/v1/sessions  ───►  session + datachannels     ◄───    operator joins
  cmd_unreliable        ◄────  (operator → robot, lossy)  ◄────    WASD / Joy
  state_reliable        ◄────  (operator → robot, json)   ◄────    ping, video_stats, estop
  state_reliable_back   ────►  (robot → operator, json)    ────►   pong, robot_telemetry, cmd_ack
  map_unreliable        ────►  (robot → operator, lossy)   ────►   minimap grid + odom
  video track           ────►  broker publishes + pulls    ────►   <video> sink
  audio track           ◄────  broker pulls operator mic   ◄────    mic (opt-in, audio_in)
```

For the WebRTC / aiortc / Cloudflare implementation details (MAX_BUNDLE, SCTP
id-0 channel, candidate propagation, thread model), see
[`dimos/teleop/hosted/README.md`](/dimos/teleop/hosted/README.md).

## Known Limitations

- **Single operator** per robot session today.
- **Connectivity.** A direct (STUN-discovered) path is preferred and used
  whenever it works; TURN is only the relay fallback for networks that can't
  connect directly. TURN credentials are fetched best-effort from the broker —
  if that fetch fails the robot runs STUN-only, so symmetric-NAT / some cellular
  networks (which need the relay) may then fail to connect.
- **No robot-side auto-reconnect.** If the link drops, the operator clicks
  **Connect** again; the robot side stays up.
- **LiveKit** ships command/state/video only — no minimap or click-to-nav yet.
- **Operator→robot audio** is opt-in (broker `audio_in=true`) and Cloudflare-only.
