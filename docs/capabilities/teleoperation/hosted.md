---
title: "Hosted Teleop"
---

Operate a DimOS robot remotely from any browser or Quest headset over WebRTC.
The robot dials out to a hosted Cloudflare Realtime SFU broker
([teleop.dimensionalos.com](https://teleop.dimensionalos.com)), so you don't
need to open any inbound ports on the robot's network — it works behind a home
router, on Wi-Fi, or wired LAN.

## Quick Start

```bash
TELEOP_API_KEY=dtk_live_... \
TELEOP_ROBOT_ID=my-robot \
TELEOP_ROBOT_NAME="Lab Go2" \
dimos run teleop-hosted-go2
```

The robot registers with the broker. Open
[teleop.dimensionalos.com](https://teleop.dimensionalos.com), log in, and your
robot appears under **Available Robots**. Click **Connect** and you're driving.

| Module | Role |
|--------|------|
| `HostedTeleopModule` | Dials the broker, owns the WebRTC connection, datachannels, video, clock-sync, and live telemetry |
| `HostedTwistTeleopModule` | Mobile-base subclass: scales operator commands into `cmd_vel` (m/s, rad/s) |
| `HostedArmTeleopModule` | Arm-IK subclass: per-hand pose routing to coordinator tasks |
| `Go2Module` *(in the blueprint)* | The robot driver receiving `cmd_vel` |

## Get an API key

1. Visit [teleop.dimensionalos.com](https://teleop.dimensionalos.com) and sign up.
2. On the dashboard, **API Keys → + New Key**.
3. Copy the key (shown once) and pass it as `TELEOP_API_KEY` when launching the blueprint.

The key is per-robot; one key authenticates one robot. The same user account can
manage many keys for different robots.

## Available blueprints

| Blueprint | Use case | Subclass |
|-----------|----------|----------|
| `teleop-hosted-go2` | Mobile base (Unitree Go2, wheeled robots) | `HostedTwistTeleopModule` |
| `teleop-hosted-xarm7` | Arm IK (UFactory xArm7) | `HostedArmTeleopModule` |

Pair with the recorder to log a session and emit a transport-stats report:

```bash
dimos run teleop-hosted-go2 teleop-recorder
```

This writes `recording_teleop_<ts>.db` plus a `report.md` next to it on
disconnect. Reports can also be regenerated from an existing .db:

```bash
python -m dimos.teleop.utils.report path/to/recording.db
```

## Operator inputs

The browser is modality-agnostic — it just streams whatever the device gives it,
and the robot blueprint decides what to do with it.

| Device | Input | Maps to |
|--------|-------|---------|
| Desktop browser | **WASD** keyboard | `TwistStamped` → `cmd_vel` |
| Phone | **On-screen WASD** keys | same path as keyboard |
| Quest 3 (Twist robot) | **Left thumbstick** Y → forward/back, X → strafe; **Right thumbstick** X → yaw | `Joy` → derived twist on the robot |
| Quest 3 (Arm robot) | **Controller poses** + analog triggers | `PoseStamped` → coordinator `TeleopIKTask` |

Shift = 2× speed, Ctrl = ½× speed and strafe on keyboard.

## Live metrics HUD

While connected, the operator sees a metrics overlay (corner pill in the
browser, in-headset stats panel in VR). Color-coded green/amber/red based on
video and command-plane health:

| Metric | Source |
|--------|--------|
| `fps`, `bitrate`, `loss`, `jitter buffer`, `decode time`, `freezes` | Operator's `getStats()` on the inbound video track |
| `RTT` | NTP-style min-RTT clock sync over the reliable datachannel |
| `cmd latency`, `jitter`, `rate` | Robot-measured from the inbound twist stream — what *actually arrived*, sent back over `state_reliable_back` |

The recorder captures these to the session `.db` and summarizes them in
`report.md`.

## Configuration

`HostedTeleopConfig` (base, applies to both subclasses):

| Field | Default | Notes |
|-------|---------|-------|
| `broker_url` | `https://teleop.dimensionalos.com` | Override via `-o` / config to point at a self-hosted broker |
| `broker_api_key` | `""` | Required. Env: `TELEOP_API_KEY` |
| `robot_id` | `""` | Required, identifies this robot. Env: `TELEOP_ROBOT_ID` |
| `robot_name` | `""` | Display name shown in the dashboard. Env: `TELEOP_ROBOT_NAME` |
| `control_loop_hz` | `50.0` | Per-hand publish + button-state cycle |
| `heartbeat_hz` | `1.0` | HTTP heartbeat to the broker (also drives channel-id sync) |
| `telemetry_hz` | `3.0` | Robot → operator HUD command-plane stats |
| `stun_urls` | `[stun:stun.cloudflare.com:3478]` | STUN servers for ICE |
| `turn_urls`, `turn_username`, `turn_credential` | `""` | TURN credentials. Fields exist; not yet auto-provisioned. |

`HostedTwistTeleopConfig` adds:

| Field | Default | Notes |
|-------|---------|-------|
| `linear_speed` | `0.5` | Multiplied into `cmd_vel.linear` (m/s) |
| `angular_speed` | `0.8` | Multiplied into `cmd_vel.angular` (rad/s) |

`HostedArmTeleopConfig` adds:

| Field | Default | Notes |
|-------|---------|-------|
| `task_names` | `{}` | Maps `"left"`/`"right"` → coordinator task name (e.g. `"teleop_xarm"`), used as `frame_id` so the coordinator routes to the right IK task |

## How it connects

```text
robot                          broker (Cloudflare SFU)            operator browser/Quest
─────                          ──────────────────────             ──────────────────────
HostedTeleopModule
  POST /api/v1/sessions  ───►  CF session + datachannels  ◄───    POST /sessions/{id}/join
                                                                     (operator joins)
  cmd_unreliable        ◄────  (operator → robot, lossy)  ◄────    WASD / Joy / poses
  state_reliable        ◄────  (operator → robot, json)   ◄────    ping, video_stats
  state_reliable_back   ────►  (robot → operator, json)    ────►   pong, robot_telemetry
  video track           ────►  CF publishes + pulls        ────►   <video> sink
```

Datachannels are **negotiated** (SCTP ids assigned by the broker). The video
track is added before the SDP offer; the broker pulls it onto each operator
session and renegotiates so the operator's `ontrack` fires.

For the WebRTC / aiortc / Cloudflare implementation details (MAX_BUNDLE
constraints, candidate propagation, the throwaway SCTP id 0 channel, thread
model), see [`dimos/teleop/quest_hosted/README.md`](/dimos/teleop/quest_hosted/README.md).

## Known Limitations

- **Single operator** per robot session today. Multi-viewer / single-driver+watchers is roadmapped.
- **TURN is not wired yet.** ICE relies on STUN only, so direct connectivity must succeed — works on most home/office networks, can fail on symmetric NAT or cellular. TURN field plumbing exists.
- **No auto-reconnect.** If the link drops mid-session, the operator must click **Connect** again. The robot side stays up; reconnection is supported, just manual.
- **Single camera** per robot today. Multi-camera support is roadmapped.
- **Operator is in a fixed slot until clean disconnect** — a tab-close leaves the slot held until the broker's grace timeout fires (or the robot restarts).
