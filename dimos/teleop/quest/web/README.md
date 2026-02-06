# Quest Teleop Web

WebXR client and server for Quest 3 VR teleoperation.

## Components

### teleop_server.ts

Deno server that bridges WebSocket and LCM:
- Serves WebXR client over HTTPS (required for Quest)
- Forwards controller data from browser to LCM
- Forwards LCM packets to browser

### static/index.html

WebXR client running on Quest 3:
- Captures controller poses at set frequency (depends on refresh rate)
- Sends PoseStamped and Joy messages via WebSocket
- X button (hold) engages teleoperation

## Running

```bash
deno run --allow-net --allow-read --allow-run --allow-write --unstable-net dimos/teleop/quest/web/teleop_server.ts
```

Server starts at `https://localhost:8443`

SSL certificates are generated automatically on first run in `assets/teleop_certs/`.

## Message Flow

```
Quest Browser                    Deno Server                    Python
    │                                │                             │
    │── PoseStamped (left) ────────→ │── vr_left_pose ───────────→ │
    │── PoseStamped (right) ───────→ │── vr_right_pose ──────────→ │
    │── Joy (left controller) ─────→ │── vr_left_joy ────────────→ │
    │── Joy (right controller) ────→ │── vr_right_joy ───────────→ │
    │                                │                             │
    │←─────────────────── LCM packets (subscribePacket) ──────────│
```

## LCM Topics

| Topic | Type | Description |
|-------|------|-------------|
| `vr_left_pose` | PoseStamped | Left controller pose (WebXR frame) |
| `vr_right_pose` | PoseStamped | Right controller pose (WebXR frame) |
| `vr_left_joy` | Joy | Left controller buttons/axes |
| `vr_right_joy` | Joy | Right controller buttons/axes |

## Joy Message Format

Quest controller data is packed into Joy messages:

**Buttons** (indices 0-6):
- 0: trigger (analog 0.0-1.0)
- 1: grip (analog 0.0-1.0)
- 2: touchpad
- 3: thumbstick press
- 4: X/A (primary)
- 5: Y/B (secondary)
- 6: menu

**Axes** (indices 0-1):
- 0: thumbstick X
- 1: thumbstick Y
