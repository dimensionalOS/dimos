# Teleop Web

WebXR client and server for Quest 3 VR teleoperation.

## Components

### teleop_server.ts

Deno server that bridges WebSocket and LCM:
- Serves WebXR client over HTTPS (required for Quest)
- Forwards LCM packets to browser
- Forwards browser packets to LCM

### static/index.html

WebXR client running on Quest 3:
- Captures controller poses at 72Hz
- Sends Transform messages via WebSocket
- X button triggers calibration (teleop_enable)

## Running

```bash
cd dimos/teleop/web
deno run --allow-net --allow-read --unstable-net teleop_server.ts
```

Server starts at `https://localhost:8443`

## SSL Certificates

Generate self-signed certs (required for WebXR):

```bash
mkdir -p certs
openssl req -x509 -newkey rsa:2048 -keyout certs/key.pem -out certs/cert.pem -days 365 -nodes -subj "/CN=localhost"
```

## Message Flow

```
Quest Browser                    Deno Server                    Python
    │                                │                             │
    │── Transform (left pose) ─────→ │── /vr_left_transform ─────→ │
    │── Transform (right pose) ────→ │── /vr_right_transform ────→ │
    │── Float32 (left trigger) ────→ │── /vr_trigger_0 ──────────→ │
    │── Float32 (right trigger) ───→ │── /vr_trigger_1 ──────────→ │
    │── Bool (X button) ───────────→ │── /vr_teleop_enable ──────→ │
    │                                │                             │
    │←─────────────────── LCM packets (subscribePacket) ──────────│
```

## LCM Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/vr_left_transform` | Transform | Left controller pose (WebXR frame) |
| `/vr_right_transform` | Transform | Right controller pose (WebXR frame) |
| `/vr_trigger_0` | Float32 | Left trigger value (0.0-1.0) |
| `/vr_trigger_1` | Float32 | Right trigger value (0.0-1.0) |
| `/vr_teleop_enable` | Bool | Calibration toggle (X button) |
