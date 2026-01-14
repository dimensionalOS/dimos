# Teleop Web [TO BE UPDATED]

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
- X button triggers calibration

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
    │── Transform (pose) ──────────→ │── LCM publish ────────────→ │
    │── Float32 (trigger) ─────────→ │── LCM publish ────────────→ │
    │── Bool (X button) ───────────→ │── LCM publish ────────────→ │
    │                                │                             │
    │←─────────────────── LCM packets (subscribePacket) ──────────│
```
