# Go2 Minimal Demo

This is a stripped-down demo app for the Unitree Go2 Air.

What it does:

- connects to the Go2 over DimensionalOS WebRTC
- serves the live FPV stream on a local dashboard
- runs a simple sequential waypoint patrol
- accepts `/patrol`, `/stop`, and `/status` via Telegram
- runs YOLO-only tool detection, detects persons, chairs, boxes etc
- sends Telegram alerts with a JPG snapshot
- plays a WAV locally and, if available, through the Go2 speaker


## Layout

- `config.yaml`: runtime config
- `.env.example`: required environment variables
- `run.sh`: start script
- `demo_app/`: source-only application package

## Setup

From the workspace root:

```bash
cd dimos/demo
pip install -r requirements.txt
cp .env.example .env
```

Set values in `.env`:

```bash
TELEGRAM_BOT_TOKEN=...
TELEGRAM_OWNER_CHAT_ID=...
ROBOT_IP=192.168.12.1
```

If you prefer shell exports, that also works.

## Run

```bash
cd dimos/demo
./run.sh
```

Then open:

- dashboard: [http://localhost:8080](http://localhost:8080)

Telegram commands:

- `/start`
- `/patrol`
- `/stop`
- `/status`

## Notes

- The app is intended to run on your laptop or another host on the same Wi-Fi as the robot.
- Detection is YOLO-only. Any matching detection above the configured threshold can trigger an alert.
