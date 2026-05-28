# Go2 Visual Event Notifier

This example turns robot perception into phone alerts. It watches a Go2 RGB camera
stream, detects stable red, yellow, or green visual states, and publishes alerts
through the local DimOS browser notification page and optionally through ntfy.

The demo uses a traffic-light-style signal, but the reusable pattern is:

```text
robot perception -> stable visual event -> external app action
```

This is a prototype visual-event trigger, not a production traffic-safety system.

## Phone Setup

Use the local browser alert page as the primary path. It does not require an API
key or internet connection.

```bash
export LAPTOP_IP=$(ipconfig getifaddr en1)
[ -z "$LAPTOP_IP" ] && export LAPTOP_IP=$(ipconfig getifaddr en0)
echo "$LAPTOP_IP"
```

Open this on the phone after the demo script starts:

```text
https://<LAPTOP_IP>:8450/notify
```

Accept the certificate warning, tap **Enable alerts**, turn volume up, and keep
the page in the foreground.

For an Android-native notification fallback, install the ntfy app and subscribe
to a hard-to-guess topic:

```bash
export NTFY_TOPIC=go2-demo-$(uuidgen | tr '[:upper:]' '[:lower:]' | cut -c1-8)
echo "$NTFY_TOPIC"
curl \
  -H "Title: Go2 Test" \
  -H "Priority: urgent" \
  -H "Tags: rotating_light" \
  -d "Go2 notification test" \
  "https://ntfy.sh/$NTFY_TOPIC"
```

## Real Go2 Run

Connect the laptop to the Go2 network, discover the robot, and verify the IP:

```bash
uv run dimos go2tool discover
export ROBOT_IP=<GO2_IP>
ping -c 3 "$ROBOT_IP"
```

Start the demo:

```bash
uv run python examples/go2_visual_event_notifier/go2_traffic_light_companion.py \
  --source go2 \
  --robot-ip "$ROBOT_IP" \
  --browser-alerts \
  --listen-host 0.0.0.0 \
  --browser-port 8450 \
  --ntfy-topic "$NTFY_TOPIC" \
  --stable-frames 3 \
  --cooldown-s 2 \
  --fps 5 \
  --log-file go2_visual_event_log.jsonl
```

Open the signal page and put it in front of the Go2 camera:

```bash
open examples/go2_visual_event_notifier/demo_signals.html
```

Expected behavior:

```text
GREEN  -> normal "Go" phone alert
YELLOW -> high-priority "Caution" phone alert
RED    -> urgent "Stop" phone alert
```

## Webcam Rehearsal

Use a local webcam to test the detector and alert delivery before connecting to
real hardware:

```bash
uv run python examples/go2_visual_event_notifier/go2_traffic_light_companion.py \
  --source webcam \
  --camera-index 0 \
  --browser-alerts \
  --listen-host 0.0.0.0 \
  --browser-port 8450 \
  --ntfy-topic "$NTFY_TOPIC" \
  --display \
  --run-seconds 60
```

If the detector is too sensitive, crop the image and lower the area threshold:

```bash
uv run python examples/go2_visual_event_notifier/go2_traffic_light_companion.py \
  --source go2 \
  --robot-ip "$ROBOT_IP" \
  --browser-alerts \
  --listen-host 0.0.0.0 \
  --roi 0.25,0.15,0.75,0.85 \
  --min-area-ratio 0.005 \
  --stable-frames 2
```

## Recording Script

Record a 90-second video with the Go2, terminal, and phone visible.

```text
0-10s:
"We built a DimOS visual-event notifier. Go2 watches the world and turns stable
visual events into phone actions."

10-25s:
Show the detector, demo script, browser notification skill, and tests.

25-45s:
Show GREEN. Terminal prints GREEN event. Phone receives "Go".

45-65s:
Show YELLOW. Terminal prints YELLOW event. Phone receives "Caution".

65-82s:
Show RED. Terminal prints RED event. Phone receives urgent "Stop".

82-90s:
"The demo is a traffic-light companion prototype, but the same DimOS pattern
generalizes to factory status lights, package alerts, warning signs, and patrol
events."
```

## Notes

- No API key, LLM agent, robot speech, navigation, or watch SDK is required.
- Keep the robot stationary; this demo only uses camera frames.
- Use a large, bright signal and avoid glare.
- Use ntfy only when the laptop has internet while connected to the robot setup.

