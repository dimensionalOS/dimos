# mlxvlm → dimos: drive the Go2 from a VLM

Use this as the prompt for the Claude working on `/Users/tex/repos/ai/mlx/mlxvlm`.

## Context

dimos is the robot OS. It runs as a separate process on the same Mac and
exposes three OpenAI/HTTP-shaped bridges on localhost so mlxvlm can plug in
without speaking any internal dimos APIs:

| URL | Direction | Format |
|---|---|---|
| `GET http://127.0.0.1:7780/video_feed/color_image` | dimos → you | MJPEG (`multipart/x-mixed-replace; boundary=frame`) |
| `GET http://127.0.0.1:7780/snapshot/color_image` | dimos → you | one JPEG |
| `GET ws://127.0.0.1:7781/audio_out` | dimos → you | binary int16 PCM frames; first WS message is `{"event":"format","sample_rate":N,"channels":N}` |
| `GET http://127.0.0.1:7781/audio_info` | dimos → you | `{sample_rate,channels}` JSON |
| `POST http://127.0.0.1:7781/play` | you → dimos | body = raw WAV bytes; plays on robot speaker (no-op in sim) |
| `POST http://127.0.0.1:7782/cmd_vel` | you → dimos | one Twist step |
| `POST http://127.0.0.1:7782/path` | you → dimos | sequence of Twist steps, executed in order |
| `POST http://127.0.0.1:7782/stop` | you → dimos | zero-Twist emergency stop |
| `GET http://127.0.0.1:7782/pose` | dimos → you | `{x, y, z, theta, ts}` of base_link in world frame |

All endpoints are CORS-open. Endpoints stay the same whether dimos is in
MuJoCo sim or driving a real Go2 — only audio is robot-only.

## Twist + Path semantics

`cmd_vel` is a standard ROS-style Twist: linear `[x_forward, y_left, z]` in
m/s, angular `[x_roll, y_pitch, z_yaw]` in rad/s. Open-loop: the bridge
publishes the Twist, sleeps for `duration` seconds, then publishes zero.

`/path` takes a list of steps. Two ways to spell a step:

1. **Raw Twist** — the model emits velocity components:
   ```json
   {"linear":[0.3,0,0],"angular":[0,0,0],"duration":1.0}
   ```

2. **Semantic** — the model emits distances in m / rotation in deg, and the
   bridge divides by `duration` to make a Twist:
   ```json
   {"forward":0.5,"left":0,"degrees":0,"duration":2.0}
   {"forward":0,"left":0,"degrees":-90,"duration":1.5}
   ```

Steps run sequentially. `/stop` cancels an in-flight `/path` cooperatively.

No SLAM, no obstacle avoidance — for closed-loop, **re-plan** every iteration
from a fresh camera frame instead of sending long paths.

## What to implement in mlxvlm

Add a `POST /api/robot/navigate` route to `app.py` that runs a perceive-act
loop until the goal is reached, the user cancels, or `max_steps` is hit:

```
input:  { "goal": "find the red chair and stop in front of it",
          "max_steps": 12,
          "step_seconds": 1.0 }

loop:
  1. frame = httpx.get(DIMOS_CAMERA_URL).content
  2. pose  = httpx.get(DIMOS_POSE_URL).json()
  3. ask Gemma-4-Falcon-Perception via the existing /api/analyze pipeline:
       "Camera frame attached. Robot pose: {pose}. Goal: {goal}.
        Decide the next ≤2 second movement. Reply ONLY with strict JSON:
          { 'action':'move'|'stop'|'arrived',
            'forward': float meters,
            'left':    float meters,
            'degrees': float rotation,
            'reason':  '<short>' }
        Positive forward=ahead, positive left=left, positive degrees=CCW.
        Keep |forward|≤1.0, |left|≤0.5, |degrees|≤45 per step."
  4. parse JSON; if action=='arrived' or 'stop', POST /stop and break
  5. POST /api/robot/navigate/steps a single PathStep with the chosen
     values + duration=step_seconds, wait for response
  6. SSE-stream {step, action, reason, pose} so the UI shows progress

env config (read at startup):
  DIMOS_CAMERA_URL  = http://127.0.0.1:7780/snapshot/color_image
  DIMOS_POSE_URL    = http://127.0.0.1:7782/pose
  DIMOS_PATH_URL    = http://127.0.0.1:7782/path
  DIMOS_STOP_URL    = http://127.0.0.1:7782/stop
  DIMOS_PLAY_URL    = http://127.0.0.1:7781/play     (optional, for spoken feedback)
```

Failure modes the loop must handle:
- camera/pose endpoint 5xx or timeout (skip the step, log, retry once)
- JSON parse failure from the model (treat as `stop`)
- per-step Twist limits exceeded by the model (clamp, log)
- max_steps hit without `arrived` (return `{status: "exhausted", history: […]}`)

UI (optional, in `static/`):
- "Goal" text input + "Go" button → POST /api/robot/navigate
- live event stream → render the latest camera frame, current pose, last
  reason, step counter
- big red "Stop" button → POST `DIMOS_STOP_URL` directly (don't wait for
  the agent)

## Audio loop (optional follow-up)
If you want the robot to speak / listen:
- Mic: open `ws://127.0.0.1:7781/audio_out`, parse the first JSON
  `format` frame, buffer ~1s of PCM, wrap in a WAV header, POST to the
  existing mlx-audio whisper server on `:8000`.
- Speaker: TTS output → POST raw WAV to `http://127.0.0.1:7781/play`.

## Things NOT to do
- Don't try to import anything from dimos. All integration is over HTTP/WS.
- Don't long-poll `/path` — it blocks until the steps finish. Use shorter
  paths (1–3 steps) per loop iteration so the VLM can re-evaluate.
- Don't assume metric scale from the camera — the VLM reasons about
  "ahead", "left", "approaching", not centimeters. Conservative step sizes
  (0.3–0.6 m forward, ≤30° turns) are safer.

## Testing without a real robot
The dimos sim publishes the same endpoints. Start it with:
```
cd ~/repos/robotics/dimos
./sim-with-llm.sh mlxvlm    # routes the in-sim agent's LLM to your :8080
```
Then in another terminal, your mlxvlm `POST /api/robot/navigate` should
drive the sim Go2 in the MuJoCo viewer, and you'll see its 3D position
change via `GET /pose`.
