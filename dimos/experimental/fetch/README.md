# Fetch Prototype

This prototype turns the dog behavior into a testable state machine before the
robot is available. It reuses the same DimOS web pattern as phone teleop:
a FastAPI server exposes an HTTPS phone UI, the phone streams camera frames over
WebSocket, and the server returns motion/speech/photo decisions.

## Behavior

1. Run the robot preflight: recovery stand, balance stand, and joystick handoff.
2. Actively look around by turning in place until a good Coke/photo target is visible: someone centered in frame and looking toward the dog who seems chill, thirsty, amused, curious, playful, social, or likely to enjoy a free Coke from a robot dog.
3. Treat phone, book, laptop, food, or drink as weak busy signals only when the person looks engrossed or unavailable.
4. Approach only if the path looks safe.
5. Stop once the target is within 4 meters.
6. Wave and say a personalized joke based on visible, non-sensitive appearance/context.
7. Tell the person to take a Coke from the dog's back first, then pose for an instant photo.
8. Coach them to hold the Coke out front and center themselves in the frame.
9. Say a photographer cue, save the photo, play a print sound, tell them the photo is ready, and trigger a dance.

The prototype does not infer identity or sensitive traits. Humor is constrained to
visible non-sensitive context such as setting, posture, lighting, colors, bags,
objects nearby, or what is happening in the scene. Fetch's comedy voice is
confessional, observational, self-deprecating, and mildly exasperated by the
absurdity of ordinary beach logistics and being a tiny robot dog carrying soda.

## Run

```bash
cd /Users/seifip/GitHub/fetch/dimos
python -m dimos.experimental.fetch.iphone_middleware --host 0.0.0.0 --port 8455
```

For Record3D USB RGBD input:

```bash
cd /Users/seifip/GitHub/fetch/dimos
python -m dimos.experimental.fetch.iphone_middleware --host 0.0.0.0 --port 8455 --record3d
```

For a live Go2 on the dog Wi-Fi:

```bash
cd /Users/seifip/GitHub/fetch/dimos
python -m dimos.experimental.fetch.iphone_middleware \
  --host 0.0.0.0 \
  --port 8455 \
  --vision-provider gemini \
  --robot-ip 192.168.12.1 \
  --robot-connection-method local_ap
```

Open this on the iPhone or Mac:

```text
http://127.0.0.1:8455/fetch
```

Tap `Record` to start the 1 second scan loop.

## iPhone Depth

The browser version streams RGB camera frames and can play provider-generated audio.
Safari does not expose raw LiDAR depth to JavaScript, so true iPhone LiDAR testing
uses the optional Record3D USB adapter. Start Record3D with USB streaming enabled,
press the red record toggle, then run the middleware with `--record3d`.

The adapter exposes:

- `GET /record3d/status`
- `GET /record3d/latest.jpg`
- `GET /record3d/latest-depth.jpg`
- `GET /record3d/stream.mjpg`
- `GET /record3d/stream-depth.mjpg`
- `POST /record3d/restart`
- `POST /record3d/analyze`
- `POST /photos/save`
- `POST /realtime/client-secret`
- `POST /robot/preflight`
- `POST /robot/action`

Saved photos are written to both the local preview folder under
`static/captures` and `~/Library/Mobile Documents/com~apple~CloudDocs/fetch`
so they sync through iCloud Drive. They are also mirrored into
`~/Library/CloudStorage/GoogleDrive-seifip@gmail.com/My Drive/robodog-fetch`.

Record3D frames are analyzed with the RGB image plus a `depth_hint` containing
center-depth and near-object summaries in meters. The default OpenAI vision model
is `gpt-5-mini`.

## Gemini Vision

The fetch policy can use Gemini for image analysis through Gemini's
OpenAI-compatible API, without LangChain:

```bash
export GEMINI_API_KEY=<YOUR_KEY>
python -m dimos.experimental.fetch.iphone_middleware \
  --host 0.0.0.0 \
  --port 8455 \
  --vision-provider gemini
```

Gemini defaults to `gemini-2.5-flash-lite` with thinking disabled
(`reasoning_effort=none`); pass `--model` only to override it.
`GOOGLE_API_KEY` is also accepted as a fallback for Gemini. Browser audio
defaults to Cartesia Sonic TTS; pass `--tts-provider gemini` to use Gemini Live
TTS or `--tts-provider openai` to use OpenAI TTS instead.

## Audio

Browser speech uses `/speak` by default. The default `/speak` provider is
Cartesia Sonic (`sonic-3.5-2026-05-04`) and requires `CARTESIA_API_KEY`.
With `--tts-provider openai`, `/speak` uses OpenAI TTS with `tts-1` and requires
`OPENAI_API_KEY`. With `--tts-provider gemini`, `/speak` uses Gemini Live TTS
(`gemini-3.1-flash-live-preview`) and requires `GEMINI_API_KEY` or
`GOOGLE_API_KEY`. `/speak` also accepts a per-request `provider` so the browser
modal can switch at runtime.

OpenAI Realtime WebRTC is optional. Enable it explicitly with
`--tts-provider openai --enable-realtime`; the browser will try Realtime first
and fall back to `/speak` if setup or playback fails.

Demo note: `/realtime/client-secret` is intentionally unauthenticated for local
live-demo use. Add an access gate before running this adapter on a shared or
public network.

```bash
export OPENAI_API_KEY=<YOUR_KEY>
python -m dimos.experimental.fetch.iphone_middleware \
  --host 0.0.0.0 \
  --port 8455 \
  --tts-provider openai \
  --enable-realtime \
  --realtime-model gpt-realtime-2 \
  --realtime-reasoning-effort low \
  --tts-voice echo
```

For Gemini Live TTS as the primary browser audio route:

```bash
export GEMINI_API_KEY=<YOUR_KEY>
python -m dimos.experimental.fetch.iphone_middleware \
  --host 0.0.0.0 \
  --port 8455 \
  --tts-voice Charon
```

`gpt-realtime-translate` is not wired into this prototype yet. It uses the
dedicated `/v1/realtime/translations` flow for continuous live interpretation,
while Fetch currently only needs one-way dog speech for generated offer and
photo-coaching lines.

## Live Conversation (Coke vendor)

By default Fetch only *speaks* (one-way TTS). With `--conversation-mode gemini_live`
the dog instead holds a real two-way voice conversation once it reaches a person:
it tells a joke, **takes a drink order**, coaches the photo, takes the picture, and
celebrates — all driven by a persistent Gemini Live session with tool calling.

```bash
export GEMINI_API_KEY=<YOUR_KEY>
python -m dimos.experimental.fetch.iphone_middleware \
  --host 0.0.0.0 \
  --port 8455 \
  --vision-provider gemini \
  --conversation-mode gemini_live \
  --robot-ip 192.168.12.1 \
  --robot-connection-method local_ap
```

How it works:

- The vision state machine still drives `search → approach → greet`. At `greet` the
  browser waves, opens the phone microphone, and hands off to a persistent
  conversation session on the server.
- The browser streams microphone audio (16 kHz PCM) to the server over the existing
  `/fetch/ws` WebSocket; the server runs one `LiveConversationSession`
  (`conversation.py`) per interaction and streams the dog's voice (24 kHz PCM) back
  for playback. Turn-taking uses the Live API's server-side voice activity
  detection, and talking over the dog triggers barge-in.
- The model drives the dog through **tool calls**: `take_order` (records quantity —
  Coke only; the customer grabs a can from the dog's back), `take_photo`,
  `celebrate` (goodbye + dance), `do_trick`, and `stop_and_reset`. There is no
  mechanical dispenser.
- Photo framing reuses the existing vision `confirm_coke` policy: framing results
  are injected into the live session as hints so the dog's spoken coaching matches
  what the camera sees. The photo fires only when the person is centered,
  clearly holding the Coke, and the shot is well framed.
- The conversation persona, menu, and safety rules live in `conversation_prompt.py`
  and mirror the vision policy in `policy.py`.

Requirements and notes:

- Requires `GEMINI_API_KEY` or `GOOGLE_API_KEY`. The conversation model defaults to
  `gemini-3.1-flash-live-preview`; override with `--conversation-model`.
- The browser needs a secure context for microphone access. `http://localhost`
  and `http://127.0.0.1` count as secure, so `--no-ssl` is fine for local desktop
  testing, but the phone-over-LAN demo must use HTTPS (the default) or the browser
  blocks the mic.
- The WebSocket `hello` advertises `audio_route: "gemini_live_conversation"` and
  `conversation_enabled: true` so the browser knows to capture the mic. The
  conversation adds these `/fetch/ws` message types: browser → server
  `conversation_start`, `mic_chunk`, `conversation_stop`; server → browser
  `audio_out`, `transcript`, `interrupted`, `conversation_state`.
- If a conversation stalls for ~30 s, or the customer declines or walks away, the
  dog resets and resumes scanning.

## Audio mode switcher

The phone UI has an **Audio** button (top right) that opens a settings modal for
switching the audio path at runtime, without restarting the server:

- **Modes**: Live conversation, Gemini TTS (one-way), OpenAI TTS (one-way), or
  Cartesia TTS (one-way, Cartesia Sonic — lowest latency). Set the relevant keys
  (`OPENAI_API_KEY`, `GEMINI_API_KEY`/`GOOGLE_API_KEY`, `CARTESIA_API_KEY`) in the
  environment / `.env` to switch freely. The server advertises which keys are
  present (`openai_available` / `gemini_available` / `cartesia_available` in the
  `hello` message), and the modal disables any mode whose key is missing and
  defaults to one that works. Live additionally requires launching with
  `--conversation-mode gemini_live`.
- **Voice**: a shared voice name (alloy/echo/fable/onyx/nova/shimmer) — Gemini and
  Cartesia routes map it to a matching provider voice automatically.
- **Live model**: the Gemini Live model used for conversation.

The choice persists in `localStorage`. To have both TTS and Live available in the
modal, launch with `--conversation-mode gemini_live` (the `/speak` TTS routes stay
available alongside it). The modal sends the selected `provider`/`voice` with
`/speak`, and `voice`/`model` with `conversation_start`. OpenAI Realtime is not in
the modal — it remains a separate launch flag (`--enable-realtime`).

## WebSocket Frame

```json
{
  "type": "frame",
  "frame_id": 1,
  "image": "data:image/jpeg;base64,...",
  "depth_hint": null,
  "ts": 1779897600000
}
```

## Decision

```json
{
  "type": "decision",
  "state": "approach",
  "candidate_found": true,
  "target": {
    "bearing": "center",
    "range": "near",
    "description": "person smiling toward the dog",
    "free_hand_evidence": "they are looking over and seem able to take a Coke/photo",
    "busy_signals": ["none"]
  },
  "simulated_cmd_vel": {
    "linear_x": 0.22,
    "angular_z": 0.0,
    "duration_s": 0.9
  },
  "action": "wave_offer",
  "photo_ready": false,
  "coke_visible": false,
  "framing": {
    "person_visible": true,
    "coke_visible": false,
    "well_framed": false,
    "notes": ""
  },
  "line": "That counter stance says you are ready for the tiny robot VIP treatment. Grab a Coke from my back first, then I will take your instant photo."
}
```

## Robot Adapter Later

The `simulated_cmd_vel` output maps directly to the Go2 velocity path used in the
direct WebRTC scripts:

- `linear_x`: forward velocity.
- `angular_z`: yaw velocity.
- `duration_s`: command duration.

On the dog, use LiDAR/depth for the final `<4m` stop condition and keep obstacle
avoidance enabled.
