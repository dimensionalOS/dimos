# 2026-05-27 — camera + audio web bridges, Go2 audio scaffold

Branch: `feat/gemini-go2-2245`

## Goal
Stream the Go2 (sim or real) camera into a web page consumable by mlxvlm,
then add the same shape for robot audio (mic out, speaker in).

## Camera

### `dimos/web/mjpeg_module.py` (new) — `camera-mjpeg-module`
- Module subscribes to `color_image: In[Image]` and republishes via FastAPI on
  `:7780`.
- Endpoints:
  - `GET /video_feed/color_image` — MJPEG (`multipart/x-mixed-replace`)
  - `GET /snapshot/color_image` — single JPEG (for backend pull, e.g. mlxvlm
    `/api/analyze` server-side)
- CORS allow-all so a browser at a different origin can `drawImage` onto a
  canvas without tainting.
- Default port moved from 5555 → 7780 to avoid common collisions.

### `dimos/robot/all_blueprints.py`
- Registered `"camera-mjpeg-module": "dimos.web.mjpeg_module.CameraMjpegModule"`.

### Use
```
dimos --simulation run unitree-go2-basic camera-mjpeg-module
# http://127.0.0.1:7780/video_feed/color_image
# http://127.0.0.1:7780/snapshot/color_image
```
Same URLs work against the real Go2 — drop `--simulation`, set `ROBOT_IP`.

## Audio (real-robot only — sim has no audio)

### `dimos/robot/unitree/connection.py` (modified)
- Added `AudioMessage(data: bytes, sample_rate: int, channels: int)` dataclass.
- `UnitreeWebRTCConnection.audio_stream()` — subscribes to
  `LegionConnection.audio` via `add_track_callback`, calls
  `switchAudioChannel(True)`, emits `AudioMessage` (int16 PCM at the frame's
  native rate, usually 48 kHz mono).
- `UnitreeWebRTCConnection.play_wav_bytes(wav: bytes)` — writes a tempfile,
  uploads via `WebRTCAudioHub.upload_megaphone`, then `enter_megaphone`. Fires
  on the connection's asyncio loop; returns immediately.

### `dimos/robot/unitree/go2/connection.py` (modified)
- New streams on `GO2Connection`:
  - `audio: Out[bytes]` — mic data (currently only the PCM bytes — see
    review issue #1, metadata is dropped).
  - `audio_in: In[bytes]` — WAV bytes to play through the speaker.
- `start()` wires both if the connection supports them (`hasattr` guard for
  sim).
- New skill `play_wav(wav_path: str)` — agent-facing, reads local WAV and
  invokes `play_wav_bytes`.

### `dimos/web/audio_ws_module.py` (new) — `audio-ws-module`
- Module with `audio: In[bytes]` (mic) and `audio_in: Out[bytes]` (speaker).
- FastAPI on `:7781`:
  - `WebSocket /audio_out` — pushes binary PCM frames to connected clients.
  - `POST /play` — body = WAV bytes; publishes to `audio_in` for the robot
    speaker.
- CORS open on the HTTP routes.

### `dimos/robot/all_blueprints.py`
- Registered `"audio-ws-module": "dimos.web.audio_ws_module.AudioWsModule"`.

### Use (real Go2 only)
```
dimos run unitree-go2-basic audio-ws-module
# ws://127.0.0.1:7781/audio_out         (mic out, binary)
# curl --data-binary @clip.wav http://127.0.0.1:7781/play
```

## Other changes

### `go2-start.sh`
- Quickstart script for the dimairos05 Go2 — wifi check, ping, NTP sync, venv
  bootstrap, env-var sanity, `exec dimos run <blueprint>`. Useful at the
  hackathon table.

### Editable install
- Switched the venv from a one-shot `uv pip install 'dimos[sim] @ .'` to
  `uv pip install -e '.[sim]'` so source edits apply without reinstall.

## Review fixes applied
- `GO2Connection.audio` retyped `Out[AudioMessage]`; rate + channels carry
  through to `audio-ws-module`, which now exposes them via `GET /audio_info` and
  sends an initial `{"event":"format", ...}` JSON frame on WS connect.
- `play_wav_bytes` reads WAV duration via `wave`, sleeps for it, then calls
  `exit_megaphone` so the robot doesn't stay in megaphone mode.
- Float frames from aiortc are now scaled `(arr * 32767).clip(...)` before
  the int16 cast instead of being silently truncated.
- Added `play_wav_b64` skill alongside `play_wav` for remote MCP clients
  that don't share the dimos host's filesystem.
- Moved `os` / `tempfile` / `wave` imports to module top.

## Known issues / follow-ups
1. Stereo packed-vs-planar layout — `arr.shape[0]` assumes planar; Go2 mic
   is mono so OK in practice. Document, don't fix yet.
2. `audio-ws-module` alone (without `unitree-go2-basic`) has an unbound `In` —
   autoconnect doesn't warn loudly enough.
3. Untested: no real Go2 available during this session. Sim has no audio.

## Mac + local-LLM agentic — what works, what doesn't

Goal: drive an MCP agent on Apple Silicon with LM Studio (:1234) or the
mlxvlm Gemma-4 server (:8080), no CUDA, no Google services.

### Blueprints

| Blueprint | Boots on Mac? | Notes |
|---|---|---|
| `unitree-go2-agentic` | ✗ | imports SecurityModule (EdgeTAM/CUDA) and Moondream VL (crashes Metal) |
| `unitree-go2-agentic-gemini` | ✗ as-is | imports `GeminiSpeakSkill` → `from google import genai` at module load. Run `uv pip install google-genai` first, even if you don't call any Gemini APIs. |
| `unitree-go2-agentic-ollama` | ✓ | clean compose; `./sim-with-llm.sh ollama` uses it |
| `unitree-go2-basic + mcp-server + mcp-client` | ✓ | minimal but reliable; default for `./sim-with-llm.sh lmstudio` and `mlxvlm` |

### Skill availability by compose

| Compose | Skills the agent gets |
|---|---|
| `unitree-go2-basic + mcp-server + mcp-client` | `observe`, `play_wav`, `play_wav_b64` (last two no-op in sim) |
| `… + unitree-skill-container + replanning-a-star-planner` | adds `relative_move`, `wait`, `current_time`, `tilt_body`, `execute_sport_command` — heavier startup (full nav stack) |

`unitree-skill-container` alone fails build:
`"No module met NavigationInterfaceSpec spec"`. The implementor is
`replanning-a-star-planner`.

### VL backends — Mac gotcha

`dimos/models/vl/types.py:17` literal: `qwen | moondream | gemini`.

- `qwen` — **not local**: hits Alibaba DashScope cloud (`dashscope-intl.aliyuncs.com`), needs `ALIBABA_API_KEY`.
- `moondream` — local; crashes Metal on Apple Silicon (per the `-gemini` blueprint docstring).
- `gemini` — Google cloud, needs `GOOGLE_API_KEY`.

There is currently **no Mac-local VL backend**. The user has mlxvlm on :8080
(Gemma-4-E4B + Falcon-Perception, OpenAI-compatible) — natural place to plug
a new `mlxvlm` / `openai_compat` backend. Not implemented yet; needs a
new file under `dimos/models/vl/`.

### LLM chat backends

- **LM Studio at :1234** and **mlxvlm at :8080** both speak `/v1/chat/completions`
  with proper OpenAI tool-call passthrough — `langchain-openai` (already
  installed) handles them via `OPENAI_BASE_URL`. No new dimos module
  needed for chat.
- McpClient model override via `-o mcpclient.model=openai:<id>` works.
  Wired up in `go2-start.sh` LMSTUDIO/MLXVLM presets.

### Next iteration
Add `mlxvlm` to `VlModelName` so `NavigationSkillContainer` and
`PersonFollowSkillContainer` can run with a local VL backend. With that
+ `google-genai` installed, `unitree-go2-agentic-gemini` becomes the
richest Mac agentic compose if we also `--disable` the Gemini-runtime
modules (`gemini-speak-skill`, `map-uploader`, `spatial-memory`).

## Composition cheatsheet
| Run | Endpoints |
|---|---|
| `unitree-go2-basic camera-mjpeg-module` | `:7780/video_feed/color_image`, `:7780/snapshot/color_image` |
| `unitree-go2-basic audio-ws-module` | `ws://:7781/audio_out`, `POST :7781/play` |
| `unitree-go2-basic camera-mjpeg-module audio-ws-module` | both |
| `unitree-go2-agentic camera-mjpeg-module audio-ws-module` | MCP + agent + both bridges |
