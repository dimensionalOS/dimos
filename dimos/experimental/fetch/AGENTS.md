# AGENTS.md — robodog-fetch

## What This Is

A prototype for a robot dog "Fetch" behavior: the dog scans for a relaxed beachgoer, approaches them, offers a Coke in exchange for a photo, takes the photo, and dances. The behavior is a vision-driven state machine analyzed per camera frame via LLM vision calls. It runs as a standalone FastAPI + WebSocket server that can use a phone browser camera, a Record3D USB stream, or a live Unitree Go2 WebRTC camera/LiDAR stream.

This package lives **inside the larger DimOS monorepo** at `dimos/experimental/fetch/`. The standalone repo (`robodog-fetch`) mirrors those files. The run commands reference the monorepo path.

## Running

```bash
# From the DimOS monorepo root:
python -m dimos.experimental.fetch.iphone_middleware --host 0.0.0.0 --port 8455

# With Record3D USB RGBD input:
python -m dimos.experimental.fetch.iphone_middleware --host 0.0.0.0 --port 8455 --record3d

# With a live Unitree Go2 robot:
python -m dimos.experimental.fetch.iphone_middleware \
  --host 0.0.0.0 --port 8455 \
  --vision-provider gemini \
  --robot-ip 192.168.12.1 --robot-connection-method local_ap

# Disable HTTPS for local debugging:
python -m dimos.experimental.fetch.iphone_middleware --host 0.0.0.0 --port 8455 --no-ssl

# Use Gemini vision instead of OpenAI:
export GEMINI_API_KEY=<KEY>
python -m dimos.experimental.fetch.iphone_middleware --host 0.0.0.0 --port 8455 --vision-provider gemini

# Use Gemini Live TTS as the primary browser audio route:
export GEMINI_API_KEY=<KEY>
python -m dimos.experimental.fetch.iphone_middleware --host 0.0.0.0 --port 8455 --tts-voice Charon

# Opt into OpenAI Realtime before /speak fallback:
export OPENAI_API_KEY=<KEY>
python -m dimos.experimental.fetch.iphone_middleware --host 0.0.0.0 --port 8455 --tts-provider openai --enable-realtime
```

Open `http://127.0.0.1:8455/fetch` on the phone or browser.

## Testing

No test runner config (`pyproject.toml`, `pytest.ini`, `setup.cfg`) exists. Tests use `pytest` conventions:

```bash
# From this repo directory:
pytest -q

# Focused policy tests:
pytest test_policy.py
```

Tests mock provider clients via `unittest.mock.patch` and local module stubs — they never hit real APIs. `test_iphone_middleware.py` stubs DimOS, Unitree, and OpenCV dependencies so middleware routing can be tested from this standalone mirror.

## Architecture

### State Machine

```
search → approach → greet → wait_for_coke → photo_ready
  ↓                                          ↑
  └──────────────── skip ←───────────────────┘
```

- **search**: Scan left/right (angular_z != 0) looking for a candidate.
- **approach**: Move toward target (linear_x > 0, bearing-based angular_z).
- **greet** (inside_4m): Wave and deliver personalized joke/offer line.
- **wait_for_coke**: Person hasn't framed the Coke yet; coach them.
- **photo_ready**: Coke and person well-framed; take photo and dance.
- **skip**: Unsafe or blocked; resume searching.

Two **interaction phases** drive different prompts:
- `find_guest`: The default — locate and evaluate a beachgoer.
- `confirm_coke`: After greeting — check if the person is holding the Coke and ready for a photo.

### File Roles

| File | Role |
|---|---|
| `policy.py` | Core state machine: `FetchPolicy.analyze_frame()` sends image+prompt to vision LLM, parses JSON response into a normalized decision dict. All state logic lives here. |
| `iphone_middleware.py` | FastAPI server (`FetchIphoneMiddleware`): WebSocket endpoint for browser, Record3D, and Go2 frame routing; REST endpoints for Record3D/Go2 frames, robot commands, TTS, Realtime client secrets, and photo capture. |
| `record3d_source.py` | Background thread (`Record3DSource`) that reads RGBD frames from Record3D USB, encodes to JPEG, and produces depth hints (median distances, confidence). |
| `tts.py` | TTS provider helpers: Gemini Live TTS, voice-name mapping, and PCM-to-WAV conversion. |
| `static/index.html` | Single-page phone UI: camera feed, Go2/Record3D previews, controls, decision display, audio routing, and photo flow. |
| `test_policy.py` | Unit tests for `policy.py` — provider routing, JSON parsing, config validation, client caching. |
| `test_iphone_middleware.py` | Middleware tests for Realtime setup, audio route advertisement, and Gemini `/speak`. |
| `test_tts.py` | Unit tests for Gemini voice mapping, WAV conversion, and Live TTS response handling. |

### Data Flow

1. Phone browser, Record3D, or Go2 captures a frame → sent via WebSocket (`type: "frame"`, `type: "record3d_frame"`, or `type: "dog_frame"`) or REST analyze endpoints.
2. `FetchPolicy.analyze_frame()` builds a vision prompt with the image and optional depth hint, calls the LLM (OpenAI or Gemini).
3. LLM returns raw JSON → `_extract_json_object()` parses it (tolerates markdown fences, extra surrounding text).
4. `_normalize_decision()` maps raw fields to canonical state/action/cmd_vel.
5. Decision dict sent back to client over WebSocket or REST response.
6. Client may call `/robot/action` (move, wave, dance, stand) or `/speak` based on the decision. `/speak` uses Gemini Live TTS by default; pass `--tts-provider openai` to use OpenAI TTS instead. OpenAI Realtime is opt-in with `--tts-provider openai --enable-realtime` and falls back to `/speak`.

## Key Gotchas

- **This is a standalone copy of a DimOS monorepo package.** The imports in `iphone_middleware.py` reference `dimos.experimental.fetch.*`, `dimos.robot.unitree.*`, `dimos.web.dimos_interface.*`, etc. Running standalone requires the DimOS monorepo on `PYTHONPATH`. The `test_policy.py` file imports `policy` directly (top-level) — it only works when run from this directory, not from the monorepo.

- **Gemini uses the OpenAI-compatible API.** Both providers go through `openai.OpenAI`. Gemini just gets `base_url="https://generativelanguage.googleapis.com/v1beta/openai/"`. The client class is always `openai.OpenAI`.

- **Gemini retries without `response_format`.** If the Gemini model doesn't support `json_object` response format, `_create_completion` catches the error and retries without it (`policy.py:493-503`).

- **API key env vars vary by provider:**
  - OpenAI: `OPENAI_API_KEY`
  - Gemini: `GEMINI_API_KEY` or `GOOGLE_API_KEY` (fallback chain)
  - OpenAI TTS and Realtime: `OPENAI_API_KEY`
  - Gemini Live TTS: `GEMINI_API_KEY` or `GOOGLE_API_KEY`

- **Audio routing is provider-aware.** Browser speech uses `/speak` by default. Gemini Live TTS is the default route. `--enable-realtime` only enables OpenAI Realtime when `--tts-provider openai`; the browser learns this from the WebSocket `hello` message fields `tts_provider`, `audio_route`, and `realtime_enabled`.

- **`/realtime/client-secret` is local-demo unauthenticated.** It is disabled by default and only responds when OpenAI Realtime is enabled, but it still needs an access gate before shared-network or public deployment.

- **Frozen dataclass with `__post_init__` validation.** `FetchPolicyConfig` uses `object.__setattr__` to normalize fields in `__post_init__`. You can't mutate it after construction.

- **Client caching.** `FetchPolicy._get_client()` caches the OpenAI client keyed on `(api_key, provider, model, timeout, retries)`. Changing `fetch_policy.config` between calls creates a new client if the cache key differs.

- **`simulated_cmd_vel` is not just for simulation.** Despite the name, these velocity commands map directly to Go2 robot Twist commands. The middleware sends them via `UnitreeWebRTCConnection.move()`.

- **Robot sport API IDs are magic numbers.** The `_sport()` helper sends raw `api_id` integers over WebRTC: 1006=recovery_stand, 1002=balance_stand, 1027=switch_joystick, 1016=wave, 1022=dance.

- **Depth hints are statistical summaries, not raw depth.** `Record3DSource` and Go2 LiDAR summarize depth/point-cloud data into JSON hints. These are passed in the vision prompt — the LLM uses them to estimate range.

- **Photos save to `static/captures/`.** This directory is gitignored. The `/photos/save` endpoint accepts an `image_data_url` from the browser or `source: "dog"` / `source: "record3d"` for server-side captures.

- **SSL by default.** The server runs HTTPS using DimOS teleop certs from `assets/teleop_certs`. Use `--no-ssl` for local development.

## Conventions

- **License header**: Apache 2.0 on all source files except `__init__.py` and `test_policy.py`.
- **Type annotations**: Full annotations throughout, using `from __future__ import annotations` for modern syntax (`X | None` instead of `Optional[X]`).
- **Literal types for enums**: `ApproachState`, `InteractionPhase`, `Bearing`, `RangeEstimate`, `VisionProvider` are all `Literal[...]` types, not enums. Validation is done with membership checks against string sets.
- **Line length**: Project does not enforce a line length limit (many lines exceed 79 chars; pylsp E501 warnings are present but ignored).
- **No logging framework in policy.py**: The policy module is pure logic with no logger. `iphone_middleware.py` and `record3d_source.py` use `dimos.utils.logging_config.setup_logger()`.
