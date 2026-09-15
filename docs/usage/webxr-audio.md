# Offline speech in WebXR collection

Spoken recording feedback is **off by default**. Enable it with `--tts.enabled=true` when
starting a WebXR collection blueprint. The computer synthesizes speech locally
using Kokoro INT8; the browser plays WAV audio through Web Audio.

## First launch

Install the optional inference dependencies:

```bash
uv sync --extra manipulation --extra tts
```

Add `--tts.enabled=true` to your existing collection launch, retaining your robot,
camera, task, and recording options:

```bash
uv run --no-sync dimos run dual-openyam-quest-collection --tts.enabled=true ...
```

TTS is nested configuration on the WebXR module. The fully qualified flag for
collection is `--armteleopmodule.tts.enabled=true`; `--tts.enabled=true` is the
ordinary unambiguous shorthand. Use `--tts.enabled=false` to disable it.

Before modules start, WebXR prepares the speech helper. It downloads the
**114 MB INT8 model** and **28 MB voice data** from the pinned
[Kokoro ONNX release](https://github.com/thewh1teagle/kokoro-onnx/releases/tag/model-files-v1.1)
if needed, verifies their SHA-256 hashes, then generates the three recording
phrases. Phrase generation took about 12 seconds on the development machine;
first-time downloads add to that startup time. No separate setup command is needed.

Assets are cached in `${XDG_CACHE_HOME:-$HOME/.cache}/dimos/tts`. Valid files are
reused without network access; missing or corrupt default assets are downloaded
again. Failed downloads never install partial files. Rerun collection to retry.
Custom model and voice paths use existing files as supplied; missing custom files
produce a path error rather than downloading defaults into those locations.

Disabled TTS imports no inference dependencies, checks no assets, and creates no
speech helper or worker. The `all` extra does not include `tts`. To have `uv run`
sync dependencies on launch, include `--extra manipulation --extra tts` before
`dimos`; the module flag does not install Python packages.

Attach terminal controls as usual:

```bash
uv run --no-sync dimos imitation collect
```

Supported collection blueprints:

- `learning-collect-webxr-xarm7`
- `learning-collect-webxr-piper`
- `openyam-quest-collection`
- `dual-openyam-quest-collection`

Open the teleoperation page in the headset and select **Connect**. This user
gesture enables browser audio. Speech uses the headset/system volume.

| Accepted episode event | Prompt |
| --- | --- |
| Start | Recording started |
| Save | Episode saved |
| Discard/cancel | Recording canceled |

Both controller buttons and terminal commands produce the same feedback.
Connection snapshots, initialization, duplicate events, and save/discard while
idle remain silent. Saving confirms the episode transition shown by the HUD;
it is not a separate disk-flush acknowledgment.

Python selects already prepared audio for each accepted event; recording
callbacks perform no inference. New events interrupt speech and invalidate
older pending audio decoding. Leaving XR or losing
the WebSocket connection stops playback. Nothing is replayed on reconnect.

If browser playback fails, the page and collection HUD show
**Audio unavailable**. Collection and controller input continue to work.

## Manual headset test

Use a fresh test recording directory and retain the robot and camera options
from your working collection launch.

1. Install the optional dependencies above. Launch your collection blueprint with
   `uv run --extra tts dimos run BLUEPRINT --tts.enabled=true` and its usual options.
2. Open the server's `/teleop` page in the headset browser. Select **Connect** and enter
   XR. Connecting should be silent. Ensure the headset volume is audible.
3. In another terminal, run `uv run dimos imitation collect`. Use the table
   below to check controller and terminal inputs independently.

| Starting state | Controller / terminal input | Expected result |
| --- | --- | --- |
| Idle | B / Space | HUD shows recording; hear “Recording started” once |
| Recording | B / Space | Saved count increases; hear “Episode saved” once |
| Idle | B / Space, then Y / D | Recording starts, then discarded count increases; hear “Recording canceled” |
| Idle | Y / D | No count increase and no speech |

4. Start and immediately save or discard. The newest prompt should interrupt
   any previous speech; an older response must not play afterward. Video,
   controller input, and the HUD should remain responsive.
5. Leave XR while speech is playing. Playback should stop. Re-enter XR, and
   disconnect/reconnect the headset network: no old event should be spoken.
   Start another episode to confirm feedback resumes.
6. Stop the stack and relaunch with `--tts.enabled=false`, omitting `--extra tts`. Reload
   the headset page. Recording and the HUD should work silently, with no
   **Audio unavailable** warning. This mode must also start in an environment
   without Kokoro installed or its model assets present.

Record the headset model, browser/version, and whether audio is audible during
XR. The Chromium smoke test does not replace this check on each browser family.

## Data flow and extending prompts

`KokoroTTS` is an ordinary library helper owned by WebXR. Its nested configuration
controls assets and voice, and `synthesize(text: str) -> bytes` returns mono PCM16
WAV. It accepts nonblank text up to 500 characters and caches 128 phrases.

Python's `CollectionPrompts` selects feedback from confirmed episode status.
Add wording to `RECORDING_PROMPTS` and the corresponding Python event selection;
WebXR prepares every phrase in that mapping during build. Its status callback
only looks up cached audio and schedules delivery.

The existing WebSocket sends the status message followed by an optional JSON
speech message: `{"type":"speech","audio":"<base64 WAV>"}`. Binary frames remain
JPEG video. The browser decodes and plays speech; it does not choose phrases or
request synthesis. Connection snapshots contain only status, never historical
speech. New events interrupt playback, and disconnects invalidate pending decoding.

## Validation

```bash
uv run --no-sync pytest dimos/stream/audio/tts/test_kokoro.py dimos/stream/audio/tts/test_assets.py dimos/teleop/webxr/test_module.py dimos/teleop/webxr/test_collection_prompts.py
node dimos/teleop/webxr/web/test_speech.mjs
```

Physical headset testing remains required: verify start/save/discard, silent
reconnects, prompt interruption, and responsive video and controls. Browser smoke
checks do not measure physical headset output latency.
