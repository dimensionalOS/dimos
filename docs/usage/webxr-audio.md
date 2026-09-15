# Offline speech in WebXR collection

Spoken recording feedback is **off by default**. Enable it with `--tts.enabled=true` when
starting a WebXR collection blueprint. The computer synthesizes speech locally
using the official Kokoro library on CPU; the browser plays WAV audio through Web Audio.

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

Before modules start, WebXR prepares the speech helper. It downloads the official
[Kokoro v1.0 model](https://huggingface.co/hexgrad/Kokoro-82M) (about 327 MB),
configuration and the selected voice (about 0.5 MB) at a pinned revision, then
generates the three recording phrases. A fresh process with cached assets took
about 5 seconds to prepare all three prompts on the development machine; first-use
downloads add startup time. No separate setup command is needed.

Kokoro uses PyTorch on CPU. On the first enabled startup, Misaki uses spaCy's
installer to install the missing `en_core_web_sm` English tokenizer into the
Python environment. This requires internet access and pip or uv. Later starts
reuse the installed tokenizer. Misaki's English dependencies provide the bundled
eSpeak pronunciation support.

Assets are stored under `${XDG_CACHE_HOME:-$HOME/.cache}/dimos/assets/huggingface`
through the [shared model-asset helper](/docs/usage/model-assets.md). Cached files are reused
without network access. A first launch needs internet access; a failed download
reports its source and can be retried by launching again. `dimos cache clean`
removes the downloaded models after collection stops. Old ONNX assets and custom
ONNX-path options are no longer used.

The default American English voice is `af_sarah`. Select another American English
voice from the official model with `--tts.voice=af_heart`. TTS configuration is
limited to `enabled` and `voice`; cache location follows dimOS's XDG cache root.

Disabled TTS imports no inference dependencies, checks no assets, and creates no
speech helper or worker. The `all` extra does not include `tts`. To have `uv run`
sync dependencies on launch, include `--extra manipulation --extra tts` before
`dimos`. Install the `tts` extra explicitly; the module flag only triggers automatic
tokenizer installation and model downloads. An exact `uv sync` can remove the
lazily installed tokenizer because it is not in the lockfile; the next enabled
startup installs it again. Use `uv run --no-sync` to preserve the prepared environment.

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
   `uv run --no-sync dimos run BLUEPRINT --tts.enabled=true` and its usual options.
2. Open the server's `/teleop` page in the headset browser. Select **Connect** and enter
   XR. Connecting should be silent. Ensure the headset volume is audible.
3. In another terminal, run `uv run --no-sync dimos imitation collect`. Use the table
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
7. After a successful enabled startup, stop the stack and block outbound internet
   access while keeping the headset's local connection available. Relaunch with
   `uv run --no-sync` and TTS enabled. Cached preparation and all three prompts
   should work without downloads or installation.

Record the headset model, browser/version, and whether audio is audible during
XR. The Chromium smoke test does not replace this check on each browser family.

## Data flow and extending prompts

`KokoroTTS` is an ordinary library helper owned by WebXR. Its nested configuration
controls whether speech is enabled and which voice is used, and `synthesize(text: str) -> bytes` returns mono PCM16
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
uv run --no-sync pytest dimos/stream/audio/tts/test_kokoro.py dimos/utils/test_assets.py dimos/teleop/webxr/test_module.py dimos/teleop/webxr/test_collection_prompts.py
node dimos/teleop/webxr/web/test_speech.mjs
```

Physical headset testing remains required: verify start/save/discard, silent
reconnects, prompt interruption, and responsive video and controls. Browser smoke
checks do not measure physical headset output latency.
