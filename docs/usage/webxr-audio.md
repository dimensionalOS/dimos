# Offline speech in WebXR collection

Spoken recording feedback is **off by default**. Enable it with `--tts.enabled=true` when
starting a WebXR collection blueprint. The computer synthesizes speech locally
using Kokoro INT8; the browser plays WAV audio through Web Audio.

## One-time setup

Install the dependencies and download verified model assets in one command:

```bash
uv run --extra manipulation --extra tts python -m dimos.stream.audio.tts.setup
```

This installs Kokoro and CPU ONNX Runtime, then downloads the **114 MB INT8
model** and **28 MB voice data** from the pinned
[Kokoro ONNX release](https://github.com/thewh1teagle/kokoro-onnx/releases/tag/model-files-v1.1).
The assets are stored in `${XDG_CACHE_HOME:-$HOME/.cache}/dimos/tts`.
Each file is checked against its pinned SHA-256 hash before installation.
Rerunning setup reuses valid files and downloads missing or corrupt files again.
Failed downloads do not replace existing files or leave partial model files.

Use `uv run --extra manipulation --extra tts` for subsequent launches, or
`uv run --no-sync` to use the environment already installed by setup.
`--tts.enabled=true` controls runtime behavior and does not install dependencies.

The `all` extra deliberately does not include `tts`. With TTS disabled, collection
requires neither Kokoro dependencies nor model files. The named `tts` module
starts idle and loads its inference engine only when `enabled=true`.

After setup, synthesis needs no internet access. Collection startup never
downloads assets; if a file is missing, its error includes the setup command.
Custom model and voice paths remain available through the TTS module's ordinary
configuration flags. Setup installs only the default assets in the cache above.

## Enable prompts

Add `--tts.enabled=true` to your existing collection launch, retaining its robot, task,
camera, and recording options. For example, the launch documented in
[Imitation Learning for Manipulation](/docs/capabilities/manipulation/imitation-learning.md)
becomes `uv run --extra tts dimos run dual-openyam-quest-collection --tts.enabled=true ...`.

Then attach the terminal controls as usual:

```bash
uv run dimos imitation collect
```

`--tts.enabled=true` belongs to **stack startup**, not the attached terminal command. Restart
the stack to change it. `--tts.enabled=false` explicitly disables it. This is the `enabled` field on
`KokoroTTSConfig`, scoped to the module instance named `tts`; there is no global
TTS flag or `DIMOS_TTS` setting.

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

The page silently preloads the three phrases. A new phrase can take several
seconds to synthesize on CPU, while repeated phrases use caches. New events
interrupt speech and invalidate older pending responses. Leaving XR or losing
the WebSocket connection stops playback. Nothing is replayed on reconnect.

If playback or a speech request fails, the page and collection HUD show
**Audio unavailable**. Collection and controller input continue to work.

## Manual headset test

Use a fresh test recording directory and retain the robot and camera options
from your working collection launch.

1. Complete the one-time setup above. Launch your collection blueprint with
   `uv run --extra tts dimos run BLUEPRINT --tts.enabled=true` and its usual options.
2. Open the server's `/teleop` page in the headset browser. Wait about 15 seconds
   for the initial three phrases to preload, then select **Connect** and enter
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

## Extend the prompts

`KokoroTTSModule.synthesize(text: str) -> bytes` returns mono PCM16 WAV through
the `SpeechSynthesisSpec` RPC contract. It accepts nonblank text up to 500
characters, uses English voice `af_sarah` by default, and caches 128 phrases.
The module has no recording-specific behavior and does not play sound on the
computer's speakers.

The web server exposes `POST /teleop/speech` with JSON `{"text":"…"}` and an
`audio/wav` response. It runs synthesis RPC off the server's event loop. The
browser's `SpeechPlayer` owns playback and cancellation; `CollectionPrompts`
owns recording-event wording. Add phrases to that mapping or call the speech
helper from another UI feature.

The existing video/status WebSocket carries no audio. Its status envelope has
a `snapshot` boolean identifying cached connection state.

## Validation

```bash
uv run pytest dimos/stream/audio/tts/test_kokoro_module.py dimos/teleop/webxr/test_module.py
node dimos/teleop/webxr/web/test_speech.mjs
```

The INT8 model was exercised on CPU with outbound socket connections blocked,
including a new phrase, “Episode 12 saved.” Browser-independent logic tests
cover reconnects, cancellation, cache reuse, and out-of-order responses.
An automated Chromium check exercised the real HTTP endpoint, WAV decoding,
cached playback, and stopping audio. Cached playback was scheduled in about
2.2 ms on the development machine; cold CPU synthesis took about 4 seconds per
phrase. These measurements do not include physical headset output latency.

Physical headset testing remains required for each supported browser family:
enter an immersive session, test B/start/save and Y/discard plus terminal
commands, reconnect silently, and check that video and controls stay responsive.
Measure event receipt to cached playback start; the local-network target is
under 250 ms. WebXR support alone does not guarantee browser audio permissions
or a particular system output device.
