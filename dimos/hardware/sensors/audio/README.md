# Audio Subsystem -- Handoff

**Owner (trial):** Zhuoran Guo · **Last updated:** 2026-06-22 · **PR #2507** · **Issue #1932**
**Location:** `dimos/hardware/sensors/audio/` (peer to `camera/`, `lidar/`)

---

## 1. What this is

The voice I/O path for dimos: **capture -> STT -> (agent) -> TTS -> effects -> speaker.**
It supplies the missing *speech-input* half of dimos's "command a robot in natural
language" story. Today it runs as a self-contained voice loopback; it is not yet wired
to the agent or to `memory2` (see section 5-6).

All five modules and both blueprints live in a single file:
`dimos/hardware/sensors/audio/module.py`.

---

## 2. Modules At A Glance

| Module | Role | Status | Verified |
|---|---|---|---|
| `AudioModule` | Mic capture -> `AudioStamped` (one msg per `frame_ms` chunk). Real path via `sounddevice`/PortAudio; `synthetic=True` sine-tone fallback needs no mic. | done | macOS, real mic + synthetic (50 Hz / 20 ms / 16 kHz mono) |
| `SpeechToTextModule` | VAD + AEC + segmentation + Whisper transcription, with multi-layer self-echo suppression. 3-backend fallback: `whisper.cpp` -> `faster-whisper` -> `openai-whisper`. | works | macOS loopback |
| `TextToSpeechModule` | Text -> speech. 3 providers: `openai` (default), `macos-say`, `pyttsx3`. Resamples to a common rate, chunks to frames, emits `tts_active` / `spoken_text` / `tts_reference_audio`. | works | macOS |
| `FunVoiceEffectsModule` | Real-time DSP chain: noise gate -> phase-vocoder pitch shift -> ring-mod ("robotize") -> bitcrush -> echo, via overlap-add STFT framing. | smoke-tested only | not fully validated - first thing to harden |
| `SpeakerModule` | Plays `AudioStamped` to the output device; (re)opens the stream on format change; emits `speaker_playing` for barge-in. | works | macOS |

**Blueprints** (module-level vars in `module.py`):
- `demo_audio` -- `AudioModule -> SpeakerModule` (mic monitor).
- `audio_speech_loopback` -- full chain wired via `autoconnect(...).remappings(...)`, including the anti-echo signal routing.

> CLI run-names: the blueprint variables are `demo_audio` and `audio_speech_loopback`.
> Confirm the exact `dimos run <name>` registration matches these before relying on the
> commands in section 3 (run-name != variable-name in some setups).

---

## 3. How To Run

```bash
# Dependencies (macOS)
brew install portaudio
pip install sounddevice numpy
# openai TTS additionally needs: pip install openai soundfile
# grant microphone permission on first real run

# Validate AudioModule in isolation (LCM round-trip + live capture-rate check)
python examples/audio/validate_audio_module.py            # synthetic (default, no mic)
python examples/audio/validate_audio_module.py --real-mic # real microphone

# Blueprints (confirm registered run-names first)
dimos run demo-audio              # mic -> speaker monitor
dimos run audio-speech-loopback   # full capture -> STT -> TTS -> effects -> speaker
```

`validate_audio_module.py` asserts: LCM encode/decode is lossless for PCM payload +
metadata + timestamp; frame rate ~= `1000 / frame_ms` Hz (50 Hz at 20 ms); timestamps
strictly increasing.

---

## 4. Architecture And Data Flow

Each module follows the established dimos shape (mirrors `CameraModule`): typed `In[]`/`Out[]`
streams, `@rpc start()/stop()`, and an `async def main()` with a single `yield`
(open resources before, tear down after). Heavy work (capture callback, transcription,
synthesis, DSP) runs on daemon threads behind bounded queues with drop-oldest backpressure.

**Loopback wiring** (`audio_speech_loopback`):

```text
AudioModule.audio ─┬─────────────────────────────► SpeechToTextModule.audio
 (mic_audio)       │                                        │ text (speech_text)
                   │                                        ▼
                   │                                TextToSpeechModule
                   │   tts_active ◄──── tts_active_signal ───┤
                   │   recent_tts_text ◄─── recent_tts_text ─┤  (self-echo guard)
                   │   tts_reference_audio ◄──── (AEC ref) ──┤
                   │                                         │ audio (tts_audio_raw)
                   │                                         ▼
                   │                                FunVoiceEffectsModule
                   │                                         │ audio_out (tts_audio)
                   │                                         ▼
                   └── speaker_playing_signal ◄──────── SpeakerModule  (barge-in)
```

**Message type -- `AudioStamped`** (`dimos/msgs/audio_msgs/AudioStamped.py`):
carries a `std_msgs.Header`, `sample_rate`, `channels`, `sample_format` (e.g. `S16LE`/`F32LE`),
`coding_format` (`pcm`), and raw PCM `data`. Helpers: `from_pcm(...)`, `to_numpy()`.

---

## 5. Key Design Decisions

- **Wire type is a stand-in.** `AudioStamped` serialises to `foxglove_msgs.RawAudio`
  because it is the only audio type currently mirrored in `dimos_lcm`. `RawAudio` has
  **no `frame_id` field**, so `frame_id` is *dropped on encode*. `format` is packed as
  `"{coding_format}/{sample_format}"` (e.g. `"pcm/S16LE"`). For cross-machine / multi-source
  use, add a native `Header`-bearing audio type to `dimos-lcm`. (This is documented in the
  `AudioStamped` module docstring; it is a pending team decision, not an endorsement of the
  foxglove schema.)

- **Self-echo / barge-in suppression is layered** (so the robot does not transcribe its own
  TTS):
  1. **Barge-in muting** -- STT drops live audio while `tts_active` or `speaker_playing` is
     true, plus a `tts_guard_seconds` tail.
  2. **Acoustic echo cancellation (AEC)** -- cross-correlation against a rolling buffer of
     `tts_reference_audio`; subtracts the reference only when correlation clears a threshold.
  3. **Self-text guard** -- fuzzy match (`SequenceMatcher`) of new transcripts against recent
     `recent_tts_text` within a window; drop near-duplicates of what we just said.
  4. **Consecutive-duplicate dedup** -- drop the same transcript repeated within a window.
  5. **Bad-transcript filter** -- drop Whisper non-speech captions (`[BLANK_AUDIO]`,
     `(music playing)`, etc.).

- **VAD + segmentation.** RMS-dB gate with hangover; prefer flushing an utterance on silence,
  with a fixed `segment_seconds` cap so the buffer can't grow unbounded during continuous speech.

- **Backend/provider fallback.** STT and TTS both degrade gracefully (try preferred backend,
  fall through the rest, and if none is available, drain the queue rather than crash).

---

## 6. Known Gaps / Not Done

1. **`memory2` not connected** -- *the original endpoint of Issue #1932.* Audio is currently
   "hear and forget": STT text / clips are never persisted. **This is the largest open item.**
2. **Agent not connected** -- the loop is mic -> STT -> TTS (an echo/effects toy), not yet
   STT -> agent reasoning -> TTS. Wiring the agent is what turns it from a parrot into something
   that can actually take spoken commands.
3. **Not validated on real robot (Go2 Pro / Jetson).** macOS -> Jetson has real gaps: audio
   device enumeration, TTS provider (`macos-say` is unavailable off macOS -> use `openai`/`pyttsx3`),
   and Whisper backend (needs an ARM-friendly backend, e.g. `whisper.cpp`).
4. **`FunVoiceEffectsModule` is smoke-tested only** -- the DSP chain runs and logs VU, but
   parameter ranges and stability across sample rates are not characterised.
5. **`RawAudio` stand-in** -- decide whether to add a native `Header`-bearing LCM audio type
   before multi-source / cross-machine audio is needed.

---

## 7. Suggested Next Steps

1. **On-robot bring-up (Go2 Pro):** device enumeration, switch TTS to `openai`/`pyttsx3`,
   tune AEC/VAD thresholds in a real acoustic environment.
2. **Connect `memory2`:** persist STT text (and optionally clips) -- the actual intent of #1932.
3. **Connect the agent:** `STT.text -> agent -> TTS.text`, so spoken language drives behaviour.
4. **Harden `FunVoiceEffects`** and resolve the `RawAudio` vs. native-`Header` type question.

---

## 8. Contact

Questions: Zhuoran Guo -- [add WeChat / email]
