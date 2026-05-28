# Voice Interaction Roadmap

## Phase 1 — Expressive Output Voice

**Status**: Implemented as provider-aware `/speak` routing.

**Scope**: Add Gemini 3.1 Flash Live Preview for output-only speech synthesis
while preserving OpenAI TTS and making OpenAI Realtime optional.

**Problem solved**: OpenAI-only TTS made the voice route rigid and less expressive
for the beach demo. The app now has a Gemini Live TTS route for playful output
voice, while OpenAI TTS and OpenAI Realtime remain available for fallback or
experimentation.

**What changed**:
- `/speak` selects the provider with `--tts-provider`.
- `--tts-provider gemini` opens an ephemeral Gemini Live session, sends the line,
  collects PCM audio chunks, and returns WAV to the browser.
- `--tts-provider openai` keeps the existing OpenAI TTS MP3 route.
- `--enable-realtime` opts into OpenAI Realtime WebRTC before `/speak` fallback,
  but only when `--tts-provider openai` is selected.
- The browser learns the active route from the WebSocket `hello` message instead
  of guessing locally.

**What doesn't change**: The interaction flow. The dog still drives the state
machine. No audio input. No listening. Same composed lines, same triggers.

**Remaining gap**: Gemini Live audio is streamed inside the server session, but
the browser still receives a completed WAV blob. True chunked playback to the
browser remains a future optimization.

**UX win**: More expressive provider choice with predictable fallback behavior.
The dog sounds more alive without changing the state machine.

---

## Phase 2 — Responsive Monologue

**Scope**: Add audio input for paralinguistic signal detection, not full
speech-to-text.

**Problem solved**: People instinctively respond to the dog verbally. If the dog
ignores their response entirely, the social contract breaks — it feels like
talking to a recording. Acknowledging even a laugh or a "sure" with adjusted
timing makes the interaction feel reciprocal.

**What changes**:
- Browser streams microphone audio to the server via WebSocket.
- Server forwards audio to Gemini Live session with input modality enabled.
- Detect paralinguistic signals only:
  - **Laughter** → proceed with confidence (the joke landed).
  - **Confusion** ("huh?", "wait—") → slow down, rephrase.
  - **Yes/affirmative** → skip coaching, move to photo.
  - **Rejection** ("no thanks") → graceful exit.
  - **Silence duration** → gentle prompt if no response after greeting.
- State machine gains signal-driven transitions alongside vision-driven ones.
- Audio route selection must continue to keep output-only Gemini `/speak` as the
  default unless a listening mode is explicitly enabled.

**Why not full ASR**: Robust keyword/affect classification handles beach noise
better than transcription. Privacy-aligned — detecting intent, not recording
speech. Predictable branching — the state machine handles a small set of signals.

**UX win**: The dog seems aware of the person. The interaction feels reciprocal
without requiring full conversation.

---

## Phase 3 — Lightweight Dialogue

**Scope**: Full bidirectional conversation scoped tightly to the Fetch context.

**Problem solved**: People ask questions the dog can't answer ("What kind of
photo?", "Is this free?", "Can you do a different pose?"). A scripted monologue
can't handle these, so the person feels ignored even though they're engaged.

**What changes**:
- Live session persists across the full interaction (not ephemeral per call).
- System prompt defines the dog's personality, conversational boundaries, and
  recovery strategies. The dog is a beach character with a job — every response
  steers back toward the photo.
- Vision and audio run concurrently in the same session (multimodal Live API).
- Conversation context carries across states — the dog remembers what it said
  and what the person replied.
- OpenAI Realtime remains an optional comparison path for voice experiments, but
  any conversational mode needs one explicit owner for state, prompts, and turn
  lifecycle to avoid mixing providers in the same interaction.

**What doesn't change**: The interaction still has a goal (photo + Coke). The
dog still drives toward it. Conversation is a tool for engagement, not the
product.

**Critical design requirement**: A separate conversational design document
defining the dog's personality, boundaries, recovery from misunderstanding, and
exit strategies. This is a UX problem, not an engineering problem.

**UX win**: The interaction feels like engaging with a character, not operating
a device. The person feels heard, the dog feels present.
