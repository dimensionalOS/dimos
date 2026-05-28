# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A speak skill backed by Google's Gemini TTS API.

Drop-in replacement for ``SpeakSkill`` that reuses ``GOOGLE_API_KEY`` (already
used for the LLM and embeddings) instead of an OpenAI key, and works on any
platform with an audio output device. Satisfies ``SpeakSkillSpec``
(``speak(text, blocking=True) -> str``).
"""

import threading
import time

from reactivex import Subject

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.stream.audio.node_output import SounddeviceAudioOutput
from dimos.stream.audio.tts.node_gemini import GeminiTTSNode
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GeminiSpeakSkillConfig(ModuleConfig):
    # Prebuilt Gemini voice name (e.g. "Kore", "Puck", "Charon", "Aoede").
    voice: str = "Kore"
    # Gemini TTS model; must be a `*-preview-tts` model to emit audio.
    model: str = "gemini-2.5-flash-preview-tts"


class GeminiSpeakSkill(Module):
    config: GeminiSpeakSkillConfig
    _tts_node: GeminiTTSNode | None = None
    _audio_output: SounddeviceAudioOutput | None = None
    _audio_lock: threading.Lock = threading.Lock()
    _text_subject: "Subject[str] | None" = None

    @rpc
    def start(self) -> None:
        super().start()
        self._tts_node = GeminiTTSNode(voice=self.config.voice, model=self.config.model)
        self._audio_output = SounddeviceAudioOutput(sample_rate=24000)
        self._audio_output.consume_audio(self._tts_node.emit_audio())
        # Wire the text pipeline ONCE. Each speak() just pushes onto this subject;
        # the TTS node's own worker drains it FIFO. (Previously consume_text was
        # called per speak(), spawning a fresh worker thread + subscription every
        # call — a leak, and the source of the repeated "Starting GeminiTTSNode".)
        self._text_subject = Subject()
        self._tts_node.consume_text(self._text_subject)

    @rpc
    def stop(self) -> None:
        if self._tts_node:
            # dispose() clears the queue and joins the worker, so in-flight/queued
            # speech is torn down here — no separate bg-thread bookkeeping needed.
            self._tts_node.dispose()
            self._tts_node = None
        if self._audio_output:
            self._audio_output.stop()
            self._audio_output = None
        self._text_subject = None
        super().stop()

    @skill
    def speak(self, text: str, blocking: bool = False) -> str:
        """Speak text out loud through the robot's speakers.

        USE THIS TOOL AS OFTEN AS NEEDED. People can't normally see what you say in text, but can hear what you speak.

        Try to be as concise as possible. Remember that speaking takes time, so get to the point quickly.

        Returns immediately by default (the audio plays in the background); pass
        ``blocking=True`` only when you must wait until the utterance finishes.

        Example usage:

            speak("Hello, I am your robot assistant.")
        """
        if self._tts_node is None or self._text_subject is None:
            return "Error: TTS not initialized"

        if not blocking:
            # Fire-and-forget: enqueue on the shared pipeline and return now.
            self._text_subject.on_next(text)
            return f"Speaking (non-blocking): {text}"

        return self._speak_blocking(text)

    def _speak_blocking(self, text: str) -> str:
        # Serialize blocking speech so utterances don't overlap on the speaker.
        with self._audio_lock:
            if self._tts_node is None or self._text_subject is None:
                return "Error: TTS not initialized"

            audio_complete = threading.Event()

            # emit_text() re-emits the exact utterance once its synthesis finishes;
            # match on the text so a concurrent non-blocking speak can't trip us.
            def on_text(t: str) -> None:
                if t == text:
                    audio_complete.set()

            def on_error(_e: Exception) -> None:
                audio_complete.set()

            subscription = self._tts_node.emit_text().subscribe(
                on_next=on_text,
                on_error=on_error,
            )

            self._text_subject.on_next(text)

            # Gemini synthesis is a network round-trip; allow more headroom than
            # the local-output time so first-token latency doesn't trip the wait.
            timeout = max(15, len(text) * 0.1)
            try:
                if not audio_complete.wait(timeout=timeout):
                    logger.warning(f"TTS timeout reached for: {text}")
                    return f"Warning: TTS timeout while speaking: {text}"
                # Small delay to ensure buffers flush.
                time.sleep(0.3)
                return f"Spoke: {text}"
            finally:
                subscription.dispose()


if __name__ == "__main__":
    skill_module = GeminiSpeakSkill()
    skill_module.start()
    print(skill_module.speak("Hello, I am your robot assistant, powered by Gemini.", blocking=True))
    skill_module.stop()
