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
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
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
    _bg_threads: list[threading.Thread] = []
    _bg_threads_lock: threading.Lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        self._tts_node = GeminiTTSNode(voice=self.config.voice, model=self.config.model)
        self._audio_output = SounddeviceAudioOutput(sample_rate=24000)
        self._audio_output.consume_audio(self._tts_node.emit_audio())

    @rpc
    def stop(self) -> None:
        with self._bg_threads_lock:
            threads = list(self._bg_threads)
        for t in threads:
            t.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        if self._tts_node:
            self._tts_node.dispose()
            self._tts_node = None
        if self._audio_output:
            self._audio_output.stop()
            self._audio_output = None
        super().stop()

    @skill
    def speak(self, text: str, blocking: bool = True) -> str:
        """Speak text out loud through the robot's speakers.

        USE THIS TOOL AS OFTEN AS NEEDED. People can't normally see what you say in text, but can hear what you speak.

        Try to be as concise as possible. Remember that speaking takes time, so get to the point quickly.

        Example usage:

            speak("Hello, I am your robot assistant.")
        """
        if self._tts_node is None:
            return "Error: TTS not initialized"

        if not blocking:
            thread = threading.Thread(
                target=self._speak_bg, args=(text,), daemon=True, name="GeminiSpeakSkill-bg"
            )
            with self._bg_threads_lock:
                self._bg_threads.append(thread)
            thread.start()
            return f"Speaking (non-blocking): {text}"

        return self._speak_blocking(text)

    def _speak_bg(self, text: str) -> None:
        try:
            self._speak_blocking(text)
        finally:
            # Remove this thread from the list of background threads when done
            with self._bg_threads_lock:
                self._bg_threads = [
                    t for t in self._bg_threads if t is not threading.current_thread()
                ]

    def _speak_blocking(self, text: str) -> str:
        # Use lock to prevent simultaneous speech
        with self._audio_lock:
            if self._tts_node is None:
                return "Error: TTS not initialized"

            text_subject: Subject[str] = Subject()
            audio_complete = threading.Event()
            self._tts_node.consume_text(text_subject)

            def set_as_complete(_t: str) -> None:
                audio_complete.set()

            def set_as_complete_e(_e: Exception) -> None:
                audio_complete.set()

            subscription = self._tts_node.emit_text().subscribe(
                on_next=set_as_complete,
                on_error=set_as_complete_e,
            )

            text_subject.on_next(text)
            text_subject.on_completed()

            # Gemini synthesis is a network round-trip; allow more headroom than
            # the local-output time so first-token latency doesn't trip the wait.
            timeout = max(15, len(text) * 0.1)

            if not audio_complete.wait(timeout=timeout):
                logger.warning(f"TTS timeout reached for: {text}")
                subscription.dispose()
                return f"Warning: TTS timeout while speaking: {text}"
            else:
                # Small delay to ensure buffers flush
                time.sleep(0.3)

            subscription.dispose()

            return f"Spoke: {text}"


if __name__ == "__main__":
    skill_module = GeminiSpeakSkill()
    skill_module.start()
    print(skill_module.speak("Hello, I am your robot assistant, powered by Gemini."))
    skill_module.stop()
