# Copyright 2025 Dimensional Inc.
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

import threading
import time

from reactivex import Subject

from dimos.core.core import rpc
from dimos.core.skill_module import SkillModule
from dimos.protocol.skill.skill import skill
from dimos.stream.audio.node_output import SounddeviceAudioOutput
from dimos.stream.audio.tts.node_openai import OpenAITTSNode, Voice
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SpeakSkill(SkillModule):
    _tts_node: OpenAITTSNode | None = None
    _audio_output: SounddeviceAudioOutput | None = None
    _audio_lock: threading.Lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        self._tts_node = OpenAITTSNode(speed=1.2, voice=Voice.ONYX)
        self._audio_output = SounddeviceAudioOutput(sample_rate=24000)
        self._audio_output.consume_audio(self._tts_node.emit_audio())

    @rpc
    def stop(self) -> None:
        if self._tts_node:
            self._tts_node.dispose()
            self._tts_node = None
        if self._audio_output:
            self._audio_output.stop()
            self._audio_output = None
        super().stop()

    @skill()
    def speak(self, text: str) -> str:
        """Speak text out loud through the robot's speakers.

        USE THIS TOOL AS OFTEN AS NEEDED. People can't normally see what you say in text, but can hear what you speak.

        Try to be as concise as possible. Remember that speaking takes time, so get to the point quickly.

        Example usage:

            speak("Hello, I am your robot assistant.")
        """
        if self._tts_node is None:
            return "Error: TTS not initialized"

        # Use lock to prevent simultaneous speech
        with self._audio_lock:
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

            timeout = max(5, len(text) * 0.1)

            if not audio_complete.wait(timeout=timeout):
                logger.warning(f"TTS timeout reached for: {text}")
                subscription.dispose()
                return f"Warning: TTS timeout while speaking: {text}"
            else:
                # Small delay to ensure buffers flush
                time.sleep(0.3)

            subscription.dispose()

            return f"Spoke: {text}"


speak_skill = SpeakSkill.blueprint

__all__ = ["SpeakSkill", "speak_skill"]
