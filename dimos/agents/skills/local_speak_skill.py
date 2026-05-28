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

"""A speak skill backed by the macOS ``say`` command.

Drop-in replacement for ``SpeakSkill`` that requires no OpenAI key and no
audio pipeline: ``say`` synthesizes and plays the audio itself. macOS only.
Satisfies ``SpeakSkillSpec`` (``speak(text, blocking=True) -> str``).
"""

import shutil
import subprocess
import threading

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class LocalSpeakSkillConfig(ModuleConfig):
    # macOS `say` voice name (e.g. "Daniel", "Samantha"); None = system default.
    voice: str | None = None
    # Speech rate in words per minute; None = `say` default (~175).
    rate: int | None = None


class LocalSpeakSkill(Module):
    """Speak text out loud through the local macOS ``say`` command."""

    config: LocalSpeakSkillConfig
    _bg_threads: list[threading.Thread] = []
    _bg_threads_lock: threading.Lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        if shutil.which("say") is None:
            logger.warning(
                "LocalSpeakSkill: `say` not found on PATH; speak() will no-op. "
                "This skill is macOS-only."
            )

    @rpc
    def stop(self) -> None:
        with self._bg_threads_lock:
            threads = list(self._bg_threads)
        for t in threads:
            t.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _command(self, text: str) -> list[str]:
        cmd = ["say"]
        if self.config.voice:
            cmd += ["-v", self.config.voice]
        if self.config.rate:
            cmd += ["-r", str(self.config.rate)]
        cmd.append(text)
        return cmd

    @skill
    def speak(self, text: str, blocking: bool = True) -> str:
        """Speak text out loud through the robot's speakers.

        USE THIS TOOL AS OFTEN AS NEEDED. People can't normally see what you say in text, but can hear what you speak.

        Try to be as concise as possible. Remember that speaking takes time, so get to the point quickly.

        Example usage:

            speak("Hello, I am your robot assistant.")
        """
        if shutil.which("say") is None:
            return "Error: `say` command not available (LocalSpeakSkill is macOS-only)"

        if not text.strip():
            return "Error: nothing to speak"

        if not blocking:
            thread = threading.Thread(
                target=self._speak_blocking, args=(text,), daemon=True, name="LocalSpeakSkill-bg"
            )
            with self._bg_threads_lock:
                self._bg_threads.append(thread)
            thread.start()
            return f"Speaking (non-blocking): {text}"

        return self._speak_blocking(text)

    def _speak_blocking(self, text: str) -> str:
        try:
            subprocess.run(self._command(text), check=True)
        except subprocess.CalledProcessError as e:
            logger.error(f"`say` failed: {e}")
            return f"Error: failed to speak: {text}"
        finally:
            with self._bg_threads_lock:
                self._bg_threads = [
                    t for t in self._bg_threads if t is not threading.current_thread()
                ]
        return f"Spoke: {text}"


if __name__ == "__main__":
    skill_module = LocalSpeakSkill()
    skill_module.start()
    print(skill_module.speak("Hello, I am your robot assistant, powered by Gemini."))
    skill_module.stop()
