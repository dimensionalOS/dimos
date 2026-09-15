# Copyright 2026 Dimensional Inc.
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

"""Spoken feedback for confirmed collection transitions."""

from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.stream.audio.tts.kokoro import KokoroTTS, KokoroTTSConfig

RECORDING_PROMPTS = {
    "start": "Recording started",
    "save": "Episode saved",
    "discard": "Recording canceled",
}


class CollectionPrompts:
    def __init__(self) -> None:
        self.previous: EpisodeStatus | None = None

    def update(self, status: EpisodeStatus) -> str | None:
        previous = self.previous
        self.previous = status
        if previous is not None and (
            status.state,
            status.last_event,
            status.episodes_saved,
            status.episodes_discarded,
        ) == (
            previous.state,
            previous.last_event,
            previous.episodes_saved,
            previous.episodes_discarded,
        ):
            return None
        if status.last_event == "start" and status.state == "recording":
            return RECORDING_PROMPTS["start"]
        if previous is not None:
            if status.last_event == "save" and status.episodes_saved > previous.episodes_saved:
                return RECORDING_PROMPTS["save"]
            if (
                status.last_event == "discard"
                and status.episodes_discarded > previous.episodes_discarded
            ):
                return RECORDING_PROMPTS["discard"]
        return None


class CollectionSpeech:
    """Prepare collection feedback once, then select WAVs for confirmed transitions."""

    def __init__(self, config: KokoroTTSConfig) -> None:
        self._config = config
        self._prompts = CollectionPrompts()
        self._audio: dict[str, bytes] = {}

    def prepare(self) -> None:
        if not self._config.enabled:
            return
        speech = KokoroTTS(self._config)
        try:
            speech.prepare()
            self._audio = {
                phrase: speech.synthesize(phrase) for phrase in RECORDING_PROMPTS.values()
            }
        finally:
            # All feedback is cached; collection needs no live inference engine.
            speech.close()

    def update(self, status: EpisodeStatus, *, snapshot: bool = False) -> bytes | None:
        phrase = self._prompts.update(status)
        return self._audio.get(phrase) if phrase is not None and not snapshot else None
