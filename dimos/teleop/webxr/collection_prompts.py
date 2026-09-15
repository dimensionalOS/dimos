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
            status.ts,
            status.state,
            status.last_event,
            status.episodes_saved,
            status.episodes_discarded,
        ) == (
            previous.ts,
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
