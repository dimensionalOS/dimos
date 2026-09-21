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

from __future__ import annotations

from typing import ClassVar, Literal, TypeAlias, cast

from dimos_lcm.imitation_msgs import EpisodeStatus as LCMEpisodeStatus
from pydantic import BaseModel

from dimos.msgs.std_msgs.Header import Header

EpisodeEvent: TypeAlias = Literal["start", "save", "discard", "init"]
RecordingState: TypeAlias = Literal["idle", "recording"]


class EpisodeStatus(BaseModel):
    """Source-timestamped status update for an imitation-learning episode."""

    msg_name: ClassVar[str] = "imitation_msgs.EpisodeStatus"

    ts: float
    state: RecordingState
    episodes_saved: int
    episodes_discarded: int
    last_event: EpisodeEvent = "init"
    task_label: str | None = None

    def lcm_encode(self) -> bytes:
        return cast(
            "bytes",
            LCMEpisodeStatus(
                header=Header(self.ts),
                state=self.state,
                episodes_saved=self.episodes_saved,
                episodes_discarded=self.episodes_discarded,
                last_event=self.last_event,
                task_label=self.task_label or "",
            ).lcm_encode(),
        )

    @classmethod
    def lcm_decode(cls, data: bytes) -> EpisodeStatus:
        message = LCMEpisodeStatus.lcm_decode(data)
        stamp = message.header.stamp
        return cls(
            ts=stamp.sec + stamp.nsec / 1_000_000_000,
            state=message.state,
            episodes_saved=message.episodes_saved,
            episodes_discarded=message.episodes_discarded,
            last_event=message.last_event,
            task_label=message.task_label or None,
        )
