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

"""Making every trial an episode of the learning-stack recorder: start before
the program runs, save it when the judge says success, discard otherwise."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
import time

from dimos.imitation.collection.episode import EpisodeStatus
from dimos.imitation.collection.episode_monitor import EpisodeCommand, EpisodeControlSpec


@dataclass
class Episode:
    """The trial's episode. Set ``success`` before the block ends; ``index``
    is the saved episode's position in the recording, or None when it was
    discarded or nothing records."""

    success: bool | None = None
    index: int | None = None


@contextmanager
def trial_episode(monitor: EpisodeControlSpec) -> Iterator[Episode]:
    """Record the block as one episode. The block must set ``episode.success``;
    a block that raises, or ends without a verdict, is discarded."""
    monitor.command("start")
    episode = Episode()
    try:
        yield episode
    except BaseException:
        monitor.command("discard")
        raise
    if episode.success is None:
        monitor.command("discard")
        raise RuntimeError("the trial ended without a verdict; its episode was discarded")
    if not episode.success:
        monitor.command("discard")
        return
    status = monitor.command("save")
    if status.episodes_saved:
        episode.index = status.episodes_saved - 1


class NoEpisodes:
    """Stands in for the recorder on a stack without one: commands are
    accepted, nothing is recorded and no episode gets an index."""

    def get_status(self) -> EpisodeStatus:
        return self._idle()

    def command(self, event: EpisodeCommand) -> EpisodeStatus:
        return self._idle()

    @staticmethod
    def _idle() -> EpisodeStatus:
        return EpisodeStatus(ts=time.time(), state="idle", episodes_saved=0, episodes_discarded=0)
