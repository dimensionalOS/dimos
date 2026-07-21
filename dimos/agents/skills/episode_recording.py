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

"""Agent skills for recording labeled training-data episodes.

Exposes start/save/discard of a recording episode to the LLM as MCP tools. The
skills forward to an injected ``EpisodeMonitorModule`` (via ``EpisodeControlSpec``),
which drives the same start/save/discard state machine used by the teleop button
path. The recorded episode is later exported to LeRobot/HDF5 with ``dimos dataprep``.
"""

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.learning.collection.episode_monitor_spec import EpisodeControlSpec


class EpisodeRecordingSkillContainer(Module):
    """Lets the agent record labeled training episodes on natural-language command."""

    _episode: EpisodeControlSpec  # injected: the EpisodeMonitorModule

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def start_recording(self, task_label: str) -> str:
        """Begin recording a training-data episode for later model training.

        Call this when the user asks to record, capture, or demonstrate a task.
        Recording continues until stop_recording or discard_recording is called.

        Args:
            task_label: short description of the task being demonstrated,
                e.g. "navigate to the kitchen".

        Example:
            start_recording("walk to the door")
        """
        return self._episode.set_episode("start", task_label)

    @skill
    def stop_recording(self) -> str:
        """Stop and SAVE the current episode as a successful demonstration.

        Use this when the demonstrated task finished successfully and the take
        should be kept for training.
        """
        return self._episode.set_episode("save")

    @skill
    def discard_recording(self) -> str:
        """Stop and DISCARD the current episode (a bad or unwanted take).

        Use this to throw away a take that should not be used for training.
        """
        return self._episode.set_episode("discard")
