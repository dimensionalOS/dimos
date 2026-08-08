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

"""Registry of the text-only SPACE tasks, one line per task."""

from __future__ import annotations

from dataclasses import dataclass

from dimos.benchmark.space_qa.sampling import DEFAULT_GROUP_SIZE


@dataclass(frozen=True, kw_only=True)
class SpaceTextTask:
    """Where a task's questions live upstream and how many stimuli it holds.

    `groups` is the stimulus count reported in the SPACE paper; each stimulus is
    asked `group_size` times with the answer in a different slot.
    """

    name: str
    groups: int
    group_size: int = DEFAULT_GROUP_SIZE

    def __post_init__(self) -> None:
        if not self.name or self.groups < 1 or self.group_size < 1:
            raise ValueError(f"invalid task registration: {self.name!r}")

    @property
    def qas_relative_path(self) -> str:
        """Path to the task's questions, relative to the release root."""
        return f"{self.name}/qas.json"

    @property
    def expected_rows(self) -> int:
        return self.groups * self.group_size


SPACE_TEXT_TASKS: tuple[SpaceTextTask, ...] = (
    SpaceTextTask(name="PTT_text", groups=100),
    SpaceTextTask(name="JLO_text", groups=50),
    SpaceTextTask(name="MPFB_text", groups=50),
    SpaceTextTask(name="MRT_text", groups=40),
    SpaceTextTask(name="SAtt_text", groups=100),
    SpaceTextTask(name="SAdd_text", groups=50),
    SpaceTextTask(name="CBTT_text", groups=50),
    SpaceTextTask(name="DirectionEstimationBEVText", groups=150),
    SpaceTextTask(name="DistanceEstimationBEVText", groups=135),
    SpaceTextTask(name="MapSketchingBEVText", groups=30),
)


def space_text_task(name: str) -> SpaceTextTask:
    for task in SPACE_TEXT_TASKS:
        if task.name == name:
            return task
    raise KeyError(f"unregistered task: {name!r}")
