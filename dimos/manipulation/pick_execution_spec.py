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

"""Manipulation operations used by the selected-object pick transaction."""

from typing import Protocol

from dimos.agents.skill_result import SkillResult
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.spec.utils import Spec


class PickExecutionSpec(Spec, Protocol):
    def open_gripper(self, robot_name: str | None = None) -> SkillResult: ...

    def close_gripper(self, robot_name: str | None = None) -> SkillResult: ...

    def get_gripper(self, robot_name: str | None = None) -> float | None: ...

    def get_ee_pose(self, robot_name: str | None = None) -> Pose | None: ...

    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        robot_name: str | None = None,
    ) -> SkillResult: ...
