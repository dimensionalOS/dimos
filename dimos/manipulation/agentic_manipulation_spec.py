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

"""Spec contract for agent-facing manipulation primitive providers."""

from __future__ import annotations

from typing import Protocol

from dimos.agents.skill_result import SkillResult
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.spec.utils import Spec


class ManipulationControlSpec(Spec, Protocol):
    def get_robot_state(
        self, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]: ...
    def move_to_joints(
        self, joints: str, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]: ...
    def set_motion_speed(self, speed_scale: float) -> bool: ...
    def get_motion_speed(self) -> float: ...
    def open_gripper(
        self, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]: ...
    def close_gripper(
        self, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]: ...
