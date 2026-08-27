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

"""Manipulation capabilities the selected-object pick transaction consumes.

Split by provider. Motion is a skill-layer verb, while the gripper, the reach
standoff, and the low-start lift are primitives the planning module owns —
only it can see the robot model's gripper hardware and its collision world.
"""

from typing import Protocol

from dimos.agents.skill_result import SkillResult
from dimos.manipulation.manipulation_spec import CommandResult
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.spec.utils import Spec


class PickMotionSpec(Spec, Protocol):
    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        planning_group: PlanningGroupID | None = None,
    ) -> SkillResult[ManipulationSkillError]: ...


class PickExecutionSpec(Spec, Protocol):
    def get_pre_grasp_offset(self) -> float: ...

    def lift_if_low(
        self, min_z: float = 0.05, planning_group: PlanningGroupID | None = None
    ) -> CommandResult: ...

    def open_gripper_and_settle(
        self, planning_group: PlanningGroupID | None = None
    ) -> CommandResult: ...

    def close_gripper_and_verify(
        self, planning_group: PlanningGroupID | None = None
    ) -> CommandResult: ...
