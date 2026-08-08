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

"""Pink implementation of the manipulation planning kinematics contract."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.kinematics.pink_solver import _PinkSolverCore
from dimos.manipulation.planning.spec.models import IKResult, WorldRobotID
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState

PinkIKConfig = PinkKinematicsConfig


class PinkIK(_PinkSolverCore):
    """Planning-only Pink backend implementing ``KinematicsSpec``."""

    def solve(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        target_pose: PoseStamped,
        seed: JointState | None = None,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
        check_collision: bool = True,
        max_attempts: int = 10,
    ) -> IKResult:
        """Solve one planning pose target."""
        return self._solve(
            world=world,
            robot_id=robot_id,
            target_pose=target_pose,
            seed=seed,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            check_collision=check_collision,
            max_attempts=max_attempts,
        )

    def solve_pose_targets(
        self,
        world: WorldSpec,
        pose_targets: Mapping[PlanningGroup, PoseStamped],
        auxiliary_groups: Sequence[PlanningGroup] = (),
        seed: JointState | None = None,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
        check_collision: bool = True,
        max_attempts: int = 10,
    ) -> IKResult:
        """Solve planning-group-scoped pose targets."""
        return self._solve_pose_targets(
            world=world,
            pose_targets=pose_targets,
            auxiliary_groups=auxiliary_groups,
            seed=seed,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            check_collision=check_collision,
            max_attempts=max_attempts,
        )
