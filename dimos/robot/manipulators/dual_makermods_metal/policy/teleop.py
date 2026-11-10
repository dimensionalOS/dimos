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

"""WebXR deadman, freshness and goal mapping for Metal (session authority stays in the task)."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.control.tasks.teleop_input import TeleopInput
from dimos.robot.manipulators.dual_makermods_metal.policy.targets import (
    ControllerPose,
    DualHandGoal,
    WebXRGoalMapper,
)

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


class StationKinematics(Protocol):
    def pose24(self, q: NDArray[np.float32] | NDArray[np.float64]) -> NDArray[np.float64]: ...


def controller_pose(pose: Pose | PoseStamped) -> ControllerPose:
    """Convert a coordinator pose message to a validated station-frame sample."""
    quaternion = pose.orientation
    return ControllerPose.from_pose(
        tuple(pose.position),
        (float(quaternion.x), float(quaternion.y), float(quaternion.z), float(quaternion.w)),
    )


class MetalTeleop:
    def __init__(
        self, kinematics: StationKinematics, mapper: WebXRGoalMapper | None = None
    ) -> None:
        self.input = TeleopInput()
        self._mapper = mapper or WebXRGoalMapper()
        self._kinematics = kinematics

    @property
    def engaged(self) -> bool:
        return self._mapper.engaged

    def update(self, now: float, timeout: float) -> tuple[NDArray[np.float64], float] | None:
        """Latest mapped goal while every source is fresh, with its receipt time."""
        left, right = self.input.pose("left"), self.input.pose("right")
        if not self._mapper.engaged or left is None or right is None:
            return None
        if not self.input.fresh(now, timeout):
            return None
        try:
            goal = self._mapper.update(controller_pose(left[0]), controller_pose(right[0]))
        except ValueError:
            return None
        return goal.to_pose24(), min(left[1], right[1])

    def anchor(
        self, q: NDArray[np.float32], now: float, timeout: float
    ) -> tuple[NDArray[np.float64], float] | None:
        """Anchor the mapping at the measured TCP pose if every input is fresh."""
        left, right = self.input.pose("left"), self.input.pose("right")
        if left is None or right is None or not self.input.fresh(now, timeout):
            return None
        try:
            current = DualHandGoal.from_pose24(self._kinematics.pose24(q))
            self._mapper.engage(current, controller_pose(left[0]), controller_pose(right[0]))
        except (ValueError, RuntimeError):
            return None
        return current.to_pose24(), min(left[1], right[1])

    def release(self) -> None:
        """Drop samples and the anchor; a new session needs fresh input."""
        self.input.clear()
        self._mapper.release()
