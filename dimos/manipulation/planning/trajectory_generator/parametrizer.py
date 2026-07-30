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

"""Internal path-to-trajectory parametrization boundary."""

from dataclasses import dataclass
import math
from typing import Protocol

from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory


class TrajectoryParametrizationError(ValueError):
    """A path could not be converted into a valid timed trajectory."""


@dataclass(frozen=True)
class TrajectoryParametrizationRequest:
    """Canonical input for an untimed selected-joint path."""

    group_ids: tuple[PlanningGroupID, ...]
    joint_names: tuple[str, ...]
    path: tuple[JointState, ...]
    velocity_limits: tuple[float, ...] | None = None
    acceleration_limits: tuple[float, ...] | None = None
    speed_scale: float = 1.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.speed_scale) or self.speed_scale <= 0.0 or self.speed_scale > 1.0:
            raise ValueError("speed_scale must be finite, > 0, and <= 1")


@dataclass(frozen=True)
class ParametrizedTrajectory:
    """Canonical output plus the limits and accelerations used to validate it."""

    trajectory: JointTrajectory
    velocity_limits: tuple[float, ...]
    acceleration_limits: tuple[float, ...]
    accelerations: tuple[tuple[float, ...], ...] | None = None


class TrajectoryParametrizer(Protocol):
    """Convert an untimed geometric path into one timed trajectory."""

    @property
    def uses_request_limits(self) -> bool: ...

    def parametrize(self, request: TrajectoryParametrizationRequest) -> ParametrizedTrajectory: ...
