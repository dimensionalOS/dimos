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

"""Formal time-parameterization stage for manipulation planning.

Turns a geometric joint-space path (a ``PlanningResult`` whose ``path`` is
untimed joint waypoints) into a time-parameterized ``JointTrajectory``.
"""

from __future__ import annotations

import math
from typing import Protocol, runtime_checkable

from dimos.manipulation.planning.spec.models import PlanningResult
from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
    JointTrajectoryGenerator,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


@runtime_checkable
class TimeParameterizer(Protocol):
    """Converts a geometric planning result into a timed trajectory."""

    def parameterize(self, result: PlanningResult) -> JointTrajectory: ...


class TrapezoidalTimeParameterizer:
    """Default time-parameterization stage.

    - Planner-provided timestamps (from optimization-based planners) are honored
      exactly, preserving the planner's timing.
    - Untimed geometric planners (e.g. RRT) get a trapezoidal velocity profile
      synthesized via ``JointTrajectoryGenerator``.
    """

    def __init__(self, generator: JointTrajectoryGenerator) -> None:
        self._generator = generator

    def parameterize(self, result: PlanningResult) -> JointTrajectory:
        waypoints = [list(state.position) for state in result.path]

        # Only a genuinely absent timeline (None) means "untimed" -> synthesize a
        # trapezoidal profile. An empty list for a non-empty path is a mismatch,
        # caught by the length check below (not silently re-timed).
        if result.timestamps is None:
            return self._generator.generate(waypoints)

        # Planner supplied a timeline: it must line up with the waypoints...
        if len(result.timestamps) != len(waypoints):
            raise ValueError(
                f"timestamps ({len(result.timestamps)}) and path "
                f"({len(waypoints)}) length mismatch"
            )
        # ...and be a valid, executable timeline. NaN/inf slip past ordinary
        # comparisons (any comparison with NaN is False; inf gives an infinite
        # duration the controller never completes), so require finite first.
        if any(not math.isfinite(t) for t in result.timestamps):
            raise ValueError(
                f"timestamps must all be finite, got {result.timestamps}"
            )
        # ...non-negative and strictly increasing (a zero-duration or
        # non-monotonic timeline is rejected by the controller at execution).
        if result.timestamps[0] < 0.0:
            raise ValueError(
                f"timestamps must be non-negative, got {result.timestamps[0]}"
            )
        if any(b <= a for a, b in zip(result.timestamps, result.timestamps[1:])):
            raise ValueError(
                f"timestamps must be strictly increasing, got {result.timestamps}"
            )

        points = [
            TrajectoryPoint(time_from_start=t, positions=list(p))
            for t, p in zip(result.timestamps, waypoints)
        ]
        return JointTrajectory(points=points)
