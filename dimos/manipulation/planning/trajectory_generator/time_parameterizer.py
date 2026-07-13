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
untimed joint waypoints) into a time-parameterized ``JointTrajectory``. This is
the explicit boundary between geometric planning and execution: planners produce
geometry, this stage produces time.
"""

from __future__ import annotations

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

        # Untimed planner: synthesize timing with a trapezoidal profile.
        if not result.timestamps:
            return self._generator.generate(waypoints)

        # Planner supplied timing: preserve it exactly.
        if len(result.timestamps) != len(waypoints):
            raise ValueError(
                f"timestamps ({len(result.timestamps)}) and path "
                f"({len(waypoints)}) length mismatch"
            )
        points = [
            TrajectoryPoint(time_from_start=t, positions=list(p))
            for t, p in zip(result.timestamps, waypoints)
        ]
        return JointTrajectory(points=points)
