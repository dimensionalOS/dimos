# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on the "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Typed samples for timed trajectory tracking (P1-1).

These types are **data-only**: no controller wiring, logging, or planner hooks.
They pair with the normative error definitions in
``dimos.navigation.trajectory_metrics``.

**Plan horizontal frame (``pose_plan``)**

- Positions and yaw use the same **plan** horizontal convention as
  ``Path``, ``PathDistancer``, and ``pose_errors_vs_reference``: x forward,
  y left in that frame (often the ``map`` frame in deployed stacks).
- Reference and measured poses must live in **one mutually consistent** plan
  frame when computing errors (for example both ``map``, or both ``odom`` with
  a known relationship). The type does not enforce which TF frame ID that is;
  integrators document the choice at the seam.

**Body command frame (``twist_body``)**

- ``Twist.linear`` and ``Twist.angular`` follow the **base / cmd_vel**
  convention used in ``trajectory_metrics.commanded_planar_speed``: planar
  speed uses ``hypot(linear.x, linear.y)`` with x forward and y lateral in the
  robot body axes.
- A spatial path expressed in the plan frame is typically rotated into this
  body frame using the **reference** yaw when populating reference twists.

**Time (``time_s``)**

- Scalar time in seconds for this sample. May be trajectory parameter time,
  wall time, or simulation time; the producer defines the convention. Loggers
  should record which convention is used once P2 work lands.

**Ownership**

- ``Pose`` and ``Twist`` message types are mutable. Each frozen dataclass below
  stores **defensive copies** constructed in ``__post_init__`` so holding a
  sample does not alias caller-owned messages.
"""

from __future__ import annotations

from dataclasses import dataclass

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist


@dataclass(frozen=True)
class TrajectoryReferenceSample:
    """One timed point on the reference trajectory (target)."""

    time_s: float
    pose_plan: Pose
    twist_body: Twist

    def __post_init__(self) -> None:
        object.__setattr__(self, "pose_plan", Pose(self.pose_plan))
        object.__setattr__(self, "twist_body", Twist(self.twist_body))


@dataclass(frozen=True)
class TrajectoryMeasuredSample:
    """One timed measurement of actual robot state for tracking."""

    time_s: float
    pose_plan: Pose
    twist_body: Twist

    def __post_init__(self) -> None:
        object.__setattr__(self, "pose_plan", Pose(self.pose_plan))
        object.__setattr__(self, "twist_body", Twist(self.twist_body))


__all__ = [
    "TrajectoryMeasuredSample",
    "TrajectoryReferenceSample",
]
