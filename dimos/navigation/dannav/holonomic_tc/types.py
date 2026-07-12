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

"""Per-tick reference and measured samples for holonomic path following.

**Plan horizontal frame (``pose_plan``)**

- Positions and yaw use the same plan horizontal convention as ``Path`` and
  ``PathDistancer``.

**Body command frame (``twist_body``)**

- ``Twist.linear`` and ``Twist.angular`` follow the base / ``cmd_vel`` convention:
  planar speed uses ``hypot(linear.x, linear.y)`` with x forward and y lateral.

**Time (``time_s``)**

- Scalar time in seconds for this sample (odom timestamp in live ``LocalPlanner`` runs).

``Pose`` and ``Twist`` are copied in ``__post_init__`` so samples do not alias
caller-owned messages.
"""

from __future__ import annotations

from dataclasses import dataclass

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist


@dataclass(frozen=True)
class TrajectorySample:
    """Timed pose + body twist sample (shared shape)."""

    time_s: float
    pose_plan: Pose
    twist_body: Twist

    def __post_init__(self) -> None:
        object.__setattr__(self, "pose_plan", Pose(self.pose_plan))
        object.__setattr__(self, "twist_body", Twist(self.twist_body))


@dataclass(frozen=True)
class TrajectoryReferenceSample(TrajectorySample):
    """One timed point on the reference trajectory (target)."""


@dataclass(frozen=True)
class TrajectoryMeasuredSample(TrajectorySample):
    """One timed measurement of actual robot state for tracking."""
