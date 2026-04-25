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

"""Deterministic holonomic plant for trajectory control tests (921 P3-2).

Ideal omnidirectional base: commanded body ``Twist`` is integrated in the
plan frame with yaw rate ``angular.z``. No actuator lag or noise; a richer
model for calibration (P5-2) can extend or replace this later.
"""

from __future__ import annotations

import math

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample


def _pose_from_planar(x: float, y: float, yaw_rad: float) -> Pose:
    return Pose(
        x,
        y,
        0.0,
        0.0,
        0.0,
        math.sin(yaw_rad / 2.0),
        math.cos(yaw_rad / 2.0),
    )


class IntegratedHolonomicPlant:
    """Plan-frame pose state updated from body-frame ``cmd_vel``."""

    def __init__(self, *, x: float = 0.0, y: float = 0.0, yaw_rad: float = 0.0) -> None:
        self._x = float(x)
        self._y = float(y)
        self._yaw = float(yaw_rad)

    @property
    def x(self) -> float:
        return self._x

    @property
    def y(self) -> float:
        return self._y

    @property
    def yaw_rad(self) -> float:
        return self._yaw

    def measured_sample(self, time_s: float, cmd_applied: Twist) -> TrajectoryMeasuredSample:
        """Build a measurement sample; ``cmd_applied`` is the twist used at this instant."""
        return TrajectoryMeasuredSample(
            time_s=float(time_s),
            pose_plan=_pose_from_planar(self._x, self._y, self._yaw),
            twist_body=Twist(cmd_applied),
        )

    def step(self, cmd_body: Twist, dt_s: float) -> None:
        """Integrate one step with body-frame planar command (same convention as ``cmd_vel``)."""
        if not (math.isfinite(dt_s) and dt_s > 0.0):
            raise ValueError(f"dt_s must be finite and positive, got {dt_s!r}")
        vx = float(cmd_body.linear.x)
        vy = float(cmd_body.linear.y)
        wz = float(cmd_body.angular.z)
        c = math.cos(self._yaw)
        s = math.sin(self._yaw)
        vx_w = c * vx - s * vy
        vy_w = s * vx + c * vy
        self._x += vx_w * dt_s
        self._y += vy_w * dt_s
        self._yaw += wz * dt_s
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))


__all__ = ["IntegratedHolonomicPlant"]
