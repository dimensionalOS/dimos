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

"""Small, observable turn-walk-turn controller for G1 watering.

This is intentionally not a navigation stack. It follows the straight path to
one explicit world-frame stance: first turn toward it, then walk forward with a
small heading correction, then turn to the final pour yaw. It never strafes or
reverses, and every tick reports its phase and errors for debugging.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from dimos.robot.unitree.g1.manip_stance import wrap_angle

MAX_LINEAR = 0.25
MAX_ANGULAR = 0.4
MIN_LINEAR = 0.06
MIN_ANGULAR = 0.08


@dataclass(frozen=True)
class Twist2D:
    """A planar velocity command: forward, left, and turn."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    @property
    def is_stop(self) -> bool:
        return self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0


class ApproachPhase(str, Enum):
    TURN_TO_PATH = "turn_to_path"
    WALK_FORWARD = "walk_forward"
    TURN_TO_STANCE = "turn_to_stance"
    ARRIVED = "arrived"
    BLOCKED = "blocked"


@dataclass(frozen=True)
class ApproachControllerConfig:
    max_linear: float = MAX_LINEAR
    max_angular: float = MAX_ANGULAR
    max_drive_angular: float = 0.18
    min_linear: float = MIN_LINEAR
    min_angular: float = MIN_ANGULAR
    linear_gain: float = 0.8
    angular_gain: float = 1.2
    drive_heading_tolerance: float = math.radians(12.0)
    position_tolerance: float = 0.06
    yaw_tolerance: float = math.radians(5.0)
    max_approach_distance: float = 2.0


@dataclass(frozen=True)
class ApproachStep:
    phase: ApproachPhase
    command: Twist2D
    distance_error: float
    path_heading_error: float
    stance_yaw_error: float
    message: str

    @property
    def arrived(self) -> bool:
        return self.phase is ApproachPhase.ARRIVED

    @property
    def blocked(self) -> bool:
        return self.phase is ApproachPhase.BLOCKED


def _usable_speed(value: float, floor: float, ceiling: float) -> float:
    if abs(value) < 1e-9:
        return 0.0
    return math.copysign(min(max(abs(value), floor), ceiling), value)


def approach_step(
    current: tuple[float, float, float],
    goal: tuple[float, float, float],
    config: ApproachControllerConfig = ApproachControllerConfig(),
) -> ApproachStep:
    """Compute one safe command toward a world-frame stance.

    ``current`` and ``goal`` are ``(x, y, yaw)``. The lateral command is
    always zero and the forward command is never negative.
    """
    values = (*current, *goal)
    if not all(math.isfinite(value) for value in values):
        return ApproachStep(
            ApproachPhase.BLOCKED,
            Twist2D(),
            math.inf,
            0.0,
            0.0,
            "Current or goal pose is non-finite",
        )

    dx = goal[0] - current[0]
    dy = goal[1] - current[1]
    distance = math.hypot(dx, dy)
    stance_yaw_error = wrap_angle(goal[2] - current[2])
    if distance > config.max_approach_distance:
        return ApproachStep(
            ApproachPhase.BLOCKED,
            Twist2D(),
            distance,
            0.0,
            stance_yaw_error,
            f"Stance is {distance:.2f} m away; limit is {config.max_approach_distance:.2f} m",
        )

    if distance <= config.position_tolerance:
        if abs(stance_yaw_error) <= config.yaw_tolerance:
            return ApproachStep(
                ApproachPhase.ARRIVED,
                Twist2D(),
                distance,
                0.0,
                stance_yaw_error,
                "Stance position and yaw reached",
            )
        command = Twist2D(
            wz=_usable_speed(
                config.angular_gain * stance_yaw_error,
                config.min_angular,
                config.max_angular,
            )
        )
        return ApproachStep(
            ApproachPhase.TURN_TO_STANCE,
            command,
            distance,
            0.0,
            stance_yaw_error,
            "Turning to final pour yaw",
        )

    path_heading = math.atan2(dy, dx)
    path_heading_error = wrap_angle(path_heading - current[2])
    if abs(path_heading_error) > config.drive_heading_tolerance:
        command = Twist2D(
            wz=_usable_speed(
                config.angular_gain * path_heading_error,
                config.min_angular,
                config.max_angular,
            )
        )
        return ApproachStep(
            ApproachPhase.TURN_TO_PATH,
            command,
            distance,
            path_heading_error,
            stance_yaw_error,
            "Turning toward straight approach path",
        )

    command = Twist2D(
        vx=abs(
            _usable_speed(
                config.linear_gain * distance,
                config.min_linear,
                config.max_linear,
            )
        ),
        wz=_usable_speed(
            config.angular_gain * path_heading_error,
            # A minimum angular speed is useful for deliberate in-place turns,
            # but destabilizes forward walking: any tiny odometry error would
            # otherwise command a persistent curve. Let drive corrections
            # approach zero continuously as the robot aligns to the path.
            0.0,
            config.max_drive_angular,
        ),
    )
    return ApproachStep(
        ApproachPhase.WALK_FORWARD,
        command,
        distance,
        path_heading_error,
        stance_yaw_error,
        "Walking forward on straight approach path",
    )
