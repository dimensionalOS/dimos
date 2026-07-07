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

"""OpenArm Mini leader to OpenArm follower joint mapping."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.openarm.config import openarm_joints
from dimos.teleop.openarm_mini.calibration import OPENARM_MINI_ARM_JOINT_NAMES
from dimos.teleop.openarm_mini.config import validate_side

LEADER_JOINT_NAMES = OPENARM_MINI_ARM_JOINT_NAMES
LEADER_MOTOR_NAMES = LEADER_JOINT_NAMES

# Mirrors the measured OpenArm v1.0 follower limits from
# dimos.hardware.manipulators.openarm.adapter. The sender-side clamp improves
# teleop behavior; the follower/control stack remains defensive.
OPENARM_FOLLOWER_JOINT_LIMITS: dict[str, tuple[tuple[float, float], ...]] = {
    "left": (
        (-3.45, 1.35),
        (-3.30, 0.15),
        (-1.50, 1.50),
        (-0.01, 2.40),
        (-1.50, 1.50),
        (-0.75, 0.75),
        (-1.50, 1.50),
    ),
    "right": (
        (-1.35, 3.45),
        (-0.15, 3.30),
        (-1.50, 1.50),
        (-0.01, 2.40),
        (-1.50, 1.50),
        (-0.75, 0.75),
        (-1.50, 1.50),
    ),
}


class OpenArmMiniMappingError(ValueError):
    """Raised when OpenArm Mini leader readings cannot be mapped safely."""


@dataclass(frozen=True)
class OpenArmMiniSideCommand:
    """Mapped command for one OpenArm follower side."""

    side: str
    positions_by_joint: dict[str, float]


def map_side_readings(
    side: str,
    readings: dict[str, float],
    *,
    target_joint_names: Sequence[str] | None = None,
    previous_positions_by_joint: dict[str, float] | None = None,
    max_joint_jump_radians: float | None = None,
) -> OpenArmMiniSideCommand:
    """Map calibrated leader arm radians into OpenArm follower joint positions."""
    validate_side(side)
    _validate_readings(readings)

    follower_joint_names = tuple(target_joint_names or openarm_joints(side))
    if len(follower_joint_names) != len(LEADER_JOINT_NAMES):
        raise OpenArmMiniMappingError(
            f"target_joint_names must contain {len(LEADER_JOINT_NAMES)} names, "
            f"got {len(follower_joint_names)}"
        )
    side_limits = OPENARM_FOLLOWER_JOINT_LIMITS[side]
    positions_by_joint = {
        follower_joint: _clamp(readings[f"joint_{index}"], *side_limits[index - 1])
        for index, follower_joint in enumerate(follower_joint_names, start=1)
    }
    _validate_jump_threshold(
        positions_by_joint,
        previous_positions_by_joint,
        max_joint_jump_radians,
    )
    return OpenArmMiniSideCommand(
        side=side,
        positions_by_joint=positions_by_joint,
    )


def combine_side_commands(commands: list[OpenArmMiniSideCommand]) -> JointState:
    """Combine side commands into a coordinator-facing OpenArm JointState."""
    names: list[str] = []
    positions: list[float] = []
    for command in commands:
        for name, position in command.positions_by_joint.items():
            names.append(name)
            positions.append(position)
    return JointState({"name": names, "position": positions})


def _validate_readings(readings: dict[str, float]) -> None:
    missing = set(LEADER_MOTOR_NAMES) - set(readings)
    if missing:
        raise OpenArmMiniMappingError(
            f"OpenArm Mini readings missing arm joints: {sorted(missing)}"
        )


def _clamp(position: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, position))


def _validate_jump_threshold(
    positions_by_joint: dict[str, float],
    previous_positions_by_joint: dict[str, float] | None,
    max_joint_jump_radians: float | None,
) -> None:
    if previous_positions_by_joint is None or max_joint_jump_radians is None:
        return
    for joint_name, position in positions_by_joint.items():
        previous_position = previous_positions_by_joint.get(joint_name)
        if previous_position is None:
            continue
        jump = abs(position - previous_position)
        if jump > max_joint_jump_radians:
            raise OpenArmMiniMappingError(
                f"Mapped OpenArm Mini {joint_name} jump {jump:.3f} rad exceeds "
                f"threshold {max_joint_jump_radians:.3f} rad"
            )
