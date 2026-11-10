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

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.openarm.config import openarm_urdf_joints
from dimos.teleop.openarm_mini.calibration import (
    OPENARM_MINI_ARM_JOINT_NAMES,
    OPENARM_MINI_GRIPPER_MOTOR_NAME,
    OpenArmMiniSide,
)

LEADER_JOINT_NAMES = OPENARM_MINI_ARM_JOINT_NAMES
LEADER_MOTOR_NAMES = LEADER_JOINT_NAMES
# Normalized gripper opening the gripper task consumes; a released trigger is open.
GRIPPER_OPEN = 1.0
GRIPPER_CLOSED = 0.0

# Inside the OpenArm URDF limits, which the connection enforces by refusing
# the whole command; clamping here keeps a leader at its stop from stalling
# every joint. Joint 4 (elbow) has no negative range.
OPENARM_FOLLOWER_JOINT_LIMITS: dict[OpenArmMiniSide, tuple[tuple[float, float], ...]] = {
    "left": (
        (-3.45, 1.35),
        (-3.30, 0.15),
        (-1.50, 1.50),
        (0.0, 2.40),
        (-1.50, 1.50),
        (-0.75, 0.75),
        (-1.50, 1.50),
    ),
    "right": (
        (-1.35, 3.45),
        (-0.15, 3.30),
        (-1.50, 1.50),
        (0.0, 2.40),
        (-1.50, 1.50),
        (-0.75, 0.75),
        (-1.50, 1.50),
    ),
}


@dataclass(frozen=True)
class OpenArmMiniSideCommand:
    """Mapped command for one OpenArm follower side."""

    side: OpenArmMiniSide
    positions_by_joint: dict[str, float]
    # None when the leader has no calibrated gripper trigger.
    gripper_opening: float | None = None


def map_side_readings(
    side: OpenArmMiniSide,
    readings: dict[str, float],
    *,
    target_joint_names: Sequence[str] | None = None,
    previous_positions_by_joint: dict[str, float] | None = None,
    max_joint_jump_radians: float | None = None,
    gripper_travel_radians: float = 1.0,
) -> OpenArmMiniSideCommand:
    """Map calibrated leader radians into follower joint positions and a gripper opening."""
    _validate_readings(readings)
    if gripper_travel_radians <= 0.0:
        raise ValueError("gripper_travel_radians must be positive")

    follower_joint_names = tuple(target_joint_names or openarm_urdf_joints(side))
    if len(follower_joint_names) != len(LEADER_JOINT_NAMES):
        raise ValueError(
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
    gripper_opening = None
    trigger = readings.get(OPENARM_MINI_GRIPPER_MOTOR_NAME)
    if trigger is not None:
        squeeze = _clamp(abs(trigger) / gripper_travel_radians, 0.0, 1.0)
        gripper_opening = GRIPPER_OPEN - squeeze * (GRIPPER_OPEN - GRIPPER_CLOSED)
    return OpenArmMiniSideCommand(
        side=side,
        positions_by_joint=positions_by_joint,
        gripper_opening=gripper_opening,
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
        raise ValueError(f"OpenArm Mini readings missing arm joints: {sorted(missing)}")


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
            raise ValueError(
                f"Mapped OpenArm Mini {joint_name} jump {jump:.3f} rad exceeds "
                f"threshold {max_joint_jump_radians:.3f} rad"
            )


def low_pass_alpha(cutoff_hz: float, period_s: float) -> float:
    """Blend weight of a first-order low-pass sampled every period_s."""
    if not (math.isfinite(cutoff_hz) and cutoff_hz > 0.0):
        raise ValueError("cutoff_hz must be positive")
    if not (math.isfinite(period_s) and period_s > 0.0):
        raise ValueError("period_s must be positive")
    tau = 1.0 / (2.0 * math.pi * cutoff_hz)
    return period_s / (tau + period_s)


def smooth_positions(
    previous: Mapping[str, float] | None,
    positions: Mapping[str, float],
    alpha: float,
) -> dict[str, float]:
    """Move each joint a fraction alpha toward its new value; unseen joints pass through."""
    if not 0.0 < alpha <= 1.0:
        raise ValueError("alpha must be in (0, 1]")
    if previous is None:
        return {joint: float(value) for joint, value in positions.items()}
    return {
        joint: (
            previous[joint] + alpha * (float(value) - previous[joint])
            if joint in previous
            else float(value)
        )
        for joint, value in positions.items()
    }
