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

"""Leader motor readings to follower joint targets, described by config, not code."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math
import sys

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

from pydantic import Field, model_validator

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.service.spec import BaseConfig
from dimos.teleop.leader.calibration import LeaderSide

GRIPPER_OPEN = 1.0
GRIPPER_CLOSED = 0.0


class JointMapping(BaseConfig):
    """One leader motor driving one follower joint: target = scale * reading + offset, clamped."""

    motor: str
    joint: str
    lower: float
    upper: float
    scale: float = 1.0
    offset: float = 0.0

    @model_validator(mode="after")
    def _validate_limits(self) -> Self:
        if not self.lower < self.upper:
            raise ValueError(f"{self.joint}: lower {self.lower} must be below upper {self.upper}")
        if self.scale == 0.0:
            raise ValueError(f"{self.joint}: scale must not be zero")
        return self

    def target(self, reading: float) -> float:
        return _clamp(self.scale * reading + self.offset, self.lower, self.upper)


class GripperMapping(BaseConfig):
    """The trigger motor squeezed through travel_radians closes the gripper from open to closed."""

    motor: str
    travel_radians: float = Field(gt=0.0)

    def opening(self, reading: float) -> float:
        squeeze = _clamp(abs(reading) / self.travel_radians, 0.0, 1.0)
        return GRIPPER_OPEN - squeeze * (GRIPPER_OPEN - GRIPPER_CLOSED)


class SideMapping(BaseConfig):
    """Everything one leader side drives on the follower."""

    joints: tuple[JointMapping, ...] = Field(min_length=1)
    gripper: GripperMapping | None = None

    @model_validator(mode="after")
    def _validate_names(self) -> Self:
        motors = [mapping.motor for mapping in self.joints]
        if self.gripper is not None:
            motors.append(self.gripper.motor)
        if len(set(motors)) != len(motors):
            raise ValueError(f"a leader motor is mapped twice: {motors}")
        joints = [mapping.joint for mapping in self.joints]
        if len(set(joints)) != len(joints):
            raise ValueError(f"a follower joint is mapped twice: {joints}")
        return self

    def motor_names(self) -> tuple[str, ...]:
        """The leader motors this side reads, joints first, then the gripper trigger."""
        names = [mapping.motor for mapping in self.joints]
        if self.gripper is not None:
            names.append(self.gripper.motor)
        return tuple(names)

    def joint_names(self) -> tuple[str, ...]:
        return tuple(mapping.joint for mapping in self.joints)


@dataclass(frozen=True)
class SideCommand:
    """Mapped targets for one follower side."""

    side: LeaderSide
    positions_by_joint: dict[str, float]
    gripper_opening: float | None


def map_side_readings(
    side: LeaderSide,
    mapping: SideMapping,
    readings: Mapping[str, float],
    *,
    previous_positions_by_joint: Mapping[str, float] | None = None,
    max_joint_jump_radians: float | None = None,
) -> SideCommand:
    """Calibrated leader radians to clamped follower targets and a gripper opening."""
    missing = set(mapping.motor_names()) - set(readings)
    if missing:
        raise ValueError(f"{side} leader readings missing motors: {sorted(missing)}")
    positions_by_joint = {
        joint_mapping.joint: joint_mapping.target(float(readings[joint_mapping.motor]))
        for joint_mapping in mapping.joints
    }
    _validate_jump(side, positions_by_joint, previous_positions_by_joint, max_joint_jump_radians)
    gripper_opening = (
        None
        if mapping.gripper is None
        else mapping.gripper.opening(float(readings[mapping.gripper.motor]))
    )
    return SideCommand(side, positions_by_joint, gripper_opening)


def combine_side_commands(commands: list[SideCommand], frame_id: str = "") -> JointState:
    """Every side's follower targets as one JointState."""
    names: list[str] = []
    positions: list[float] = []
    for command in commands:
        names.extend(command.positions_by_joint)
        positions.extend(command.positions_by_joint.values())
    return JointState(name=names, position=positions, frame_id=frame_id)


def low_pass_alpha(cutoff_hz: float, period_s: float) -> float:
    """Blend weight of a first-order low-pass sampled every period_s."""
    if not (math.isfinite(cutoff_hz) and cutoff_hz > 0.0):
        raise ValueError("cutoff_hz must be positive")
    if not (math.isfinite(period_s) and period_s > 0.0):
        raise ValueError("period_s must be positive")
    tau = 1.0 / (2.0 * math.pi * cutoff_hz)
    return period_s / (tau + period_s)


def smooth_positions(
    previous: Mapping[str, float] | None, positions: Mapping[str, float], alpha: float
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


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _validate_jump(
    side: LeaderSide,
    positions_by_joint: Mapping[str, float],
    previous_positions_by_joint: Mapping[str, float] | None,
    max_joint_jump_radians: float | None,
) -> None:
    if previous_positions_by_joint is None or max_joint_jump_radians is None:
        return
    for joint, position in positions_by_joint.items():
        previous = previous_positions_by_joint.get(joint)
        if previous is None:
            continue
        jump = abs(position - previous)
        if jump > max_joint_jump_radians:
            raise ValueError(
                f"{side} leader {joint} jump {jump:.3f} rad exceeds {max_joint_jump_radians:.3f} rad"
            )
