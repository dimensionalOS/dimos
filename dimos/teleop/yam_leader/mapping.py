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

"""Map YAM leader readings onto dual OpenYAM follower commands."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math
from typing import Literal

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection

YamLeaderSide = Literal["left", "right"]
YAM_LEADER_SIDES: tuple[YamLeaderSide, ...] = ("left", "right")

# Six arm servos then the gripper trigger, IDs 1-7, named joint_1..joint_7 in the calibration.
YAM_LEADER_ARM_MOTORS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
YAM_LEADER_GRIPPER_MOTOR = "joint_7"
YAM_LEADER_MOTORS = (*YAM_LEADER_ARM_MOTORS, YAM_LEADER_GRIPPER_MOTOR)

# dual_openyam.urdf joint limits; sender-side clamp, the coordinator still enforces its own.
YAM_FOLLOWER_JOINT_LIMITS: tuple[tuple[float, float], ...] = (
    (-2.618, 3.1416),
    (0.0, 3.6652),
    (0.0, 3.1416),
    (-1.693, 1.5708),
    (-1.5708, 1.5708),
    (-2.0944, 2.0944),
)

GRIPPER_OPEN = 1.0
GRIPPER_CLOSED = 0.0

_FOLLOWER_GROUPS = {
    group.name: tuple(group.joint_names)
    for group in DualOpenYamConnection.planning_model("arm").planning_groups
}


def yam_follower_joint_names(side: YamLeaderSide) -> tuple[str, ...]:
    """Follower arm joints driven by a leader side, in leader motor order."""
    joints = _FOLLOWER_GROUPS.get(f"{side}_manipulator")
    if joints is None:
        raise ValueError(f"unknown YAM leader side {side!r}")
    return joints


@dataclass(frozen=True)
class YamLeaderSideCommand:
    """Mapped command for one dual OpenYAM follower side."""

    side: YamLeaderSide
    positions_by_joint: dict[str, float]
    gripper_opening: float


def map_side_readings(
    side: YamLeaderSide,
    readings: Mapping[str, float],
    *,
    gripper_travel_radians: float,
    previous_positions_by_joint: Mapping[str, float] | None = None,
    max_joint_jump_radians: float | None = None,
) -> YamLeaderSideCommand:
    """Calibrated leader radians to follower joints and a gripper opening; trigger 0 rad = released."""
    missing = set(YAM_LEADER_MOTORS) - set(readings)
    if missing:
        raise ValueError(f"YAM leader readings missing motors: {sorted(missing)}")
    if gripper_travel_radians <= 0.0:
        raise ValueError("gripper_travel_radians must be positive")
    follower_joints = yam_follower_joint_names(side)
    positions_by_joint = {
        joint: _clamp(float(readings[motor]), *limits)
        for joint, motor, limits in zip(
            follower_joints, YAM_LEADER_ARM_MOTORS, YAM_FOLLOWER_JOINT_LIMITS, strict=True
        )
    }
    _validate_jump(positions_by_joint, previous_positions_by_joint, max_joint_jump_radians)
    squeeze = _clamp(
        abs(float(readings[YAM_LEADER_GRIPPER_MOTOR])) / gripper_travel_radians, 0.0, 1.0
    )
    return YamLeaderSideCommand(
        side=side,
        positions_by_joint=positions_by_joint,
        gripper_opening=GRIPPER_OPEN - squeeze * (GRIPPER_OPEN - GRIPPER_CLOSED),
    )


def combine_side_commands(commands: list[YamLeaderSideCommand]) -> JointState:
    """Combine side commands into one coordinator-facing JointState."""
    names: list[str] = []
    positions: list[float] = []
    for command in commands:
        names.extend(command.positions_by_joint)
        positions.extend(command.positions_by_joint.values())
    return JointState(name=names, position=positions)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _validate_jump(
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
                f"YAM leader {joint} jump {jump:.3f} rad exceeds {max_joint_jump_radians:.3f} rad"
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


def streamed_trajectory(
    last_sent: Mapping[str, float] | None,
    positions: Mapping[str, float],
    horizon_s: float,
) -> JointTrajectory:
    """Two points from the last streamed target, or one point from the measured position."""
    if not (math.isfinite(horizon_s) and horizon_s > 0.0):
        raise ValueError("horizon_s must be positive")
    names = list(positions)
    target = [float(positions[name]) for name in names]
    if last_sent is None or any(name not in last_sent for name in names):
        return JointTrajectory(joint_names=names, points=[TrajectoryPoint(positions=target)])
    start = [float(last_sent[name]) for name in names]
    return JointTrajectory(
        joint_names=names,
        points=[
            TrajectoryPoint(time_from_start=0.0, positions=start),
            TrajectoryPoint(time_from_start=horizon_s, positions=target),
        ],
    )
