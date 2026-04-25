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

"""Per-control-tick telemetry record (P2-1).

Each tick captures reference and measured pose and body-frame velocity, pose
tracking error scalars, the commanded ``cmd_vel``, control period ``dt``, and
optional wall or simulation timestamps. Integration code chooses a
``TrajectoryControlTickSink`` (in-memory list, no-op, or custom export).

Optional environment flag ``DIMOS_TRAJECTORY_CONTROL_TICK_LOG`` (truthy:
``1``, ``true``, ``yes``, ``on``) signals that a runner should allocate a sink
or forward ticks; it does not perform I/O by itself.

JSONL export (stable field names) lives in ``dimos.navigation.trajectory_control_tick_export``
and ``trajectory_control_tick_jsonl.md``.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.navigation.trajectory_metrics import (
    commanded_planar_speed,
    planar_position_divergence,
    pose_errors_vs_reference,
)
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample

DIMOS_TRAJECTORY_CONTROL_TICK_LOG_ENV = "DIMOS_TRAJECTORY_CONTROL_TICK_LOG"


@dataclass(frozen=True)
class TrajectoryControlTick:
    """One closed-loop trajectory control iteration (data only)."""

    # Reference: plan-frame pose and body-frame twist, trajectory time basis
    ref_time_s: float
    ref_x_m: float
    ref_y_m: float
    ref_yaw_rad: float
    ref_twist_linear_x_m_s: float
    ref_twist_linear_y_m_s: float
    ref_twist_linear_z_m_s: float
    ref_twist_angular_x_rad_s: float
    ref_twist_angular_y_rad_s: float
    ref_twist_angular_z_rad_s: float

    # Measured state in the same frames and time basis as ``trajectory_types``
    meas_time_s: float
    meas_x_m: float
    meas_y_m: float
    meas_yaw_rad: float
    meas_twist_linear_x_m_s: float
    meas_twist_linear_y_m_s: float
    meas_twist_linear_z_m_s: float
    meas_twist_angular_x_rad_s: float
    meas_twist_angular_y_rad_s: float
    meas_twist_angular_z_rad_s: float

    # Error scalars (see ``trajectory_metrics``)
    e_along_track_m: float
    e_cross_track_m: float
    e_heading_rad: float
    planar_position_divergence_m: float

    # Command actually published this tick (body frame)
    cmd_linear_x_m_s: float
    cmd_linear_y_m_s: float
    cmd_linear_z_m_s: float
    cmd_angular_x_rad_s: float
    cmd_angular_y_rad_s: float
    cmd_angular_z_rad_s: float
    commanded_planar_speed_m_s: float

    dt_s: float
    wall_time_s: float | None = None
    sim_time_s: float | None = None


def _planar_yaw_rad(pose_plan_pose: object) -> float:
    return float(pose_plan_pose.orientation.euler.z)


def _twist_components(twist: Twist) -> tuple[float, float, float, float, float, float]:
    return (
        float(twist.linear.x),
        float(twist.linear.y),
        float(twist.linear.z),
        float(twist.angular.x),
        float(twist.angular.y),
        float(twist.angular.z),
    )


def trajectory_control_tick_from_samples(
    reference: TrajectoryReferenceSample,
    measurement: TrajectoryMeasuredSample,
    command: Twist,
    dt_s: float,
    *,
    wall_time_s: float | None = None,
    sim_time_s: float | None = None,
) -> TrajectoryControlTick:
    """Build a tick record from trajectory samples and the clamped command."""
    ref_p = reference.pose_plan
    meas_p = measurement.pose_plan
    x_ref, y_ref = float(ref_p.position.x), float(ref_p.position.y)
    yaw_ref = _planar_yaw_rad(ref_p)
    x_m, y_m = float(meas_p.position.x), float(meas_p.position.y)
    yaw_m = _planar_yaw_rad(meas_p)

    e_at, e_ct, e_psi = pose_errors_vs_reference(x_m, y_m, yaw_m, x_ref, y_ref, yaw_ref)
    div = planar_position_divergence(e_at, e_ct)

    rlin = _twist_components(reference.twist_body)
    mlin = _twist_components(measurement.twist_body)
    clin = _twist_components(command)
    v_cmd = commanded_planar_speed(command)

    return TrajectoryControlTick(
        ref_time_s=float(reference.time_s),
        ref_x_m=x_ref,
        ref_y_m=y_ref,
        ref_yaw_rad=yaw_ref,
        ref_twist_linear_x_m_s=rlin[0],
        ref_twist_linear_y_m_s=rlin[1],
        ref_twist_linear_z_m_s=rlin[2],
        ref_twist_angular_x_rad_s=rlin[3],
        ref_twist_angular_y_rad_s=rlin[4],
        ref_twist_angular_z_rad_s=rlin[5],
        meas_time_s=float(measurement.time_s),
        meas_x_m=x_m,
        meas_y_m=y_m,
        meas_yaw_rad=yaw_m,
        meas_twist_linear_x_m_s=mlin[0],
        meas_twist_linear_y_m_s=mlin[1],
        meas_twist_linear_z_m_s=mlin[2],
        meas_twist_angular_x_rad_s=mlin[3],
        meas_twist_angular_y_rad_s=mlin[4],
        meas_twist_angular_z_rad_s=mlin[5],
        e_along_track_m=e_at,
        e_cross_track_m=e_ct,
        e_heading_rad=e_psi,
        planar_position_divergence_m=div,
        cmd_linear_x_m_s=clin[0],
        cmd_linear_y_m_s=clin[1],
        cmd_linear_z_m_s=clin[2],
        cmd_angular_x_rad_s=clin[3],
        cmd_angular_y_rad_s=clin[4],
        cmd_angular_z_rad_s=clin[5],
        commanded_planar_speed_m_s=v_cmd,
        dt_s=float(dt_s),
        wall_time_s=wall_time_s,
        sim_time_s=sim_time_s,
    )


@runtime_checkable
class TrajectoryControlTickSink(Protocol):
    """Consumer of control ticks (in-memory buffer, exporter, metrics, etc.)."""

    def append(self, tick: TrajectoryControlTick) -> None: ...


class NullTrajectoryControlTickSink:
    """No-op sink; safe default when logging is disabled."""

    def append(self, _: TrajectoryControlTick) -> None:
        pass


class ListTrajectoryControlTickSink:
    """In-memory sink for tests and temporary capture."""

    def __init__(self) -> None:
        self.ticks: list[TrajectoryControlTick] = []

    def append(self, tick: TrajectoryControlTick) -> None:
        self.ticks.append(tick)


def trajectory_control_tick_logging_enabled() -> bool:
    """True if ``DIMOS_TRAJECTORY_CONTROL_TICK_LOG`` is set to a truthy token."""
    v = os.environ.get(DIMOS_TRAJECTORY_CONTROL_TICK_LOG_ENV, "").strip().lower()
    return v in ("1", "true", "yes", "on")


def append_trajectory_control_tick(
    sink: TrajectoryControlTickSink | None,
    reference: TrajectoryReferenceSample,
    measurement: TrajectoryMeasuredSample,
    command: Twist,
    dt_s: float,
    *,
    wall_time_s: float | None = None,
    sim_time_s: float | None = None,
) -> TrajectoryControlTick | None:
    """If ``sink`` is None, skip work and return None; else record and return the tick."""
    if sink is None:
        return None
    tick = trajectory_control_tick_from_samples(
        reference,
        measurement,
        command,
        dt_s,
        wall_time_s=wall_time_s,
        sim_time_s=sim_time_s,
    )
    sink.append(tick)
    return tick


__all__ = [
    "DIMOS_TRAJECTORY_CONTROL_TICK_LOG_ENV",
    "ListTrajectoryControlTickSink",
    "NullTrajectoryControlTickSink",
    "TrajectoryControlTick",
    "TrajectoryControlTickSink",
    "append_trajectory_control_tick",
    "trajectory_control_tick_from_samples",
    "trajectory_control_tick_logging_enabled",
]
