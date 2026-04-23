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

"""Tests for ``trajectory_control_tick_log`` (P2-1)."""

import math

import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_control_tick_log import (
    DIMOS_TRAJECTORY_CONTROL_TICK_LOG_ENV,
    ListTrajectoryControlTickSink,
    NullTrajectoryControlTickSink,
    append_trajectory_control_tick,
    trajectory_control_tick_from_samples,
    trajectory_control_tick_logging_enabled,
)
from dimos.navigation.trajectory_metrics import planar_position_divergence, pose_errors_vs_reference
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample


def _sample_pair(
    *,
    ref_xyyaw: tuple[float, float, float] = (0.0, 0.0, 0.0),
    meas_xyyaw: tuple[float, float, float] = (1.0, 0.0, 0.0),
) -> tuple[TrajectoryReferenceSample, TrajectoryMeasuredSample]:
    rx, ry, ryaw = ref_xyyaw
    mx, my, myaw = meas_xyyaw
    ref = TrajectoryReferenceSample(
        time_s=0.1,
        pose_plan=Pose(rx, ry, 0.0, 0.0, 0.0, math.sin(ryaw / 2), math.cos(ryaw / 2)),
        twist_body=Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)),
    )
    meas = TrajectoryMeasuredSample(
        time_s=0.11,
        pose_plan=Pose(mx, my, 0.0, 0.0, 0.0, math.sin(myaw / 2), math.cos(myaw / 2)),
        twist_body=Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)),
    )
    return ref, meas


def test_tick_matches_pose_errors_and_divergence() -> None:
    ref, meas = _sample_pair()
    cmd = Twist(linear=Vector3(0.3, 0.4, 0.0), angular=Vector3(0.0, 0.0, 0.1))
    dt = 0.02
    tick = trajectory_control_tick_from_samples(
        ref,
        meas,
        cmd,
        dt,
        wall_time_s=1000.0,
        sim_time_s=1.25,
    )
    e_at, e_ct, e_psi = pose_errors_vs_reference(
        tick.meas_x_m,
        tick.meas_y_m,
        tick.meas_yaw_rad,
        tick.ref_x_m,
        tick.ref_y_m,
        tick.ref_yaw_rad,
    )
    assert tick.e_along_track_m == pytest.approx(e_at)
    assert tick.e_cross_track_m == pytest.approx(e_ct)
    assert tick.e_heading_rad == pytest.approx(e_psi)
    assert tick.planar_position_divergence_m == pytest.approx(planar_position_divergence(e_at, e_ct))
    assert tick.commanded_planar_speed_m_s == pytest.approx(0.5)
    assert tick.dt_s == pytest.approx(dt)
    assert tick.wall_time_s == pytest.approx(1000.0)
    assert tick.sim_time_s == pytest.approx(1.25)


def test_list_sink_collects_ticks() -> None:
    ref, meas = _sample_pair()
    cmd = Twist()
    sink = ListTrajectoryControlTickSink()
    tick = trajectory_control_tick_from_samples(ref, meas, cmd, 0.05)
    sink.append(tick)
    assert len(sink.ticks) == 1
    assert sink.ticks[0] is tick


def test_null_sink_is_safe() -> None:
    sink = NullTrajectoryControlTickSink()
    ref, meas = _sample_pair()
    sink.append(trajectory_control_tick_from_samples(ref, meas, Twist(), 0.05))


def test_append_with_none_sink_returns_none() -> None:
    ref, meas = _sample_pair()
    assert (
        append_trajectory_control_tick(None, ref, meas, Twist(), 0.05, wall_time_s=1.0)
        is None
    )


def test_append_records_tick() -> None:
    ref, meas = _sample_pair()
    sink = ListTrajectoryControlTickSink()
    out = append_trajectory_control_tick(sink, ref, meas, Twist(), 0.05, sim_time_s=3.0)
    assert out is not None
    assert len(sink.ticks) == 1
    assert sink.ticks[0].sim_time_s == pytest.approx(3.0)


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        ("1", True),
        ("true", True),
        ("yes", True),
        ("on", True),
        ("TRUE", True),
        ("", False),
        ("0", False),
        ("no", False),
    ],
)
def test_logging_enabled_env(value: str, expected: bool, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(DIMOS_TRAJECTORY_CONTROL_TICK_LOG_ENV, raising=False)
    if value != "":
        monkeypatch.setenv(DIMOS_TRAJECTORY_CONTROL_TICK_LOG_ENV, value)
    assert trajectory_control_tick_logging_enabled() is expected


def test_trajectory_control_tick_is_frozen() -> None:
    ref, meas = _sample_pair()
    tick = trajectory_control_tick_from_samples(ref, meas, Twist(), 0.05)
    with pytest.raises(AttributeError):
        tick.dt_s = 1.0  # type: ignore[misc]
