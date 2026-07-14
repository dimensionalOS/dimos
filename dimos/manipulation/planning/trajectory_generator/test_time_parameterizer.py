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

"""Tests for the formal time-parameterization stage."""

import pytest

from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import PlanningResult
from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
    JointTrajectoryGenerator,
)
from dimos.manipulation.planning.trajectory_generator.time_parameterizer import (
    TrapezoidalTimeParameterizer,
)
from dimos.msgs.sensor_msgs.JointState import JointState

NAMES = ["joint_a", "joint_b"]


def _path() -> list[JointState]:
    return [
        JointState(name=NAMES, position=[0.0, 0.0]),
        JointState(name=NAMES, position=[0.5, 0.3]),
        JointState(name=NAMES, position=[1.0, 0.6]),
    ]


def _stage() -> tuple[TrapezoidalTimeParameterizer, JointTrajectoryGenerator]:
    gen = JointTrajectoryGenerator(num_joints=2, max_velocity=1.0, max_acceleration=2.0)
    return TrapezoidalTimeParameterizer(gen), gen


def test_untimed_path_falls_back_to_trapezoidal() -> None:
    """An untimed planner (no timestamps) gets the trapezoidal profile -- unchanged behavior."""
    stage, gen = _stage()
    result = PlanningResult(status=PlanningStatus.SUCCESS, path=_path(), timestamps=None)

    traj = stage.parameterize(result)
    expected = gen.generate([list(s.position) for s in result.path])

    assert traj.duration == pytest.approx(expected.duration)
    assert traj.duration > 0.0


def test_planner_timestamps_are_preserved() -> None:
    """Planner-provided timing is honored exactly, not re-synthesized (the bug fix)."""
    stage, _ = _stage()
    timestamps = [0.0, 3.0, 6.0]
    result = PlanningResult(status=PlanningStatus.SUCCESS, path=_path(), timestamps=timestamps)

    traj = stage.parameterize(result)

    assert traj.duration == pytest.approx(6.0)
    assert [p.time_from_start for p in traj.points] == pytest.approx(timestamps)
    assert traj.points[0].positions == [0.0, 0.0]
    assert traj.points[-1].positions == [1.0, 0.6]


def test_timestamp_path_length_mismatch_raises() -> None:
    """A timestamp/waypoint count mismatch is a hard error, not a silent bug."""
    stage, _ = _stage()
    result = PlanningResult(
        status=PlanningStatus.SUCCESS, path=_path(), timestamps=[0.0, 1.0]  # 2 vs 3 waypoints
    )
    with pytest.raises(ValueError):
        stage.parameterize(result)


def test_empty_timestamps_for_nonempty_path_raises() -> None:
    """An empty timestamp list for a non-empty path is a mismatch, not 'untimed'."""
    stage, _ = _stage()
    result = PlanningResult(status=PlanningStatus.SUCCESS, path=_path(), timestamps=[])
    with pytest.raises(ValueError):
        stage.parameterize(result)


def test_non_monotonic_timestamps_raise() -> None:
    """A zero-duration or out-of-order timeline is rejected up front."""
    stage, _ = _stage()
    for bad in ([0.0, 0.0, 0.0], [0.0, 2.0, 1.0]):
        result = PlanningResult(status=PlanningStatus.SUCCESS, path=_path(), timestamps=bad)
        with pytest.raises(ValueError):
            stage.parameterize(result)


def test_negative_timestamps_raise() -> None:
    """A negative timeline is rejected."""
    stage, _ = _stage()
    result = PlanningResult(status=PlanningStatus.SUCCESS, path=_path(), timestamps=[-1.0, 1.0, 2.0])
    with pytest.raises(ValueError):
        stage.parameterize(result)


def test_non_finite_timestamps_raise() -> None:
    """NaN or infinite timestamps are rejected (they would break the controller)."""
    stage, _ = _stage()
    for bad in ([0.0, float("inf"), 2.0], [0.0, float("nan"), 2.0]):
        result = PlanningResult(status=PlanningStatus.SUCCESS, path=_path(), timestamps=bad)
        with pytest.raises(ValueError):
            stage.parameterize(result)
