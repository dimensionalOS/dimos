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

"""Tests for manipulation trajectory parametrization artifacts and backends."""

from __future__ import annotations

import pytest

from dimos.manipulation.planning.spec.config import TrajectoryParametrizationConfig
from dimos.manipulation.planning.spec.enums import ParametrizationStatus, PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan, GeneratedTrajectory
from dimos.manipulation.planning.trajectory_generator.parametrizers import (
    SimpleTrapezoidParametrizer,
    create_trajectory_parametrizer,
)
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


def _successful_plan() -> GeneratedPlan:
    return GeneratedPlan(
        group_ids=("arm/manipulator",),
        path=[
            JointState({"name": ["arm/joint_b", "arm/joint_a"], "position": [0.0, 1.0]}),
            JointState({"name": ["arm/joint_b", "arm/joint_a"], "position": [0.5, 0.25]}),
            JointState({"name": ["arm/joint_b", "arm/joint_a"], "position": [1.0, -0.5]}),
        ],
        status=PlanningStatus.SUCCESS,
        planning_time=0.25,
        path_length=1.5,
        iterations=7,
        message="planned",
    )


def test_generated_plan_is_geometric_and_generated_trajectory_carries_metadata() -> None:
    plan = _successful_plan()
    trajectory = GeneratedTrajectory(
        joint_names=["arm/joint_b", "arm/joint_a"],
        points=[TrajectoryPoint(time_from_start=0.0, positions=[0.0, 1.0])],
        duration=1.25,
        status=ParametrizationStatus.SUCCESS,
        message="timed",
        source_group_ids=plan.group_ids,
        source_plan_status=plan.status,
        source_plan_message=plan.message,
    )

    assert not hasattr(plan, "duration")
    assert not hasattr(plan, "joint_names")
    assert trajectory.is_success()
    assert trajectory.duration == pytest.approx(1.25)
    assert trajectory.speed_scale == pytest.approx(1.0)
    assert trajectory.source_group_ids == plan.group_ids
    assert trajectory.source_plan_status == PlanningStatus.SUCCESS
    assert trajectory.source_plan_message == "planned"


def test_unsuccessful_plan_parametrization_fails_without_changing_plan_status() -> None:
    plan = GeneratedPlan(
        group_ids=("arm/manipulator",),
        status=PlanningStatus.TIMEOUT,
        message="planner timed out",
    )

    trajectory = SimpleTrapezoidParametrizer(TrajectoryParametrizationConfig()).parametrize(plan)

    assert plan.status == PlanningStatus.TIMEOUT
    assert trajectory.status == ParametrizationStatus.INVALID_PLAN
    assert trajectory.source_plan_status == PlanningStatus.TIMEOUT
    assert trajectory.source_plan_message == "planner timed out"
    assert "not successful" in trajectory.message


def test_simple_trapezoid_preserves_joint_ordering_and_shared_monotonic_time_domain() -> None:
    plan = _successful_plan()
    trajectory = SimpleTrapezoidParametrizer(
        TrajectoryParametrizationConfig(simple_points_per_segment=4)
    ).parametrize(plan)

    assert trajectory.status == ParametrizationStatus.SUCCESS
    assert trajectory.joint_names == ["arm/joint_b", "arm/joint_a"]
    times = [point.time_from_start for point in trajectory.points]
    assert times == sorted(times)
    assert len(set(times)) == len(trajectory.points)
    assert trajectory.duration == pytest.approx(times[-1])
    assert trajectory.points[-1].positions == pytest.approx([1.0, -0.5])
    assert all(len(point.positions) == len(trajectory.joint_names) for point in trajectory.points)


def test_simple_trapezoid_speed_scale_slows_timing_when_reduced() -> None:
    plan = _successful_plan()
    parametrizer = SimpleTrapezoidParametrizer(TrajectoryParametrizationConfig())

    default_trajectory = parametrizer.parametrize(plan)
    slowed_trajectory = parametrizer.parametrize(plan, speed_scale=0.5)

    assert default_trajectory.status == ParametrizationStatus.SUCCESS
    assert slowed_trajectory.status == ParametrizationStatus.SUCCESS
    assert default_trajectory.speed_scale == pytest.approx(1.0)
    assert slowed_trajectory.speed_scale == pytest.approx(0.5)
    assert slowed_trajectory.duration > default_trajectory.duration
    assert slowed_trajectory.points[-1].positions == pytest.approx(
        default_trajectory.points[-1].positions
    )


def test_parametrizer_factory_defaults_to_simple_trapezoid() -> None:
    parametrizer = create_trajectory_parametrizer()

    assert isinstance(parametrizer, SimpleTrapezoidParametrizer)
    assert parametrizer.get_name() == "simple_trapezoid"


@pytest.mark.parametrize(
    "kwargs",
    [
        {"velocity_scale": 0.0},
        {"acceleration_scale": 0.0},
        {"minimum_segment_duration": 0.0},
        {"simple_points_per_segment": 0},
        {"roboplan_dt": 0.0},
        {"roboplan_max_adaptive_iterations": 0},
        {"roboplan_max_adaptive_step_size": 0.0},
        {"roboplan_max_blend_deviation": 0.0},
    ],
)
def test_trajectory_parametrization_config_rejects_non_positive_controls(
    kwargs: dict[str, float | int],
) -> None:
    with pytest.raises(ValueError):
        TrajectoryParametrizationConfig(**kwargs)


def test_roboplan_factory_reports_world_owned_backend() -> None:
    with pytest.raises(ValueError, match='world_backend="roboplan"'):
        create_trajectory_parametrizer(TrajectoryParametrizationConfig(backend="roboplan"))
