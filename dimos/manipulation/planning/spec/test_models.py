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

"""Tests for planning model contracts."""

from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import (
    CartesianDelta,
    GeneratedPlan,
    LinearTcpPathConstraint,
    PlanningResult,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def test_cartesian_delta_defaults_to_world_identity_delta() -> None:
    delta = CartesianDelta()

    assert delta.translation == (0.0, 0.0, 0.0)
    assert delta.rotation_rpy == (0.0, 0.0, 0.0)
    assert delta.frame_id == "world"


def test_cartesian_delta_carries_relative_target_values() -> None:
    delta = CartesianDelta(translation=(0.1, 0.0, 0.0), rotation_rpy=(0.0, 0.0, 0.2))

    assert delta.translation == (0.1, 0.0, 0.0)
    assert delta.rotation_rpy == (0.0, 0.0, 0.2)
    assert delta.frame_id == "world"


def test_cartesian_delta_is_frozen() -> None:
    delta = CartesianDelta()

    with pytest.raises(FrozenInstanceError):
        delta.frame_id = "tool"  # type: ignore[misc]


def test_planning_result_and_generated_plan_default_to_no_path_constraints() -> None:
    result = PlanningResult(status=PlanningStatus.SUCCESS)
    plan = GeneratedPlan(group_ids=("arm/manipulator",), status=PlanningStatus.SUCCESS)

    assert result.path_constraints is None
    assert plan.path_constraints is None


def test_linear_tcp_path_constraint_fields_and_frozen_behavior() -> None:
    start = PoseStamped(frame_id="world", position=Vector3(0.0, 0.0, 0.0))  # type: ignore[call-arg]
    target = PoseStamped(frame_id="world", position=Vector3(0.1, 0.0, 0.0))  # type: ignore[call-arg]

    constraint = LinearTcpPathConstraint(
        group_id="arm/manipulator",
        tcp_frame="tcp",
        start_pose=start,
        target_pose=target,
        max_translational_deviation=0.002,
        max_rotational_deviation=0.003,
    )

    assert constraint.kind == "linear_tcp"
    assert constraint.group_id == "arm/manipulator"
    assert constraint.tcp_frame == "tcp"
    assert constraint.start_pose is start
    assert constraint.target_pose is target
    assert constraint.max_translational_deviation == pytest.approx(0.002)
    assert constraint.max_rotational_deviation == pytest.approx(0.003)
    with pytest.raises(FrozenInstanceError):
        constraint.tcp_frame = "other_tcp"  # type: ignore[misc]
