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

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.spec.models import CartesianDelta, CartesianPlanningRequest
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState

GROUP = PlanningGroup(
    id="arm/manipulator",
    robot_name="arm",
    group_name="manipulator",
    joint_names=("arm/joint1", "arm/joint2"),
    local_joint_names=("joint1", "joint2"),
    base_link="base",
    tip_link="tcp",
)
SELECTION = PlanningGroupSelection.from_groups((GROUP,))
START = JointState({"name": ["arm/joint1", "arm/joint2"], "position": [0.0, 0.0]})


def test_cartesian_planning_request_carries_absolute_pose_target() -> None:
    target = PoseStamped(frame_id="world", position=[0.1, 0.2, 0.3])

    request = CartesianPlanningRequest(
        selection=SELECTION,
        group_id="arm/manipulator",
        start=START,
        target=target,
        target_mode="absolute",
    )

    assert request.target is target
    assert request.target_mode == "absolute"
    assert request.path_mode == "free"
    assert request.reference_frame == "world"


def test_cartesian_planning_request_carries_relative_delta_target() -> None:
    delta = CartesianDelta(translation=(0.1, 0.0, 0.0), rotation_rpy=(0.0, 0.0, 0.2))

    request = CartesianPlanningRequest(
        selection=SELECTION,
        group_id="arm/manipulator",
        start=START,
        target=delta,
        target_mode="relative",
        path_mode="linear",
    )

    assert request.target is delta
    assert request.target_mode == "relative"
    assert request.path_mode == "linear"


def test_cartesian_models_are_frozen() -> None:
    delta = CartesianDelta()
    request = CartesianPlanningRequest(
        selection=SELECTION,
        group_id="arm/manipulator",
        start=START,
        target=delta,
        target_mode="relative",
    )

    with pytest.raises(FrozenInstanceError):
        delta.frame_id = "tool"  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        request.path_mode = "linear"  # type: ignore[misc]
