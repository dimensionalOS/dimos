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

"""Unit tests for planning group joint-name normalization."""

from __future__ import annotations

import pytest

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.groups.utils import joint_target_to_global_names
from dimos.msgs.sensor_msgs.JointState import JointState


def _make_group() -> PlanningGroup:
    return PlanningGroup(
        id="left/arm",
        robot_name="left",
        group_name="arm",
        joint_names=("left/j1", "left/j2", "left/j3"),
        local_joint_names=("j1", "j2", "j3"),
        base_link="base",
        tip_link="ee",
    )


def test_joint_target_to_global_names_accepts_named_global_targets_in_group_order() -> None:
    group = _make_group()
    target = JointState(name=["left/j3", "left/j1", "left/j2"], position=[3.0, 1.0, 2.0])

    normalized = joint_target_to_global_names(group, target)

    assert normalized.name == ["left/j1", "left/j2", "left/j3"]
    assert normalized.position == [1.0, 2.0, 3.0]


def test_joint_target_to_global_names_accepts_named_local_targets_in_group_order() -> None:
    group = _make_group()
    target = JointState(name=["j2", "j3", "j1"], position=[2.0, 3.0, 1.0])

    normalized = joint_target_to_global_names(group, target)

    assert normalized.name == ["left/j1", "left/j2", "left/j3"]
    assert normalized.position == [1.0, 2.0, 3.0]


def test_joint_target_to_global_names_rejects_mixed_global_and_local_target_names() -> None:
    group = _make_group()
    target = JointState(name=["left/j1", "j2", "left/j3"], position=[1.0, 2.0, 3.0])

    with pytest.raises(ValueError, match="mixes global and local joint names"):
        joint_target_to_global_names(group, target)
