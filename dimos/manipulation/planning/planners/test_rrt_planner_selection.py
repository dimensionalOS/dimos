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

"""Focused tests for selected-joint RRT planning."""

from __future__ import annotations

from contextlib import nullcontext
from pathlib import Path

import numpy as np
import pytest

from dimos.manipulation.planning.groups.models import (
    PlanningGroup,
    PlanningGroupDefinition,
    PlanningGroupSelection,
)
from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState


def _pose() -> PoseStamped:
    return PoseStamped(position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))


def _group(name: str, joints: tuple[str, ...]) -> PlanningGroup:
    return PlanningGroup(
        id=f"arm/{name}",
        robot_name="arm",
        group_name=name,
        joint_names=tuple(f"arm/{joint}" for joint in joints),
        local_joint_names=joints,
        base_link="base",
        tip_link="tool",
    )


class _World:
    is_finalized = True

    def __init__(self, current: list[float] | None = None) -> None:
        self.current = current or [0.0, 0.0, 0.7]
        self.projected_states: list[JointState] = []
        self.config = RobotModelConfig(
            name="arm",
            model_path=Path("robot.urdf"),
            base_pose=_pose(),
            joint_names=["joint_a", "joint_b", "gripper"],
            base_link="base",
            planning_groups=[
                PlanningGroupDefinition("arm", ("joint_a", "joint_b"), "base", "tool")
            ],
        )

    def get_robot_ids(self) -> list[str]:
        return ["robot"]

    def get_robot_config(self, robot_id: str) -> RobotModelConfig:
        return self.config

    def scratch_context(self) -> nullcontext[None]:
        return nullcontext(None)

    def get_joint_state(self, ctx: object, robot_id: str) -> JointState:
        return JointState({"name": ["joint_a", "joint_b", "gripper"], "position": self.current})

    def get_joint_limits(self, robot_id: str) -> tuple[np.ndarray, np.ndarray]:
        return np.array([-1.0, -1.0, -1.0]), np.array([1.0, 1.0, 1.0])

    def set_joint_state(self, ctx: object, robot_id: str, joint_state: JointState) -> None:
        self.projected_states.append(joint_state)

    def is_collision_free(self, ctx: object, robot_id: str) -> bool:
        return True


@pytest.mark.parametrize(
    ("start", "goal", "expected_start", "expected_goal"),
    [
        (
            JointState({"position": [0.1, 0.2]}),
            JointState({"position": [0.3, 0.4]}),
            [0.1, 0.2],
            [0.3, 0.4],
        ),
        (
            JointState({"name": ["arm/joint_b", "arm/joint_a"], "position": [0.2, 0.1]}),
            JointState({"name": ["arm/joint_b", "arm/joint_a"], "position": [0.4, 0.3]}),
            [0.1, 0.2],
            [0.3, 0.4],
        ),
        (
            JointState({"name": ["joint_b", "joint_a"], "position": [0.2, 0.1]}),
            JointState({"name": ["joint_b", "joint_a"], "position": [0.4, 0.3]}),
            [0.1, 0.2],
            [0.3, 0.4],
        ),
    ],
)
def test_plan_selected_joint_path_normalizes_target_forms(
    start: JointState, goal: JointState, expected_start: list[float], expected_goal: list[float]
) -> None:
    group = _group("arm", ("joint_a", "joint_b"))
    result = RRTConnectPlanner().plan_selected_joint_path(
        _World(), PlanningGroupSelection.from_groups((group,)), start, goal
    )

    assert result.status == PlanningStatus.SUCCESS
    assert result.path is not None
    assert result.path[0].name == ["arm/joint_a", "arm/joint_b"]
    assert result.path[0].position == expected_start
    assert result.path[-1].position == expected_goal


@pytest.mark.parametrize(
    ("start", "goal", "status", "message"),
    [
        (
            JointState({"name": ["arm/joint_a"], "position": [0.0]}),
            JointState({"position": [0.0, 0.0]}),
            PlanningStatus.INVALID_START,
            "missing",
        ),
        (
            JointState({"position": [0.0, 0.0]}),
            JointState(
                {"name": ["arm/joint_a", "arm/joint_b", "arm/extra"], "position": [0.0, 0.0, 0.0]}
            ),
            PlanningStatus.INVALID_GOAL,
            "extra",
        ),
        (
            JointState({"name": ["arm/joint_a", "joint_b"], "position": [0.0, 0.0]}),
            JointState({"position": [0.0, 0.0]}),
            PlanningStatus.INVALID_START,
            "mixes",
        ),
    ],
)
def test_plan_selected_joint_path_rejects_bad_targets(
    start: JointState, goal: JointState, status: PlanningStatus, message: str
) -> None:
    group = _group("arm", ("joint_a", "joint_b"))
    result = RRTConnectPlanner().plan_selected_joint_path(
        _World(), PlanningGroupSelection.from_groups((group,)), start, goal
    )

    assert result.status == status
    assert message in result.message


def test_plan_selected_joint_path_rejects_local_names_for_multi_group_selection() -> None:
    selection = PlanningGroupSelection.from_groups(
        (_group("arm", ("joint_a",)), _group("gripper", ("gripper",)))
    )

    result = RRTConnectPlanner().plan_selected_joint_path(
        _World(),
        selection,
        JointState({"name": ["joint_a", "gripper"], "position": [0.0, 0.0]}),
        JointState({"position": [0.1, 0.2]}),
    )

    assert result.status == PlanningStatus.INVALID_START
    assert "multi-group" in result.message


def test_plan_selected_joint_path_direct_edge_projects_full_state_with_unselected_joints() -> None:
    world = _World(current=[0.0, 0.0, 0.77])
    group = _group("arm", ("joint_a", "joint_b"))

    result = RRTConnectPlanner().plan_selected_joint_path(
        world,
        PlanningGroupSelection.from_groups((group,)),
        JointState({"position": [0.1, 0.2]}),
        JointState({"position": [0.3, 0.4]}),
    )

    assert result.status == PlanningStatus.SUCCESS
    assert world.projected_states
    assert all(state.name == ["joint_a", "joint_b", "gripper"] for state in world.projected_states)
    assert all(state.position[2] == 0.77 for state in world.projected_states)
