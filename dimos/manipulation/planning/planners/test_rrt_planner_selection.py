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
import math
from pathlib import Path

import numpy as np
import pytest

from dimos.manipulation.planning.groups.models import (
    PlanningGroup,
    PlanningGroupDefinition,
    PlanningGroupSelection,
)
from dimos.manipulation.planning.planners.roboplan_config import RoboPlanCartesianPathConfig
from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner
from dimos.manipulation.planning.planners.selected_joint_space import SelectedJointSpace
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import PlanarBaseDefinition, RobotModel


def _pose() -> PoseStamped:
    return PoseStamped(position=Vector3(), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))


def _group(name: str, joints: tuple[str, ...]) -> PlanningGroup:
    return PlanningGroup(
        id=name,
        joint_names=tuple(f"arm/{joint}" for joint in joints),
        base_link="base",
        tip_link="tool",
    )


class _World:
    is_finalized = True

    def __init__(self, current: list[float] | None = None) -> None:
        self.current = current or [0.0, 0.0, 0.7]
        self.projected_states: list[JointState] = []
        self.config = RobotModelConfig(
            model=RobotModel.from_file(Path("robot.urdf")),
            base_pose=_pose(),
            joint_names=["arm/joint_a", "arm/joint_b", "arm/gripper"],
            base_link="base",
            planning_groups=[
                PlanningGroupDefinition("arm", ("arm/joint_a", "arm/joint_b"), "base", "tool")
            ],
        )

    def get_model_config(self) -> RobotModelConfig:
        return self.config

    def scratch_context(self) -> nullcontext[None]:
        return nullcontext(None)

    def get_joint_state(self, ctx: object) -> JointState:
        return JointState(
            {"name": ["arm/joint_a", "arm/joint_b", "arm/gripper"], "position": self.current}
        )

    def get_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return np.array([-1.0, -1.0, -1.0]), np.array([1.0, 1.0, 1.0])

    def get_joint_velocity_limits(self) -> np.ndarray:
        return np.ones(3)

    def set_joint_state(self, ctx: object, joint_state: JointState) -> None:
        self.projected_states.append(joint_state)

    def is_collision_free(self, ctx: object) -> bool:
        return True

    def check_config_collision_free(self, joint_state: JointState) -> bool:
        self.projected_states.append(joint_state)
        return True


class _PlanarWorld(_World):
    def __init__(self) -> None:
        super().__init__([0.0, 0.0, 0.0])
        planar = PlanarBaseDefinition(
            velocity_limits=(2.0, 1.0, 4.0),
            acceleration_limits=(4.0, 2.0, 8.0),
        )
        self.config = RobotModelConfig(
            model=RobotModel.from_file(Path("robot.urdf")).with_planar_base(planar),
            base_pose=_pose(),
            joint_names=list(planar.joint_names),
            base_link=planar.root_link,
            planning_groups=[
                PlanningGroupDefinition("moving_base", planar.joint_names, planar.root_link)
            ],
        )

    def get_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return np.full(3, -np.inf), np.full(3, np.inf)

    def get_joint_velocity_limits(self) -> np.ndarray:
        return np.array([2.0, 1.0, 4.0])

    def get_joint_state(self, ctx: object) -> JointState:
        return JointState(name=list(self.config.joint_names), position=self.current)


class _WallPlanarWorld(_PlanarWorld):
    def check_config_collision_free(self, joint_state: JointState) -> bool:
        positions = dict(zip(joint_state.name, joint_state.position, strict=True))
        return not (abs(positions["base/x"]) < 0.15 and abs(positions["base/y"]) < 1.5)


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
            "missing",
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


def test_plan_selected_joint_path_rejects_noncanonical_names() -> None:
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
    assert "missing" in result.message


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
    assert all(
        state.name == ["arm/joint_a", "arm/joint_b", "arm/gripper"]
        for state in world.projected_states
    )
    assert all(state.position[2] == 0.77 for state in world.projected_states)


def test_plan_cartesian_path_is_explicitly_unsupported() -> None:
    group = _group("arm", ("joint_a", "joint_b"))
    selection = PlanningGroupSelection.from_groups((group,))

    result = RRTConnectPlanner().plan_cartesian_path(
        _World(),
        selection,
        JointState({"position": [0.0, 0.0]}),
        {
            group.id: (
                Transform.identity(),
                Transform(translation=Vector3(0.1, 0.0, 0.0)),
            )
        },
        RoboPlanCartesianPathConfig(),
    )

    assert result.status == PlanningStatus.UNSUPPORTED
    assert result.path == []
    assert "does not support Cartesian" in result.message


def test_planar_selected_space_uses_request_local_domain_and_velocity_metric() -> None:
    world = _PlanarWorld()
    group = PlanningGroup("moving_base", tuple(world.config.joint_names), world.config.base_link)
    space = SelectedJointSpace.from_world(world, PlanningGroupSelection.from_groups((group,)))
    start = np.array([100.0, -50.0, math.pi - 0.1])
    goal = np.array([104.0, -48.0, -math.pi + 0.1])

    lower, upper = space.planning_domain(start, goal, margin=2.0)

    assert lower == pytest.approx([98.0, -52.0, -math.pi])
    assert upper == pytest.approx([106.0, -46.0, math.pi])
    assert space.distance(start, goal) == pytest.approx(
        math.sqrt((4.0 / 2.0) ** 2 + (2.0 / 1.0) ** 2 + (0.2 / 4.0) ** 2)
    )


def test_planar_direct_path_lifts_yaw_across_wrap_boundary() -> None:
    world = _PlanarWorld()
    group = PlanningGroup("moving_base", tuple(world.config.joint_names), world.config.base_link)

    result = RRTConnectPlanner().plan_selected_joint_path(
        world,
        PlanningGroupSelection.from_groups((group,)),
        JointState(position=[0.0, 0.0, math.pi - 0.1]),
        JointState(position=[0.0, 0.0, -math.pi + 0.1]),
    )

    assert result.status == PlanningStatus.SUCCESS
    assert result.path is not None
    assert result.path[-1].position[2] == pytest.approx(math.pi + 0.1)


def test_planar_rrt_expands_request_domain_for_obstacle_detour(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    world = _WallPlanarWorld()
    group = PlanningGroup("moving_base", tuple(world.config.joint_names), world.config.base_link)

    random = np.random.RandomState(7)
    monkeypatch.setattr(np.random, "uniform", random.uniform)
    result = RRTConnectPlanner(
        step_size=0.25,
        connect_step_size=0.25,
        goal_tolerance=0.1,
        collision_step_size=0.05,
    ).plan_selected_joint_path(
        world,
        PlanningGroupSelection.from_groups((group,)),
        JointState(position=[-2.0, 0.0, 0.0]),
        JointState(position=[2.0, 0.0, 0.0]),
        timeout=2.0,
        max_iterations=2000,
    )

    assert result.status == PlanningStatus.SUCCESS
    assert "2 domain attempts" in result.message
    assert result.iterations > 1000
