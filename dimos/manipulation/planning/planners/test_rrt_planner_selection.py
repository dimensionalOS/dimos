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

"""Tests for selected-joint RRT planning group contracts."""

from __future__ import annotations

from collections.abc import Callable
from contextlib import nullcontext
from pathlib import Path
from typing import cast

import numpy as np

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import CartesianDelta
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState


def _pose() -> PoseStamped:
    return PoseStamped(position=[0, 0, 0], orientation=[0, 0, 0, 1])


def _robot_config(name: str, joint_names: list[str]) -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=Path("robot.urdf"),
        base_pose=_pose(),
        joint_names=joint_names,
        end_effector_link="tool0",
    )


def _joint_state(names: list[str], positions: list[float]) -> JointState:
    return JointState({"name": names, "position": positions})


def _group(
    group_id: str,
    robot_name: str,
    joint_names: tuple[str, ...],
) -> PlanningGroup:
    return PlanningGroup(
        id=group_id,
        robot_name=robot_name,
        group_name=group_id.split("/", maxsplit=1)[1],
        joint_names=joint_names,
        local_joint_names=tuple(name.split("/", maxsplit=1)[1] for name in joint_names),
        base_link="base_link",
        tip_link="tool0",
    )


def _selection(*groups: PlanningGroup) -> PlanningGroupSelection:
    return PlanningGroupSelection.from_groups(tuple(groups))


class _SelectionWorld:
    is_finalized = True

    def __init__(
        self,
        robot_configs: dict[str, RobotModelConfig],
        coupled_collision_predicate: Callable[[dict[str, JointState]], bool] | None = None,
    ) -> None:
        self._robot_configs = robot_configs
        self._coupled_collision_predicate = coupled_collision_predicate
        self.coupled_collision_checks = 0
        self.config_collision_names: list[list[str]] = []
        self.edge_collision_names: list[tuple[list[str], list[str]]] = []

    def get_robot_config(self, robot_id: str) -> RobotModelConfig:
        return self._robot_configs[robot_id]

    def get_robot_ids(self) -> list[str]:
        return list(self._robot_configs)

    def check_config_collision_free(self, robot_id: str, joint_state: JointState) -> bool:
        self.config_collision_names.append(list(joint_state.name))
        return True

    def get_joint_limits(self, robot_id: str) -> tuple[np.ndarray, np.ndarray]:
        joint_count = len(self._robot_configs[robot_id].joint_names)
        return -np.ones(joint_count), np.ones(joint_count)

    def check_edge_collision_free(
        self,
        robot_id: str,
        start: JointState,
        goal: JointState,
        step_size: float,
    ) -> bool:
        self.edge_collision_names.append((list(start.name), list(goal.name)))
        return True

    def scratch_context(self) -> nullcontext[dict[str, JointState]]:
        return nullcontext({})

    def set_joint_state(
        self, ctx: dict[str, JointState], robot_id: str, joint_state: JointState
    ) -> None:
        assert joint_state.name == self._robot_configs[robot_id].joint_names
        ctx[robot_id] = joint_state

    def is_collision_free(self, ctx: dict[str, JointState], robot_id: str) -> bool:
        self.coupled_collision_checks += 1
        if self._coupled_collision_predicate is None:
            return True
        return self._coupled_collision_predicate(ctx)


def test_plan_selected_joint_path_rejects_missing_and_extra_start_names() -> None:
    group = _group("arm/arm", "arm", ("arm/joint1", "arm/joint2"))
    world = _SelectionWorld(robot_configs={"robot_1": _robot_config("arm", ["joint1", "joint2"])})

    result = RRTConnectPlanner().plan_selected_joint_path(
        cast("WorldSpec", world),
        _selection(group),
        start=_joint_state(["arm/joint1", "arm/extra"], [0.0, 0.0]),
        goal=_joint_state(["arm/joint1", "arm/joint2"], [0.0, 0.0]),
    )

    assert result.status == PlanningStatus.INVALID_START
    assert "missing" in result.message
    assert "extra" in result.message


def test_plan_selected_joint_path_rejects_missing_and_extra_goal_names() -> None:
    group = _group("arm/arm", "arm", ("arm/joint1", "arm/joint2"))
    world = _SelectionWorld(robot_configs={"robot_1": _robot_config("arm", ["joint1", "joint2"])})

    result = RRTConnectPlanner().plan_selected_joint_path(
        cast("WorldSpec", world),
        _selection(group),
        start=_joint_state(["arm/joint1", "arm/joint2"], [0.0, 0.0]),
        goal=_joint_state(["arm/joint1", "arm/extra"], [0.0, 0.0]),
    )

    assert result.status == PlanningStatus.INVALID_GOAL
    assert "missing" in result.message
    assert "extra" in result.message


def test_plan_selected_joint_path_plans_cross_robot_full_group_selection() -> None:
    left_group = _group("left/arm", "left", ("left/joint1",))
    right_group = _group("right/arm", "right", ("right/joint1",))
    world = _SelectionWorld(
        robot_configs={
            "left_robot": _robot_config("left", ["joint1"]),
            "right_robot": _robot_config("right", ["joint1"]),
        },
    )
    joint_state = _joint_state(["left/joint1", "right/joint1"], [0.0, 0.0])

    result = RRTConnectPlanner().plan_selected_joint_path(
        cast("WorldSpec", world),
        _selection(left_group, right_group),
        start=joint_state,
        goal=_joint_state(["left/joint1", "right/joint1"], [0.1, -0.1]),
    )

    assert result.status == PlanningStatus.SUCCESS
    assert len(result.path) == 2
    assert result.path[0].name == ["left/joint1", "right/joint1"]
    assert result.path[-1].position == [0.1, -0.1]
    assert world.coupled_collision_checks > 0


def test_plan_selected_joint_path_converts_single_robot_backend_boundary_to_local() -> None:
    group = _group("arm/manipulator", "arm", ("arm/joint1", "arm/joint2"))
    world = _SelectionWorld(robot_configs={"robot_1": _robot_config("arm", ["joint1", "joint2"])})

    result = RRTConnectPlanner().plan_selected_joint_path(
        cast("WorldSpec", world),
        _selection(group),
        start=_joint_state(["arm/joint2", "arm/joint1"], [0.2, 0.1]),
        goal=_joint_state(["arm/joint1", "arm/joint2"], [0.3, 0.4]),
    )

    assert result.status == PlanningStatus.SUCCESS
    assert [waypoint.name for waypoint in result.path] == [
        ["arm/joint1", "arm/joint2"],
        ["arm/joint1", "arm/joint2"],
    ]
    assert result.path[0].position == [0.1, 0.2]
    assert result.path[-1].position == [0.3, 0.4]
    assert world.config_collision_names == [["joint1", "joint2"], ["joint1", "joint2"]]
    assert world.edge_collision_names == [(["joint1", "joint2"], ["joint1", "joint2"])]


def test_plan_selected_joint_path_rejects_cross_robot_coupled_goal_collision() -> None:
    def coupled_free(ctx: dict[str, JointState]) -> bool:
        if {"left_robot", "right_robot"} - set(ctx):
            return True
        left = ctx["left_robot"].position[0]
        right = ctx["right_robot"].position[0]
        return not (left > 0.04 and right > 0.04)

    left_group = _group("left/arm", "left", ("left/joint1",))
    right_group = _group("right/arm", "right", ("right/joint1",))
    world = _SelectionWorld(
        robot_configs={
            "left_robot": _robot_config("left", ["joint1"]),
            "right_robot": _robot_config("right", ["joint1"]),
        },
        coupled_collision_predicate=coupled_free,
    )

    result = RRTConnectPlanner().plan_selected_joint_path(
        cast("WorldSpec", world),
        _selection(left_group, right_group),
        start=_joint_state(["left/joint1", "right/joint1"], [0.0, 0.0]),
        goal=_joint_state(["left/joint1", "right/joint1"], [0.1, 0.1]),
    )

    assert result.status == PlanningStatus.COLLISION_AT_GOAL
    assert world.coupled_collision_checks > 0


def test_plan_selected_joint_path_rejects_single_robot_subset_selection() -> None:
    group = _group("arm/wrist", "arm", ("arm/joint2",))
    world = _SelectionWorld(robot_configs={"robot_1": _robot_config("arm", ["joint1", "joint2"])})
    joint_state = _joint_state(["arm/joint2"], [0.0])

    result = RRTConnectPlanner().plan_selected_joint_path(
        cast("WorldSpec", world),
        _selection(group),
        start=joint_state,
        goal=joint_state,
    )

    assert result.status == PlanningStatus.UNSUPPORTED


def test_plan_cartesian_path_returns_unsupported_without_ik_fallback() -> None:
    group = _group("arm/manipulator", "arm", ("arm/joint1", "arm/joint2"))
    world = _SelectionWorld(robot_configs={"robot_1": _robot_config("arm", ["joint1", "joint2"])})
    selection = _selection(group)
    start = _joint_state(["arm/joint1", "arm/joint2"], [0.0, 0.0])

    result = RRTConnectPlanner().plan_cartesian_path(
        cast("WorldSpec", world),
        selection,
        start,
        {"arm/manipulator": PoseStamped(position=[0.1, 0.0, 0.0])},
    )

    assert result.status == PlanningStatus.UNSUPPORTED
    assert "Cartesian planning is not supported" in result.message
    assert world.config_collision_names == []
    assert world.edge_collision_names == []
    assert world.coupled_collision_checks == 0


def test_plan_relative_cartesian_path_returns_unsupported_without_ik_fallback() -> None:
    group = _group("arm/manipulator", "arm", ("arm/joint1", "arm/joint2"))
    world = _SelectionWorld(robot_configs={"robot_1": _robot_config("arm", ["joint1", "joint2"])})
    selection = _selection(group)
    start = _joint_state(["arm/joint1", "arm/joint2"], [0.0, 0.0])

    result = RRTConnectPlanner().plan_relative_cartesian_path(
        cast("WorldSpec", world),
        selection,
        start,
        {"arm/manipulator": CartesianDelta(translation=(0.1, 0.0, 0.0))},
    )

    assert result.status == PlanningStatus.UNSUPPORTED
    assert "Cartesian planning is not supported" in result.message
    assert world.config_collision_names == []
    assert world.edge_collision_names == []
    assert world.coupled_collision_checks == 0
