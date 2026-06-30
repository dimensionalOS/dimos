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

"""Tests for dual-arm control blueprints."""

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.groups.identifiers import make_global_joint_names
from dimos.robot.manipulators.xarm.blueprints.basic import coordinator_dual_xarm


def _dual_xarm6_planner_coordinator() -> Blueprint:
    try:
        from dimos.manipulation.blueprints import dual_xarm6_planner_coordinator
    except RuntimeError as exc:
        if "Failed to pull LFS file" in str(exc):
            pytest.skip(f"xArm LFS data unavailable: {exc}")
        raise
    return dual_xarm6_planner_coordinator


def _coordinator_task_names(blueprint) -> list[str]:
    atom = next(atom for atom in blueprint.blueprints if atom.module is ControlCoordinator)
    return [task.name for task in atom.kwargs["tasks"]]


def _coordinator_tasks(blueprint):
    atom = next(atom for atom in blueprint.blueprints if atom.module is ControlCoordinator)
    return atom.kwargs["tasks"]


def _manipulation_robots(blueprint):
    atom = next(atom for atom in blueprint.blueprints if atom.module is ManipulationModule)
    return atom.kwargs["robots"]


def _manipulation_visualization(blueprint):
    atom = next(atom for atom in blueprint.blueprints if atom.module is ManipulationModule)
    return atom.kwargs["visualization"]


def test_dual_xarm6_integrated_blueprint_has_planner_and_coordinator() -> None:
    dual_xarm6_planner_coordinator = _dual_xarm6_planner_coordinator()
    modules = [atom.module for atom in dual_xarm6_planner_coordinator.blueprints]

    assert ManipulationModule in modules
    assert ControlCoordinator in modules


def test_dual_xarm6_integrated_blueprint_uses_viser_for_execution_ui() -> None:
    dual_xarm6_planner_coordinator = _dual_xarm6_planner_coordinator()
    visualization = _manipulation_visualization(dual_xarm6_planner_coordinator)

    assert visualization == {"backend": "viser", "allow_plan_execute": True}


def test_dual_xarm6_integrated_tasks_match_planner_robots() -> None:
    dual_xarm6_planner_coordinator = _dual_xarm6_planner_coordinator()
    tasks_by_name = {task.name: task for task in _coordinator_tasks(dual_xarm6_planner_coordinator)}

    for robot in _manipulation_robots(dual_xarm6_planner_coordinator):
        task = tasks_by_name[robot.coordinator_task_name]
        assert task.joint_names == make_global_joint_names(robot.name, robot.joint_names)


def test_dual_coordinator_xarm_task_names_match_split_blueprint() -> None:
    assert _coordinator_task_names(coordinator_dual_xarm) == [
        "traj_left",
        "traj_right",
    ]
