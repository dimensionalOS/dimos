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

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.xarm.blueprints.basic import coordinator_dual_xarm


def _dual_xarm6_planner() -> Blueprint:
    from dimos.robot.manipulators.xarm.blueprints.basic import dual_xarm6_planner

    return dual_xarm6_planner


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


def test_dual_xarm6_planner_blueprint_has_planner() -> None:
    dual_xarm6_planner = _dual_xarm6_planner()
    modules = [atom.module for atom in dual_xarm6_planner.blueprints]

    assert ManipulationModule in modules


def test_dual_xarm6_planner_uses_meshcat_visualization() -> None:
    dual_xarm6_planner = _dual_xarm6_planner()
    visualization = _manipulation_visualization(dual_xarm6_planner)

    assert visualization == {"backend": "meshcat"}


def test_dual_xarm6_planner_robots_use_global_joint_mapping() -> None:
    dual_xarm6_planner = _dual_xarm6_planner()

    for robot in _manipulation_robots(dual_xarm6_planner):
        assert robot.coordinator_task_name == f"traj_{robot.name}"


def test_dual_coordinator_xarm_task_names_match_manipulation_robot_defaults() -> None:
    assert _coordinator_task_names(coordinator_dual_xarm) == [
        "traj_left",
        "traj_right",
    ]
