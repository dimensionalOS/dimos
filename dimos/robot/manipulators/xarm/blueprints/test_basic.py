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

import os
import subprocess
import sys
from typing import get_origin, get_type_hints

import pytest

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.contract.limits import limits, movable_joints
from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.control.tasks.registry import control_task_registry
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.stream import In
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.robot.manipulators.common.mixed import coordinator_piper_xarm, coordinator_teleop_dual
from dimos.robot.manipulators.xarm.blueprints.basic import (
    coordinator_dual_xarm,
    coordinator_xarm7,
    dual_xarm6_planner_coordinator,
)
from dimos.robot.manipulators.xarm.connection import XArmConnection

pytestmark = pytest.mark.self_hosted


def _connection_joints(connections: list[ConnectionConfig]) -> list[str]:
    return [joint.name for connection in connections for joint in movable_joints(connection.model)]


@pytest.mark.parametrize(
    "blueprint", [coordinator_piper_xarm.solidify(), coordinator_dual_xarm.solidify()]
)
def test_dual_arm_trajectory_uses_unique_connection_joints(blueprint) -> None:
    atom = next(
        atom for atom in blueprint.active_blueprints if issubclass(atom.module, ControlCoordinator)
    )
    names = _connection_joints(atom.kwargs["connections"])
    assert len(names) == len(set(names))

    config = next(task for task in atom.kwargs["tasks"] if task.type == "trajectory")
    task = control_task_registry.create(config.type, config, connections={})
    assert task.name == "joint_trajectory"
    assert config.joint_names == names


def test_standalone_xarm7_has_only_arm_tasks() -> None:
    blueprint = coordinator_xarm7.solidify()
    atom = next(
        atom for atom in blueprint.active_blueprints if issubclass(atom.module, ControlCoordinator)
    )
    joints = set(_connection_joints(atom.kwargs["connections"]))
    assert joints == {f"arm/joint{i}" for i in range(1, 8)}
    assert any(atom.module is XArmConnection for atom in blueprint.active_blueprints)
    for config in atom.kwargs["tasks"]:
        assert set(config.joint_names) <= joints
        task = control_task_registry.create(config.type, config, connections={})
        assert task.name == "joint_trajectory"


def test_dual_xarm_mock_limits_survive_blueprint_config_parsing() -> None:
    coordinator_atom = next(
        atom
        for atom in dual_xarm6_planner_coordinator.solidify().active_blueprints
        if issubclass(atom.module, ControlCoordinator)
    )

    parsed = BlueprintConfigParser(dual_xarm6_planner_coordinator.solidify()).parse(environ={})
    config = ControlCoordinatorConfig.model_validate(parsed.module_kwargs(coordinator_atom.name))

    assert [connection.source for connection in config.connections] == ["left", "right"]
    for connection in config.connections:
        assert len(limits(connection)) == 6


def test_dual_xarm_models_grippers_without_control_tasks() -> None:
    manipulation_atom = next(
        atom
        for atom in dual_xarm6_planner_coordinator.solidify().active_blueprints
        if issubclass(atom.module, ManipulationModule)
    )

    parsed = BlueprintConfigParser(dual_xarm6_planner_coordinator.solidify()).parse(environ={})
    config = ManipulationModuleConfig.model_validate(parsed.module_kwargs(manipulation_atom.name))

    assert [group.tip_link for group in config.model.planning_groups[:2]] == [
        "left/link_tcp",
        "right/link_tcp",
    ]

    coordinator_atom = next(
        atom
        for atom in dual_xarm6_planner_coordinator.solidify().active_blueprints
        if issubclass(atom.module, ControlCoordinator)
    )
    coordinator_config = ControlCoordinatorConfig.model_validate(
        parsed.module_kwargs(coordinator_atom.name)
    )

    assert all(task.type != "gripper" for task in coordinator_config.tasks)
    assert all("/joint" in joint for joint in _connection_joints(coordinator_config.connections))


def test_mixed_arm_task_bindings_have_coordinator_inputs() -> None:
    atom = next(
        atom
        for atom in coordinator_teleop_dual.solidify().active_blueprints
        if issubclass(atom.module, ControlCoordinator)
    )
    annotations = get_type_hints(atom.module)
    for task in atom.kwargs["tasks"]:
        for binding in control_task_registry.bindings_for(task.type).consumes:
            port = task.stream_bind.get(binding.stream, binding.stream)
            assert get_origin(annotations.get(port)) is In, (task.name, port)


def test_real_arm_blueprints_load_without_mujoco() -> None:
    """mujoco is an optional extra; without --simulation the blueprints must not need it."""
    code = (
        "import sys; sys.modules['mujoco'] = None; "
        "import dimos.robot.manipulators.xarm.blueprints.basic"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        timeout=120,
        env={**os.environ, "SIMULATION": ""},
    )
    assert result.returncode == 0, result.stderr
