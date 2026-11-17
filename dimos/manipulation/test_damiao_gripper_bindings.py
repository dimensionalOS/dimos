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

"""Deployed Damiao planners bind gripper RPCs to real coordinator task names."""

from unittest.mock import MagicMock, call

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.robot.manipulators.dual_makermods_metal.blueprints.basic import (
    dual_makermods_metal_planner_coordinator,
)
from dimos.robot.manipulators.dual_openyam.blueprints.basic import dual_openyam_planner_coordinator
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import teleop_webxr_dual_openyam
from dimos.robot.manipulators.makermods_metal.blueprints.planning import (
    makermods_metal_planner_coordinator,
)
from dimos.robot.manipulators.openarm.blueprints.teleop import teleop_webxr_openarm
from dimos.robot.manipulators.openyam.blueprints.basic import openyam_planner_coordinator
from dimos.robot.manipulators.openyam.blueprints.learning_rollout import build_openyam_rollout
from dimos.robot.manipulators.openyam.blueprints.teleop import (
    keyboard_teleop_openyam,
    keyboard_teleop_openyam_planner,
    teleop_webxr_openyam,
)


@pytest.mark.parametrize(
    "blueprint,expected",
    [
        (makermods_metal_planner_coordinator, {"arm": "metal_gripper"}),
        (
            dual_makermods_metal_planner_coordinator,
            {"left_arm": "left_arm_gripper", "right_arm": "right_arm_gripper"},
        ),
        (openyam_planner_coordinator, {"manipulator": "arm_gripper"}),
        (keyboard_teleop_openyam, {"manipulator": "arm_gripper"}),
        (keyboard_teleop_openyam_planner, {"manipulator": "arm_gripper"}),
        (teleop_webxr_openyam, {"manipulator": "arm_gripper"}),
        (build_openyam_rollout(quest_control=True), {"manipulator": "arm_gripper"}),
        (
            dual_openyam_planner_coordinator,
            {"left_manipulator": "left_arm_gripper", "right_manipulator": "right_arm_gripper"},
        ),
        (
            teleop_webxr_dual_openyam,
            {"left_manipulator": "left_arm_gripper", "right_manipulator": "right_arm_gripper"},
        ),
        (teleop_webxr_openarm, {"left_arm": "left_arm_gripper", "right_arm": "right_arm_gripper"}),
    ],
)
def test_deployed_gripper_command_and_feedback_dispatch(blueprint, expected, module_factory):
    atoms = blueprint.active_blueprints
    planner = next(atom for atom in atoms if issubclass(atom.module, ManipulationModule))
    coordinator = next(atom for atom in atoms if issubclass(atom.module, ControlCoordinator))
    tasks = {task.name: task for task in coordinator.kwargs["tasks"]}
    config = ManipulationModuleConfig(**planner.kwargs)
    assert config.gripper_tasks == expected
    assert "gripper_hardware_id" not in config.model.model_dump()
    for task_name in expected.values():
        assert tasks[task_name].type == "gripper"
        assert len(tasks[task_name].joint_names) == 1
        assert tasks[task_name].joint_names[0].startswith("arm/")

    module = module_factory()
    module.config = config
    module._world_monitor = MagicMock()
    module._world_monitor.planning_groups = PlanningGroupRegistry(config.model.planning_groups)
    module._world_monitor.current_group_joint_state.return_value = None
    module._world_monitor.get_group_ee_pose.return_value = None
    invoke = module._control_coordinator.task_invoke
    invoke.return_value = True
    for index, group in enumerate(expected):
        assert module.set_gripper_position(0.2 + index * 0.5, group).succeeded
    assert invoke.call_args_list == [
        call(task, "set_normalized", {"values": [0.2 + index * 0.5]})
        for index, task in enumerate(expected.values())
    ]
    invoke.reset_mock()
    invoke.return_value = [0.4]
    state = module.get_state()
    assert {group: state.groups[group].gripper_position for group in expected} == dict.fromkeys(
        expected, 0.4
    )
    assert invoke.call_args_list == [call(task, "get_normalized", {}) for task in expected.values()]
