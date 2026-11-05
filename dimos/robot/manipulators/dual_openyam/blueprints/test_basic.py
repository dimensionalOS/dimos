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


from pathlib import Path

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hardware.whole_body.mock.adapter import MockWholeBodyAdapter
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.robot.assets.model import LoadedRobotModel, RobotModel
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    DualOpenYamCoordinator,
    DualOpenYamCoordinatorConfig,
    dual_openyam_planner_coordinator,
)
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_ARM_JOINTS


def test_planner_groups_route_to_their_own_registered_gripper_tasks():
    parsed = BlueprintConfigParser(dual_openyam_planner_coordinator).parse(environ={})
    manipulation = ManipulationModuleConfig.model_validate(
        parsed.module_kwargs(ManipulationModule.name)
    )
    model = manipulation.model
    coordinator = DualOpenYamCoordinatorConfig.model_validate(
        parsed.module_kwargs(DualOpenYamCoordinator.name)
    )
    tasks = {task.name: task for task in coordinator.tasks if task.type == "gripper"}

    assert {name: task.joint_names for name, task in tasks.items()} == {
        "left_arm_gripper": ["left_arm/gripper"],
        "right_arm_gripper": ["right_arm/gripper"],
    }
    assert {
        group: tasks[task].joint_names for group, task in manipulation.gripper_tasks.items()
    } == {
        "left_manipulator": ["left_arm/gripper"],
        "right_manipulator": ["right_arm/gripper"],
    }
    assert [group.name for group in model.planning_groups] == [
        "left_manipulator",
        "right_manipulator",
    ]
    trajectory = next(task for task in coordinator.tasks if task.type == "trajectory")
    assert not set(trajectory.joint_names) & {"left_arm/gripper", "right_arm/gripper"}


@pytest.mark.parametrize("ports", [{}, {"left_can_port": "left", "right_can_port": "right"}])
def test_setup_supplies_model_limits_to_hardware(mocker, ports):
    names = DUAL_OPENYAM_ARM_JOINTS
    # Different bounds and reverse document order catch accidental positional mapping.
    bounds = {name: (-float(i + 1), float(i + 2)) for i, name in enumerate(names)}
    xml = (
        '<robot name="test"><link name="base"/>'
        + "".join(
            f'<link name="{name}_link"/><joint name="{name}" type="revolute">'
            f'<parent link="base"/><child link="{name}_link"/>'
            f'<limit lower="{bounds[name][0]}" upper="{bounds[name][1]}"/></joint>'
            for name in reversed(names)
        )
        + "</robot>"
    )
    mocker.patch.object(
        RobotModel, "load", return_value=LoadedRobotModel(xml, Path("model.urdf"), {})
    )
    mocker.patch.object(ControlCoordinator, "_setup_from_config")
    coordinator = DualOpenYamCoordinator(**ports)
    try:
        coordinator._setup_from_config()
        component = coordinator.config.hardware[0]
        adapter = MockWholeBodyAdapter(dof=len(component.joints))
        hardware = ConnectedWholeBody(adapter, component)
        limits = hardware.get_limits()
        assert limits is not None
        assert limits.position_lower == [*[bounds[name][0] for name in names], 0.0, 0.0]
        assert limits.position_upper == [*[bounds[name][1] for name in names], 1.0, 1.0]
    finally:
        coordinator.stop()
