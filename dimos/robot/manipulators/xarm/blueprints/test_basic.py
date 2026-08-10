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

from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hardware.spec import JointLimits
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.robot.manipulators.xarm.blueprints.basic import dual_xarm6_planner_coordinator


def test_dual_xarm_mock_limits_survive_blueprint_config_parsing() -> None:
    coordinator_atom = next(
        atom
        for atom in dual_xarm6_planner_coordinator.active_blueprints
        if issubclass(atom.module, ControlCoordinator)
    )

    parsed = BlueprintConfigParser(dual_xarm6_planner_coordinator).parse(environ={})
    config = ControlCoordinatorConfig.model_validate(parsed.module_kwargs(coordinator_atom.name))

    assert [type(component.limits) for component in config.hardware] == [
        JointLimits,
        JointLimits,
    ]


def test_dual_xarm_arm_only_hardware_does_not_declare_grippers() -> None:
    manipulation_atom = next(
        atom
        for atom in dual_xarm6_planner_coordinator.active_blueprints
        if issubclass(atom.module, ManipulationModule)
    )

    parsed = BlueprintConfigParser(dual_xarm6_planner_coordinator).parse(environ={})
    config = ManipulationModuleConfig.model_validate(parsed.module_kwargs(manipulation_atom.name))

    assert config.model.gripper_hardware_id is None

    coordinator_atom = next(
        atom
        for atom in dual_xarm6_planner_coordinator.active_blueprints
        if issubclass(atom.module, ControlCoordinator)
    )
    coordinator_config = ControlCoordinatorConfig.model_validate(
        parsed.module_kwargs(coordinator_atom.name)
    )

    assert all(task.type != "gripper" for task in coordinator_config.tasks)
    assert all(
        not joint.endswith("/gripper")
        for hardware in coordinator_config.hardware
        for joint in hardware.joints
    )
