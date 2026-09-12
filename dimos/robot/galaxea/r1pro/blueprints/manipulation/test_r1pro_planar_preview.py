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

"""R1 Pro real-hardware and planar-preview blueprint contracts."""

from dimos.control.components import HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_control
from dimos.robot.galaxea.r1pro.blueprints.manipulation.r1pro_planar_preview import (
    r1pro_planar_preview,
)
from dimos.robot.galaxea.r1pro.config import R1PRO_PLANAR_BASE, R1PRO_PLANNING_JOINTS
from dimos.robot.galaxea.r1pro.connection import R1PRO_UPPER_BODY_JOINTS


def test_planar_preview_uses_fake_hardware_for_all_planning_joints() -> None:
    parsed = BlueprintConfigParser(r1pro_planar_preview).parse(environ={})
    manipulation_atom = next(
        atom
        for atom in r1pro_planar_preview.active_blueprints
        if issubclass(atom.module, ManipulationModule)
    )
    coordinator_atom = next(
        atom
        for atom in r1pro_planar_preview.active_blueprints
        if issubclass(atom.module, ControlCoordinator)
    )
    manipulation = ManipulationModuleConfig.model_validate(
        parsed.module_kwargs(manipulation_atom.name)
    )
    coordinator = ControlCoordinatorConfig.model_validate(
        parsed.module_kwargs(coordinator_atom.name)
    )

    assert manipulation.visualization.backend == "viser"
    assert manipulation.trajectory_parametrization is None
    prepared = prepare_robot_model(manipulation.model)
    assert prepared.joint_space.velocity_limits[:3] == R1PRO_PLANAR_BASE.velocity_limits
    assert prepared.joint_space.acceleration_limits[:3] == R1PRO_PLANAR_BASE.acceleration_limits
    assert coordinator.hardware[0].hardware_type == HardwareType.WHOLE_BODY
    assert coordinator.hardware[0].adapter_type == "mock_whole_body"
    assert coordinator.hardware[0].joints == list(R1PRO_PLANNING_JOINTS)
    assert coordinator.tasks[0].joint_names == list(R1PRO_PLANNING_JOINTS)


def test_real_control_keeps_planar_joints_out_of_hardware() -> None:
    blueprint = r1pro_control()
    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    coordinator_atom = next(
        atom for atom in blueprint.active_blueprints if issubclass(atom.module, ControlCoordinator)
    )
    coordinator = ControlCoordinatorConfig.model_validate(
        parsed.module_kwargs(coordinator_atom.name)
    )

    assert [hardware.hardware_type for hardware in coordinator.hardware] == [
        HardwareType.WHOLE_BODY,
        HardwareType.BASE,
    ]
    assert coordinator.hardware[0].joints == R1PRO_UPPER_BODY_JOINTS
    assert coordinator.hardware[1].joints == make_twist_base_joints("chassis")
    real_joint_names = {
        joint_name for hardware in coordinator.hardware for joint_name in hardware.joints
    }
    assert set(R1PRO_PLANAR_BASE.joint_names).isdisjoint(real_joint_names)
