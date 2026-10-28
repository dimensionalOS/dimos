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

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.makermods_metal.blueprints.planning import (
    makermods_metal_planner_coordinator,
)
from dimos.robot.manipulators.makermods_metal.config import METAL_ARM_JOINTS
from dimos.robot.manipulators.makermods_metal.coordinator import (
    MetalCoordinator,
    MetalCoordinatorConfig,
)


def test_planner_and_coordinator_share_arm_joints_and_state_topic() -> None:
    blueprint = makermods_metal_planner_coordinator
    parsed = BlueprintConfigParser(blueprint).parse([], environ={})
    planner = next(
        atom for atom in blueprint.active_blueprints if atom.module is ManipulationModule
    )
    coordinator = next(
        atom for atom in blueprint.active_blueprints if issubclass(atom.module, ControlCoordinator)
    )
    config = ManipulationModuleConfig.model_validate(parsed.module_kwargs(planner.name))
    assert isinstance(config.visualization, ViserVisualizationConfig)
    assert config.model.joint_names == list(METAL_ARM_JOINTS)
    assert config.model.planning_groups[0].joint_names == METAL_ARM_JOINTS
    assert config.model.planning_groups[0].tip_link == "gripper_base"
    assert config.model.gripper_hardware_id == "metal"
    tasks = MetalCoordinatorConfig.model_validate(parsed.module_kwargs(coordinator.name)).tasks
    trajectory = next(task for task in tasks if task.type == "trajectory")
    assert trajectory.joint_names == config.model.joint_names
    for atom in (planner, coordinator):
        assert "coordinator_joint_state" in [stream.name for stream in atom.streams]
        assert (
            blueprint.remapping_map.get(
                (atom.name, "coordinator_joint_state"), "coordinator_joint_state"
            )
            == "coordinator_joint_state"
        )


def test_planner_preserves_physical_can_configuration() -> None:
    parsed = BlueprintConfigParser(makermods_metal_planner_coordinator).parse(
        [
            "--can-port",
            "serial123",
            "--gripper-opening-direction",
            "increasing_position",
            "--gripper-current",
            "0.1",
        ],
        environ={},
    )
    config = MetalCoordinatorConfig.model_validate(parsed.module_kwargs("ControlCoordinator"))
    assert config.can_port == "serial123"
    assert config.gripper_current == 0.1
    assert config.gripper_opening_direction == "increasing_position"


def test_planner_defaults_to_mock_hardware() -> None:
    parsed = BlueprintConfigParser(makermods_metal_planner_coordinator).parse([], environ={})
    coordinator = MetalCoordinator(**parsed.module_kwargs("ControlCoordinator"))
    try:
        coordinator._setup_from_config()
        assert coordinator.config.hardware[0].adapter_type == "mock_whole_body"
        assert list(coordinator._tasks) == ["joint_trajectory", "metal_gripper"]
    finally:
        coordinator.stop()
