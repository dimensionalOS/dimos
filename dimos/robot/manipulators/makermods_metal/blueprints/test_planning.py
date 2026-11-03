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

"""The main single-Metal planner entry uses the deployed connection schema."""

from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.robot.manipulators.makermods_metal.blueprints.planning import (
    makermods_metal_planner_coordinator,
)


def test_planner_joint_names_gripper_task_and_can_selection_match():
    blueprint = makermods_metal_planner_coordinator
    parsed = BlueprintConfigParser(blueprint).parse(["--can-port", "serial123"], environ={})
    [planner] = [a for a in blueprint.blueprints if a.module is ManipulationModule]
    [coordinator] = [a for a in blueprint.blueprints if issubclass(a.module, ControlCoordinator)]
    model = ManipulationModuleConfig.model_validate(parsed.module_kwargs(planner.name)).model
    tasks = ControlCoordinatorConfig.model_validate(parsed.module_kwargs(coordinator.name)).tasks
    assert model.joint_names == [f"arm/joint{i}" for i in range(1, 7)]
    assert tasks[0].joint_names == model.joint_names
    assert model.gripper_hardware_id == "metal"
    assert tasks[1].name == "metal_gripper"
    assert tasks[1].joint_names == ["arm/gripper"]
    assert parsed.module_kwargs("metalconnection")["can_port"] == "serial123"
