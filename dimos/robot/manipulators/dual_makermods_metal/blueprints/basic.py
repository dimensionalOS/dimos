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

"""One bimanual ManipulationModule, two Metal adapters, and Viser."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.robot.manipulators.dual_makermods_metal.config import ALL_ARM_JOINTS, SIDES
from dimos.robot.manipulators.dual_makermods_metal.coordinator import DualMetalCoordinator
from dimos.robot.manipulators.dual_makermods_metal.model import (
    DEFAULT_LEFT_XYZ,
    DEFAULT_RIGHT_XYZ,
    ZERO_RPY,
    Vector3,
    dual_metal_model_config,
)


class DualMetalManipulationConfig(ManipulationModuleConfig):
    left_base_xyz: Vector3 = DEFAULT_LEFT_XYZ
    right_base_xyz: Vector3 = DEFAULT_RIGHT_XYZ
    left_base_rpy: Vector3 = ZERO_RPY
    right_base_rpy: Vector3 = ZERO_RPY


class DualMetalManipulationModule(ManipulationModule):
    """Resolve configured rig geometry at startup, before opening the planning world."""

    config: DualMetalManipulationConfig

    def _initialize_planning(self) -> None:
        self.config.model = dual_metal_model_config(
            left_xyz=self.config.left_base_xyz,
            right_xyz=self.config.right_base_xyz,
            left_rpy=self.config.left_base_rpy,
            right_rpy=self.config.right_base_rpy,
        )
        super()._initialize_planning()


dual_makermods_metal_planner_coordinator = autoconnect(
    DualMetalCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=[
            joint_trajectory_task(list(ALL_ARM_JOINTS)),
            *[
                TaskConfig(
                    name=f"{side}_arm_gripper",
                    type="gripper",
                    joint_names=[f"{side}_arm/gripper"],
                    priority=20,
                    stream_bind={"gripper_command": f"{side}_gripper_command"},
                )
                for side in SIDES
            ],
        ],
    ),
    DualMetalManipulationModule.blueprint(
        model=dual_metal_model_config(),
        visualization={"backend": "viser"},
        default_speed_scale=0.2,
    ),
)
