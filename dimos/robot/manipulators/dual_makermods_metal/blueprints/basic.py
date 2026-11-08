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

"""The main bimanual Metal planning setup over a direct connection."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoCoordinator,
    arm_connection,
    arm_remaps,
)
from dimos.robot.manipulators.dual_makermods_metal.config import (
    ALL_ARM_JOINTS,
    DEFAULT_LEFT_XYZ,
    DEFAULT_RIGHT_XYZ,
    FINGER_JOINTS,
    SIDES,
    ZERO_RPY,
    Vector3,
    dual_metal_model_config,
)
from dimos.robot.manipulators.dual_makermods_metal.connection import DualMetalConnection

_MODEL = dual_metal_model_config(robot="arm")


class DualMetalManipulationConfig(ManipulationModuleConfig):
    left_base_xyz: Vector3 = DEFAULT_LEFT_XYZ
    right_base_xyz: Vector3 = DEFAULT_RIGHT_XYZ
    left_base_rpy: Vector3 = ZERO_RPY
    right_base_rpy: Vector3 = ZERO_RPY


class DualMetalManipulationModule(ManipulationModule):
    """Apply the configured mount geometry before creating the planning world."""

    config: DualMetalManipulationConfig

    def _initialize_planning(self) -> None:
        self.config.model = dual_metal_model_config(
            left_xyz=self.config.left_base_xyz,
            right_xyz=self.config.right_base_xyz,
            left_rpy=self.config.left_base_rpy,
            right_rpy=self.config.right_base_rpy,
            robot="arm",
        )
        super()._initialize_planning()


def dual_metal_control() -> Blueprint:
    """Trajectories and both jaws over the two-bus connection."""
    return autoconnect(
        DamiaoCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=_MODEL.model,
            tasks=[
                joint_trajectory_task([f"arm/{name}" for name in ALL_ARM_JOINTS]),
                # Each side's joint7 is open at its lower end.
                *[
                    TaskConfig(
                        name=f"{side}_arm_gripper",
                        type="gripper",
                        joint_names=[f"arm/{FINGER_JOINTS[side][0]}"],
                        priority=20,
                        params={"open_at": "lower"},
                        stream_bind={"gripper_command": f"{side}_gripper_command"},
                    )
                    for side in SIDES
                ],
            ],
            zero_stiffness_joints=[],
        ),
        arm_connection(DualMetalConnection),
    ).remappings(arm_remaps(DualMetalConnection, DamiaoCoordinator))


dual_makermods_metal_planner_coordinator = autoconnect(
    dual_metal_control(),
    DualMetalManipulationModule.blueprint(
        gripper_tasks={"left_arm": "left_arm_gripper", "right_arm": "right_arm_gripper"},
        model=_MODEL,
        visualization={"backend": "viser"},
        default_speed_scale=0.2,
    ),
)
