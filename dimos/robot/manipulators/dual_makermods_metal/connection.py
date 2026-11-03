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

"""Two Metal arms on distinct CAN buses with the single-arm firmware settings."""

import can_motor_control
from pydantic import model_validator
from typing_extensions import Self

from dimos.robot.manipulators.damiao.damiao_connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
)
from dimos.robot.manipulators.dual_makermods_metal.config import ALL_ARM_JOINTS, ARM_JOINTS
from dimos.robot.manipulators.dual_makermods_metal.model import dual_metal_model_config
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection, build_metal_robot


class DualMetalConnectionConfig(DamiaoConnectionConfig):
    source: str = "dual_metal"
    left_can_port: str
    right_can_port: str

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        if not self.left_can_port.strip() or not self.right_can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if self.left_can_port == self.right_can_port:
            raise ValueError("Dual Metal requires distinct CAN selectors")
        if self.model is None:
            self.model = DualMetalConnection.control_model()
        return self


class DualMetalConnection(DamiaoConnection):
    config: DualMetalConnectionConfig
    bus_names = ["left", "right"]
    arm_joints = {f"{side}_arm": list(joints) for side, joints in ARM_JOINTS.items()}
    gripper_joints = {"left_gripper": "left_gripper", "right_gripper": "right_gripper"}
    kinematic_joint_names = list(ALL_ARM_JOINTS)
    kp = {side: list(MetalConnection.kp["arm"]) for side in arm_joints}
    kd = {side: list(MetalConnection.kd["arm"]) for side in arm_joints}
    planning = dual_metal_model_config()
    kinematic_model = planning.model
    gravity_comp = False
    feedback_clamp_margin_rad = MetalConnection.feedback_clamp_margin_rad

    def _can_devices(self) -> dict[str, str]:
        return {"left": self.config.left_can_port, "right": self.config.right_can_port}

    def _build_robot(self) -> can_motor_control.Robot:
        return build_metal_robot(self)
