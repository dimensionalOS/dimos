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


"""DualOpenYam native CAN topology and connection settings."""

from __future__ import annotations

import can_motor_control
from can_motor_control import damiao
from pydantic import model_validator
from typing_extensions import Self

from dimos.robot.manipulators.common.damiao_connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_HOME_JOINTS,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.model import DUAL_OPENYAM_MODEL


def _arm_motors(side: str) -> list[can_motor_control.MotorSpec]:
    return [
        can_motor_control.MotorSpec(f"{side}_joint1", damiao.MotorType.DM4340, 0x01, 0x11),
        can_motor_control.MotorSpec(f"{side}_joint2", damiao.MotorType.DM4340, 0x02, 0x12),
        can_motor_control.MotorSpec(f"{side}_joint3", damiao.MotorType.DM4340, 0x03, 0x13),
        can_motor_control.MotorSpec(f"{side}_joint4", damiao.MotorType.DM4310, 0x04, 0x14),
        can_motor_control.MotorSpec(f"{side}_joint5", damiao.MotorType.DM4310, 0x05, 0x15),
        can_motor_control.MotorSpec(f"{side}_joint6", damiao.MotorType.DM4310, 0x06, 0x16),
    ]


def _gripper_motor(side: str) -> can_motor_control.MotorSpec:
    return can_motor_control.MotorSpec(
        f"{side}_gripper",
        damiao.MotorType.DM4310,
        0x08,
        0x18,
    )


class DualOpenYamConnectionConfig(DamiaoConnectionConfig):
    source: str = "dual_openyam"
    left_can_port: str | None = None
    right_can_port: str | None = None

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        ports = (self.left_can_port, self.right_can_port)
        if any(port is not None and not port.strip() for port in ports):
            raise ValueError("CAN selector must not be empty")
        if (self.left_can_port is None) != (self.right_can_port is None):
            raise ValueError("Dual OpenYAM requires both CAN selectors")
        if self.left_can_port is not None and self.left_can_port == self.right_can_port:
            raise ValueError("Dual OpenYAM requires distinct CAN selectors")
        if self.model is None:
            self.model = DualOpenYamConnection.control_model()
        return self


class DualOpenYamConnection(DamiaoConnection):
    config: DualOpenYamConnectionConfig
    bus_names = ["left", "right"]
    arm_joints = {
        "left_arm": [f"left_joint{i}" for i in range(1, 7)],
        "right_arm": [f"right_joint{i}" for i in range(1, 7)],
    }
    gripper_joints = {"left_gripper": "left_gripper", "right_gripper": "right_gripper"}
    kinematic_joint_names = [*arm_joints["left_arm"], *arm_joints["right_arm"]]
    kp = {side: [80.0, 80.0, 80.0, 10.0, 10.0, 10.0] for side in ["left_arm", "right_arm"]}
    kd = {side: [5.0, 5.0, 5.0, 1.5, 1.5, 1.5] for side in ["left_arm", "right_arm"]}
    kinematic_model = DUAL_OPENYAM_MODEL
    planning = dual_openyam_model_config()
    mock_positions = [*DUAL_OPENYAM_HOME_JOINTS, 0.0, 0.0]

    def _can_devices(self) -> dict[str, str] | None:
        if self.config.left_can_port is None or self.config.right_can_port is None:
            return None
        return {"left": self.config.left_can_port, "right": self.config.right_can_port}

    def _build_robot(self) -> can_motor_control.Robot:
        return (
            can_motor_control.Robot.builder()
            .add_bus(
                "left",
                self._make_can_bus("left"),
                damiao.DamiaoCodec(),
            )
            .add_bus(
                "right",
                self._make_can_bus("right"),
                damiao.DamiaoCodec(),
            )
            .add_arm("left_arm", bus="left", motors=_arm_motors("left"))
            .add_arm("right_arm", bus="right", motors=_arm_motors("right"))
            .add_gripper(
                "left_gripper",
                bus="left",
                motor=_gripper_motor("left"),
                opening_direction="decreasing_position",
                default_current=0.15,
            )
            .add_gripper(
                "right_gripper",
                bus="right",
                motor=_gripper_motor("right"),
                opening_direction="decreasing_position",
                default_current=0.15,
            )
            .build()
        )
