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


"""OpenYam native CAN topology and connection settings."""

from __future__ import annotations

import can_motor_control
from can_motor_control import damiao
from pydantic import model_validator
from typing_extensions import Self

from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.damiao_connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
)
from dimos.robot.manipulators.openyam.config import make_openyam_model_config
from dimos.utils.data import LfsPath


class OpenYamConnectionConfig(DamiaoConnectionConfig):
    source: str = "yam"
    can_port: str | None = None

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        if self.can_port is not None and not self.can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if self.model is None:
            self.model = OpenYamConnection.control_model()
        return self


class OpenYamConnection(DamiaoConnection):
    config: OpenYamConnectionConfig
    bus_name = "openyam"
    bus_names = [bus_name]
    arm_joints = {"arm": [f"yam_joint{i}" for i in range(1, 7)]}
    gripper_joints = {"gripper": "gripper"}
    kinematic_joint_names = arm_joints["arm"]
    kp = {"arm": [80.0, 80.0, 80.0, 10.0, 10.0, 10.0]}
    kd = {"arm": [5.0, 5.0, 5.0, 1.5, 1.5, 1.5]}
    kinematic_model = RobotModel.from_file(
        LfsPath("yam_description") / "urdf/yam_gripper_gravity.urdf"
    )
    planning = make_openyam_model_config()
    mock_positions = [0.0] * 7

    def _can_devices(self) -> dict[str, str] | None:
        return None if self.config.can_port is None else {"openyam": self.config.can_port}

    def _build_robot(self) -> can_motor_control.Robot:
        arm_motors = [
            can_motor_control.MotorSpec("yam_joint1", damiao.MotorType.DM4340, 0x01, 0x11),
            can_motor_control.MotorSpec("yam_joint2", damiao.MotorType.DM4340, 0x02, 0x12),
            can_motor_control.MotorSpec("yam_joint3", damiao.MotorType.DM4340, 0x03, 0x13),
            can_motor_control.MotorSpec("yam_joint4", damiao.MotorType.DM4310, 0x04, 0x14),
            can_motor_control.MotorSpec("yam_joint5", damiao.MotorType.DM4310, 0x05, 0x15),
            can_motor_control.MotorSpec("yam_joint6", damiao.MotorType.DM4310, 0x06, 0x16),
        ]
        gripper_motor = can_motor_control.MotorSpec(
            "yam_gripper",
            damiao.MotorType.DM4310,
            0x08,
            0x18,
        )
        return (
            can_motor_control.Robot.builder()
            .add_bus(
                self.bus_name,
                self._make_can_bus(self.bus_name),
                damiao.DamiaoCodec(),
            )
            .add_arm("arm", bus=self.bus_name, motors=arm_motors)
            .add_gripper(
                "gripper",
                bus=self.bus_name,
                motor=gripper_motor,
                opening_direction="decreasing_position",
                default_current=0.15,
            )
            .build()
        )
