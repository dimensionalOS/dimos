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

"""Seeed B601-DM native CAN topology and connection settings.

SAFETY: the arm has no brakes. Disabling the motors (deactivation, a
disconnect, or a feedback fault) lets a raised arm fall, and the arm motors
carry no torque while activation calibrates the gripper. Rest the arm in its
folded pose first, clear the workspace, and keep the 24 V switch within reach.
"""

from __future__ import annotations

import can_motor_control
from can_motor_control import damiao
from pydantic import model_validator
from typing_extensions import Self

from dimos.robot.manipulators.damiao.damiao_connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
)
from dimos.robot.manipulators.seeed_b601_dm.config import (
    B601_DM_MODEL,
    b601_dm_model_config,
)
from dimos.robot.manipulators.seeed_b601_dm.joints import B601_DM_ARM_JOINTS

# Measured on the arm: the gripper motor travels 6.39 rad between its stops,
# and opening moves it toward more negative positions. Calibration drives
# past the full stroke so it reaches both stops from any start.
GRIPPER_CALIBRATION_TRAVEL_RAD = 7.0
GRIPPER_CURRENT = 0.15


class B601DMConnectionConfig(DamiaoConnectionConfig):
    """One explicitly selected CAN interface or Damiao USB-CAN serial port."""

    source: str = "b601_dm"
    can_port: str

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        if not self.can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if self.model is None:
            self.model = B601DMConnection.control_model()
        return self


class B601DMConnection(DamiaoConnection):
    """Six arm motors and a calibrated gripper on one Damiao CAN bus.

    DM4340 and DM4340P share the DM4340 driver profile. ``can_port`` is a
    SocketCAN interface (``can0``) by default; point it at the serial port of
    the Damiao USB-CAN bridge shipped with the arm (``/dev/ttyACM0``) to use
    that bridge instead.
    """

    config: B601DMConnectionConfig
    bus_name = "b601_dm"
    bus_names = [bus_name]
    arm_joints = {"arm": list(B601_DM_ARM_JOINTS)}
    gripper_joints = {"gripper": "gripper"}
    kinematic_joint_names = list(B601_DM_ARM_JOINTS)
    # Seeed's SDK MIT gains; the shoulder kd is capped at the shared codec's
    # 5.0 maximum. The gripper runs its own current-limited control.
    kp = {"arm": [120.0, 120.0, 120.0, 18.0, 18.0, 18.0]}
    kd = {"arm": [5.0, 5.0, 5.0, 2.0, 2.0, 2.0]}
    kinematic_model = B601_DM_MODEL
    planning = b601_dm_model_config()
    gravity_comp = True

    def _can_devices(self) -> dict[str, str]:
        return {self.bus_name: self.config.can_port}

    def _build_robot(self) -> can_motor_control.Robot:
        # Seeed's motor table. DM4340P uses the DM4340 driver profile.
        arm_motors = [
            can_motor_control.MotorSpec("joint1", damiao.MotorType.DM4340, 0x01, 0x11),
            can_motor_control.MotorSpec("joint2", damiao.MotorType.DM4340, 0x02, 0x12),
            can_motor_control.MotorSpec("joint3", damiao.MotorType.DM4340, 0x03, 0x13),
            can_motor_control.MotorSpec("joint4", damiao.MotorType.DM4310, 0x04, 0x14),
            can_motor_control.MotorSpec("joint5", damiao.MotorType.DM4310, 0x05, 0x15),
            can_motor_control.MotorSpec("joint6", damiao.MotorType.DM4310, 0x06, 0x16),
        ]
        gripper_motor = can_motor_control.MotorSpec(
            "gripper_motor",
            damiao.MotorType.DM4310,
            0x07,
            0x17,
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
                default_current=GRIPPER_CURRENT,
                calibration_travel=GRIPPER_CALIBRATION_TRAVEL_RAD,
            )
            .build()
        )
