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

"""OpenArm v2.0 bimanual physical topology for the generic Damiao whole-body adapter."""

from __future__ import annotations

import can_motor_control
from can_motor_control import damiao

from dimos.hardware.whole_body.damiao.adapter import DamiaoWholeBodyAdapter
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.openarm.config import OPENARM_BIMANUAL_MODEL


def _arm_motors(side: str) -> list[can_motor_control.MotorSpec]:
    return [
        can_motor_control.MotorSpec(f"openarm_{side}_joint1", damiao.MotorType.DM8009, 0x01, 0x11),
        can_motor_control.MotorSpec(f"openarm_{side}_joint2", damiao.MotorType.DM8009, 0x02, 0x12),
        can_motor_control.MotorSpec(f"openarm_{side}_joint3", damiao.MotorType.DM4340, 0x03, 0x13),
        can_motor_control.MotorSpec(f"openarm_{side}_joint4", damiao.MotorType.DM4340, 0x04, 0x14),
        can_motor_control.MotorSpec(f"openarm_{side}_joint5", damiao.MotorType.DM4310, 0x05, 0x15),
        can_motor_control.MotorSpec(f"openarm_{side}_joint6", damiao.MotorType.DM4310, 0x06, 0x16),
        can_motor_control.MotorSpec(f"openarm_{side}_joint7", damiao.MotorType.DM4310, 0x07, 0x17),
    ]


def _gripper_motor(side: str) -> can_motor_control.MotorSpec:
    return can_motor_control.MotorSpec(
        f"openarm_{side}_gripper",
        damiao.MotorType.DM4310,
        0x08,
        0x18,
    )


class OpenArmDamiaoAdapter(DamiaoWholeBodyAdapter):
    """Two OpenArm v2.0 arms with grippers, one CAN bus per arm."""

    arm_joints = {
        "left_arm": tuple(f"left_arm/joint{index}" for index in range(1, 8)),
        "right_arm": tuple(f"right_arm/joint{index}" for index in range(1, 8)),
    }
    gripper_joints = {
        "left_gripper": "left_arm/gripper",
        "right_gripper": "right_arm/gripper",
    }
    # can0/can1 follow USB enumeration order; remap through
    # DamiaoRuntimeConfig.bus_addresses if the rig comes up swapped.
    bus_defaults = {"left": "can1", "right": "can0"}
    kinematic_joint_names = (
        *(f"openarm_left_joint{index}" for index in range(1, 8)),
        *(f"openarm_right_joint{index}" for index in range(1, 8)),
    )

    @property
    def kinematic_model(self) -> RobotModel:
        """Return the official bimanual arm model."""
        return OPENARM_BIMANUAL_MODEL

    def _build_robot(self) -> can_motor_control.Robot:
        return (
            can_motor_control.Robot.builder()
            .add_bus(
                "left",
                can_motor_control.SocketCanBus(self.bus_address("left")),
                damiao.DamiaoCodec(),
            )
            .add_bus(
                "right",
                can_motor_control.SocketCanBus(self.bus_address("right")),
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
