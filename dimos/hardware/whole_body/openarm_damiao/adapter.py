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

"""OpenArm v10 bimanual physical topology for the generic Damiao whole-body adapter."""

from __future__ import annotations

from pathlib import Path

import can_motor_control
from can_motor_control import damiao

from dimos.hardware.whole_body.damiao.adapter import DamiaoWholeBodyAdapter
from dimos.utils.data import LfsPath

# Per-arm motor models, shoulder to wrist, from
# openarm_description/config/arm/v10/joint_limits.yaml. Both arms use the same
# CAN send ids 0x01..0x07 because each arm owns a dedicated bus.
_ARM_MOTOR_TYPES = (
    damiao.MotorType.DM8006,
    damiao.MotorType.DM8006,
    damiao.MotorType.DM4340,
    damiao.MotorType.DM4340,
    damiao.MotorType.DM4310,
    damiao.MotorType.DM4310,
    damiao.MotorType.DM4310,
)


def _arm_motors(side: str) -> list[can_motor_control.MotorSpec]:
    return [
        can_motor_control.MotorSpec(f"openarm_{side}_joint{index}", motor_type, index, index | 0x10)
        for index, motor_type in enumerate(_ARM_MOTOR_TYPES, start=1)
    ]


def _gripper_motor(side: str) -> can_motor_control.MotorSpec:
    return can_motor_control.MotorSpec(
        f"openarm_{side}_gripper",
        damiao.MotorType.DM4310,
        0x08,
        0x18,
    )


class OpenArmDamiaoAdapter(DamiaoWholeBodyAdapter):
    """Two OpenArm v10 arms with grippers, one CAN bus per arm."""

    arm_joints = {
        "left_arm": tuple(f"left_arm/joint{index}" for index in range(1, 8)),
        "right_arm": tuple(f"right_arm/joint{index}" for index in range(1, 8)),
    }
    gripper_joints = {
        "left_gripper": "left_arm/gripper",
        "right_gripper": "right_arm/gripper",
    }
    # Linux assigns can0/can1 in USB enumeration order; remap a swapped rig
    # through DamiaoRuntimeConfig.bus_addresses instead of editing topology.
    bus_defaults = {"left": "can1", "right": "can0"}
    gravity_joint_names = (
        *(f"openarm_left_joint{index}" for index in range(1, 8)),
        *(f"openarm_right_joint{index}" for index in range(1, 8)),
    )

    @property
    def gravity_model_path(self) -> Path:
        """Return the lazy bimanual gravity-compensation URDF path."""
        return LfsPath("openarm_description") / "urdf/robot/openarm_v10_bimanual.urdf"

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
