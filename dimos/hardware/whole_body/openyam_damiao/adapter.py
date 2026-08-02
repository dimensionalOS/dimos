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

"""OpenYAM physical topology for the generic Damiao whole-body adapter."""

from __future__ import annotations

from pathlib import Path

import can_motor_control
from can_motor_control import damiao

from dimos.hardware.whole_body.damiao.adapter import DamiaoWholeBodyAdapter
from dimos.utils.data import LfsPath


class OpenYamDamiaoAdapter(DamiaoWholeBodyAdapter):
    """One OpenYAM arm and calibrated gripper on a shared CAN bus."""

    arm_joints = {
        "arm": tuple(f"arm/joint{index}" for index in range(1, 7)),
    }
    gripper_joints = {"gripper": "arm/gripper"}
    bus_defaults = {"can": "can0"}
    gravity_model_path = Path(LfsPath("yam_description")) / "urdf/yam_gripper_gravity.urdf"
    gravity_joint_names = tuple(f"yam_joint{index}" for index in range(1, 7))

    def _build_robot(self) -> can_motor_control.Robot:
        arm_motors = [
            can_motor_control.MotorSpec(
                f"yam_joint{index}",
                damiao.MotorType.DM4340 if index <= 3 else damiao.MotorType.DM4310,
                index,
                index | 0x10,
            )
            for index in range(1, 7)
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
                "can",
                can_motor_control.SocketCanBus(self.bus_address("can")),
                damiao.DamiaoCodec(),
            )
            .add_arm("arm", bus="can", motors=arm_motors)
            .add_gripper(
                "gripper",
                bus="can",
                motor=gripper_motor,
                opening_direction="decreasing_position",
                default_current=0.15,
            )
            .build()
        )
