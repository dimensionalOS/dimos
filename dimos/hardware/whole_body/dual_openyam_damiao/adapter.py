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

"""Dual OpenYAM bimanual physical topology for the Damiao whole-body adapter."""

from __future__ import annotations

import can_motor_control
from can_motor_control import damiao

from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.damiao.adapter import DamiaoWholeBodyAdapter
from dimos.robot.assets.model import RobotModel
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


class DualOpenYamDamiaoAdapter(DamiaoWholeBodyAdapter):
    """Two standard YAM follower arms and linear grippers, one bus per side."""

    arm_joints = {
        "left_arm": tuple(f"left_joint{index}" for index in range(1, 7)),
        "right_arm": tuple(f"right_joint{index}" for index in range(1, 7)),
    }
    gripper_joints = {
        "left_gripper": "left_arm/gripper",
        "right_gripper": "right_arm/gripper",
    }
    bus_names = ("left", "right")
    kinematic_joint_names = (*arm_joints["left_arm"], *arm_joints["right_arm"])

    def get_limits(self) -> JointLimits:
        """Declare both grippers in their normalized opening coordinate."""
        arm_count = len(self.kinematic_joint_names)
        return JointLimits(
            position_lower=[*([None] * arm_count), 0.0, 0.0],
            position_upper=[*([None] * arm_count), 1.0, 1.0],
            velocity_max=[None] * len(self.joint_names),
        )

    @property
    def kinematic_model(self) -> RobotModel:
        """Return the authoritative dual copy of the verified OpenYAM URDF."""
        return DUAL_OPENYAM_MODEL

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
