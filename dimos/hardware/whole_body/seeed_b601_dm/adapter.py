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

"""Seeed B601-DM topology for the shared Damiao adapter and motor driver."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any, Literal

import can_motor_control
from can_motor_control import damiao

from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.damiao.adapter import DamiaoWholeBodyAdapter
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.seeed_b601_dm.config import (
    B601_DM_ARM_JOINTS,
    B601_DM_GRIPPER_JOINT,
    B601_DM_MODEL,
)


class SeeedB601DMAdapter(DamiaoWholeBodyAdapter):
    """Six arm motors and a gripper on a shared native CAN bus.

    DM4340 and DM4340P use the same driver profile: position +/-12.5 rad,
    velocity +/-10 rad/s, torque +/-28 Nm. They share the DM4340 enum.
    """

    arm_joints = {"arm": B601_DM_ARM_JOINTS}
    gripper_joints = {"gripper": B601_DM_GRIPPER_JOINT}
    bus_names = ("b601_dm",)
    kinematic_joint_names = arm_joints["arm"]

    def __init__(
        self,
        address: str | Path | None = None,
        *,
        gripper_opening_direction: Literal["increasing_position", "decreasing_position"],
        gripper_current: float,
        runtime_config: DamiaoRuntimeConfig | Mapping[str, Any] | None = None,
        dof: int | None = None,
        hardware_id: str = "b601_dm",
        domain_id: int = 0,
    ) -> None:
        if gripper_opening_direction not in ("increasing_position", "decreasing_position"):
            raise ValueError("configure the measured B601 gripper opening direction")
        if not 0.0 < gripper_current <= 1.0:
            raise ValueError("B601 gripper current must be in (0, 1] per-unit")
        self._gripper_opening_direction = gripper_opening_direction
        self._gripper_current = gripper_current
        super().__init__(
            address,
            runtime_config=runtime_config,
            dof=dof,
            hardware_id=hardware_id,
            domain_id=domain_id,
        )

    @property
    def kinematic_model(self) -> RobotModel:
        return B601_DM_MODEL

    def get_limits(self) -> JointLimits:
        return JointLimits(
            position_lower=[*([None] * 6), 0.0],
            position_upper=[*([None] * 6), 1.0],
            velocity_max=[None] * 7,
        )

    def _build_robot(self) -> can_motor_control.Robot:
        # Seeed rebotarm_dm.yaml: 4340P at IDs 1-3, 4310 at IDs 4-7.
        motors = [
            can_motor_control.MotorSpec(
                name,
                damiao.MotorType.DM4340 if index <= 3 else damiao.MotorType.DM4310,
                index,
                index + 0x10,
            )
            for index, name in enumerate(self.kinematic_joint_names, start=1)
        ]
        return (
            can_motor_control.Robot.builder()
            .add_bus("b601_dm", self._make_can_bus("b601_dm"), damiao.DamiaoCodec())
            .add_arm("arm", bus="b601_dm", motors=motors)
            .add_gripper(
                "gripper",
                bus="b601_dm",
                motor=can_motor_control.MotorSpec("gripper", damiao.MotorType.DM4310, 0x07, 0x17),
                opening_direction=self._gripper_opening_direction,
                default_current=self._gripper_current,
            )
            .build()
        )
