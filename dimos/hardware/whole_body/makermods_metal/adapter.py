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

"""Seeed Metal Arm topology for the shared Damiao adapter and motor driver."""

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
from dimos.robot.manipulators.makermods_metal.config import (
    METAL_ARM_JOINTS,
    METAL_GRIPPER_JOINT,
    METAL_MODEL,
)


class MakerModsMetalAdapter(DamiaoWholeBodyAdapter):
    """Metal firmware profiles on the common Damiao lifecycle and CAN transport."""

    arm_joints = {"arm": METAL_ARM_JOINTS}
    gripper_joints = {"gripper": METAL_GRIPPER_JOINT}
    bus_names = ("metal",)
    kinematic_joint_names = arm_joints["arm"]

    def __init__(
        self,
        address: str | Path | None = None,
        *,
        gripper_opening_direction: Literal["increasing_position", "decreasing_position"],
        gripper_current: float,
        runtime_config: DamiaoRuntimeConfig | Mapping[str, Any] | None = None,
        dof: int | None = None,
        hardware_id: str = "metal",
        domain_id: int = 0,
    ) -> None:
        if gripper_opening_direction not in ("increasing_position", "decreasing_position"):
            raise ValueError("configure the measured Metal gripper opening direction")
        if not 0.0 < gripper_current <= 1.0:
            raise ValueError("Metal gripper current must be in (0, 1] per-unit")
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
        return METAL_MODEL

    def get_limits(self) -> JointLimits:
        return JointLimits(
            position_lower=[*([None] * 6), 0.0],
            position_upper=[*([None] * 6), 1.0],
            velocity_max=[None] * 7,
        )

    def _build_robot(self) -> can_motor_control.Robot:
        profile_type = getattr(damiao, "MotorProfile", None)
        if profile_type is None:
            raise RuntimeError(
                "Metal Arm requires can-motor-control>=0.0.10 with custom MotorProfile support; "
                "stock DM profiles use incompatible firmware scaling"
            )
        # Firmware ranges from MakerMods, registered only on this robot's codec.
        low = profile_type(0x8000, p_max=6.28, v_max=10.0, t_max=30.0)
        shoulder = profile_type(0x8001, p_max=6.28, v_max=10.0, t_max=120.0)
        high = profile_type(0x8002, p_max=6.28, v_max=30.0, t_max=20.0)
        codec = damiao.DamiaoCodec(profiles=[low, shoulder, high])
        profiles = (low, shoulder, low, low, high, high)
        gripper_profile = high
        motors = [
            can_motor_control.MotorSpec(name, profile, index, index + 0x10)
            for index, (name, profile) in enumerate(
                zip(self.kinematic_joint_names, profiles, strict=True), start=1
            )
        ]
        return (
            can_motor_control.Robot.builder()
            .add_bus("metal", self._make_can_bus("metal"), codec)
            .add_arm("arm", bus="metal", motors=motors)
            .add_gripper(
                "gripper",
                bus="metal",
                motor=can_motor_control.MotorSpec("gripper", gripper_profile, 0x07, 0x17),
                opening_direction=self._gripper_opening_direction,
                default_current=self._gripper_current,
            )
            .build()
        )
