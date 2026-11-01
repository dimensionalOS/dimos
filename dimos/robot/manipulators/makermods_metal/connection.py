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

"""Metal native CAN topology and connection settings."""

from __future__ import annotations

import can_motor_control
from can_motor_control import damiao
from pydantic import model_validator
from typing_extensions import Self

from dimos.robot.manipulators.common.damiao_connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
)
from dimos.robot.manipulators.makermods_metal.config import (
    METAL_ARM_JOINTS,
    METAL_MODEL,
    make_metal_model_config,
)


class MetalConnectionConfig(DamiaoConnectionConfig):
    source: str = "metal"
    can_port: str | None = None

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        if self.can_port is not None and not self.can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if self.model is None:
            self.model = MetalConnection.control_model()
        return self


class MetalConnection(DamiaoConnection):
    config: MetalConnectionConfig
    bus_names = ["metal"]
    arm_joints = {"arm": list(METAL_ARM_JOINTS)}
    gripper_joints = {"gripper": "gripper"}
    kinematic_joint_names = arm_joints["arm"]
    kp = {"arm": [100.0, 100.0, 100.0, 50.0, 20.0, 20.0]}
    kd = {"arm": [5.0, 5.0, 5.0, 5.0, 0.6, 0.6]}
    kinematic_model = METAL_MODEL
    planning = make_metal_model_config()
    mock_positions = [0.0] * 7
    gravity_comp = False
    # The encoder reports -0.05050 rad at joint3's nominal zero stop.
    feedback_clamp_margin_rad = 0.06

    def _can_devices(self) -> dict[str, str] | None:
        return None if self.config.can_port is None else {"metal": self.config.can_port}

    def _build_robot(self) -> can_motor_control.Robot:
        profile_type = getattr(damiao, "MotorProfile", None)
        if profile_type is None:
            raise RuntimeError(
                f"{type(self).__name__} requires can-motor-control>=0.0.11 with custom MotorProfile support; "
                "stock DM profiles use incompatible firmware scaling"
            )
        # Firmware ranges from MakerMods, registered only on this robot's codec.
        low = profile_type(0x8000, p_max=6.28, v_max=10.0, t_max=30.0)
        shoulder = profile_type(0x8001, p_max=6.28, v_max=10.0, t_max=120.0)
        high = profile_type(0x8002, p_max=6.28, v_max=30.0, t_max=20.0)
        codec = damiao.DamiaoCodec(profiles=[low, shoulder, high])
        profiles = [low, shoulder, low, low, high, high]
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
                opening_direction="increasing_position",
                default_current=0.1,
            )
            .build()
        )
