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

"""MakerMods Metal topology owned directly by its connection module."""

from pydantic import model_validator
from typing_extensions import Self

from dimos.robot.manipulators.common.damiao_connection import (
    ArmSpec,
    DamiaoConnection,
    DamiaoConnectionConfig,
    DamiaoRobot,
    GripperSpec,
    JointDrive,
    MotorProfileSpec,
)
from dimos.robot.manipulators.common.damiao_model import local_control_model
from dimos.robot.manipulators.makermods_metal.config import (
    METAL_ARM_JOINTS,
    METAL_MODEL,
    make_metal_model_config,
)

_LOW = MotorProfileSpec(0x8000, p_max=6.28, v_max=10.0, t_max=30.0)
_SHOULDER = MotorProfileSpec(0x8001, p_max=6.28, v_max=10.0, t_max=120.0)
_HIGH = MotorProfileSpec(0x8002, p_max=6.28, v_max=30.0, t_max=20.0)
_METAL_PROFILES = (_LOW, _SHOULDER, _LOW, _LOW, _HIGH, _HIGH)
_METAL_KP = (100.0, 100.0, 100.0, 50.0, 20.0, 20.0)
_METAL_KD = (5.0, 5.0, 5.0, 5.0, 0.6, 0.6)
METAL_ROBOT = DamiaoRobot(
    buses=("metal",),
    arms=(
        ArmSpec(
            "arm",
            "metal",
            tuple(
                JointDrive(name, motor, index, index + 0x10, kp, kd)
                for index, (name, motor, kp, kd) in enumerate(
                    zip(METAL_ARM_JOINTS, _METAL_PROFILES, _METAL_KP, _METAL_KD, strict=True),
                    start=1,
                )
            ),
        ),
    ),
    grippers=(GripperSpec("gripper", "metal", _HIGH, 0x07, 0x17, "increasing_position", 0.1),),
    physical_model=METAL_MODEL,
    planning=make_metal_model_config(),
    mock_positions=(0.0,) * 7,
    gravity_comp=False,
    # The encoder reports -0.05050 rad at joint3's nominal zero stop.
    feedback_margin_rad=0.06,
)


class MetalConnectionConfig(DamiaoConnectionConfig):
    source: str = "metal"
    can_port: str | None = None

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        if self.can_port is not None and not self.can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if self.model is None:
            self.model = local_control_model(METAL_ROBOT)
        return self


class MetalConnection(DamiaoConnection):
    config: MetalConnectionConfig
    robot = METAL_ROBOT

    def _can_devices(self) -> dict[str, str] | None:
        return None if self.config.can_port is None else {"metal": self.config.can_port}
