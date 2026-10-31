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

"""OpenArm topology and direct Damiao connection configuration."""

from __future__ import annotations

from typing import ClassVar, Literal

from pydantic import model_validator
from typing_extensions import Self

from dimos.robot.manipulators.common.damiao_connection import (
    ArmSpec,
    DamiaoConnection,
    DamiaoConnectionConfig,
    DamiaoRobot,
    GripperSpec,
    JointDrive,
)
from dimos.robot.manipulators.common.damiao_model import local_control_model
from dimos.robot.manipulators.openarm.config import (
    OPENARM_BIMANUAL_MODEL,
    OPENARM_JOINTS,
    openarm_bimanual_model_config,
)

# MIT gains measured on v1.0 hardware, carried over as the v2.0 starting
# point: with gravity compensation active the PD terms only handle transient
# tracking, and high kd excites gearbox buzz. Gripper slots bypass MIT
# control, so their gains are 0.
_ARM_KP = (100.0, 100.0, 80.0, 80.0, 60.0, 60.0, 60.0)
_ARM_KD = (1.5, 1.5, 1.0, 1.0, 0.8, 0.8, 0.8)


def _arm(side: str) -> ArmSpec:
    motors: tuple[Literal["DM8009", "DM4340", "DM4310"], ...] = (
        "DM8009",
        "DM8009",
        "DM4340",
        "DM4340",
        "DM4310",
        "DM4310",
        "DM4310",
    )
    return ArmSpec(
        name=f"{side}_arm",
        bus=side,
        joints=tuple(
            JointDrive(
                name=f"openarm_{side}_joint{index}",
                motor=motor,
                send_id=index,
                recv_id=index | 0x10,
                kp=kp,
                kd=kd,
            )
            for index, (motor, kp, kd) in enumerate(zip(motors, _ARM_KP, _ARM_KD, strict=True), 1)
        ),
    )


ROBOT = DamiaoRobot(
    buses=("left", "right"),
    arms=(_arm("left"), _arm("right")),
    grippers=tuple(
        GripperSpec(
            name=f"{side}_gripper",
            bus=side,
            motor="DM4310",
            send_id=0x08,
            recv_id=0x18,
            direction="decreasing_position",
            current=0.15,
        )
        for side in ("left", "right")
    ),
    physical_model=OPENARM_BIMANUAL_MODEL,
    planning=openarm_bimanual_model_config(),
    mock_positions=(0.0,) * len(OPENARM_JOINTS),
    gravity_comp=True,
)


class OpenArmConnectionConfig(DamiaoConnectionConfig):
    source: str = "openarm"
    left_can_port: str | None = None
    right_can_port: str | None = None

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        if self.left_can_port is not None and not self.left_can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if self.right_can_port is not None and not self.right_can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if (self.left_can_port is None) != (self.right_can_port is None):
            raise ValueError("OpenArm requires both CAN selectors")
        if self.left_can_port is not None and self.left_can_port == self.right_can_port:
            raise ValueError("OpenArm requires distinct CAN selectors")
        if self.model is None:
            self.model = local_control_model(ROBOT)
        return self


class OpenArmConnection(DamiaoConnection):
    config: OpenArmConnectionConfig
    robot: ClassVar[DamiaoRobot] = ROBOT

    def _can_devices(self) -> dict[str, str] | None:
        if self.config.left_can_port is None or self.config.right_can_port is None:
            return None
        return {"left": self.config.left_can_port, "right": self.config.right_can_port}
