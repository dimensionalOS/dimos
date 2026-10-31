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


"""Two YAM follower arms owned by one Damiao connection."""

from __future__ import annotations

from typing import ClassVar

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
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_HOME_JOINTS,
    DUAL_OPENYAM_SIDES,
    dual_openyam_model_config,
)
from dimos.robot.manipulators.dual_openyam.model import DUAL_OPENYAM_MODEL

DUAL_OPENYAM_ROBOT = DamiaoRobot(
    buses=DUAL_OPENYAM_SIDES,
    arms=tuple(
        ArmSpec(
            name=f"{side}_arm",
            bus=side,
            joints=tuple(
                JointDrive(
                    name=f"{side}_joint{index}",
                    motor="DM4340" if index <= 3 else "DM4310",
                    send_id=index,
                    recv_id=index + 0x10,
                    kp=80.0 if index <= 3 else 10.0,
                    kd=5.0 if index <= 3 else 1.5,
                )
                for index in range(1, 7)
            ),
        )
        for side in DUAL_OPENYAM_SIDES
    ),
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
        for side in DUAL_OPENYAM_SIDES
    ),
    physical_model=DUAL_OPENYAM_MODEL,
    planning=dual_openyam_model_config(),
    mock_positions=(*DUAL_OPENYAM_HOME_JOINTS, 0.0, 0.0),
    gravity_comp=True,
)


class DualOpenYamConnectionConfig(DamiaoConnectionConfig):
    source: str = "dual_openyam"
    left_can_port: str | None = None
    right_can_port: str | None = None

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        ports = (self.left_can_port, self.right_can_port)
        if any(port is not None and not port.strip() for port in ports):
            raise ValueError("CAN selector must not be empty")
        if (self.left_can_port is None) != (self.right_can_port is None):
            raise ValueError("Dual OpenYAM requires both CAN selectors")
        if self.left_can_port is not None and self.left_can_port == self.right_can_port:
            raise ValueError("Dual OpenYAM requires distinct CAN selectors")
        if self.model is None:
            self.model = local_control_model(DUAL_OPENYAM_ROBOT)
        return self


class DualOpenYamConnection(DamiaoConnection):
    config: DualOpenYamConnectionConfig
    robot: ClassVar[DamiaoRobot] = DUAL_OPENYAM_ROBOT

    def _can_devices(self) -> dict[str, str] | None:
        if self.config.left_can_port is None or self.config.right_can_port is None:
            return None
        return {"left": self.config.left_can_port, "right": self.config.right_can_port}
