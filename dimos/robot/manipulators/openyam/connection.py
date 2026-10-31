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


"""OpenYAM topology and direct Damiao connection."""

from __future__ import annotations

from typing import ClassVar

from pydantic import model_validator
from typing_extensions import Self

from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.damiao_connection import (
    ArmSpec,
    DamiaoConnection,
    DamiaoConnectionConfig,
    DamiaoRobot,
    GripperSpec,
    JointDrive,
)
from dimos.robot.manipulators.common.damiao_model import local_control_model
from dimos.robot.manipulators.openyam.config import make_openyam_model_config
from dimos.utils.data import LfsPath

OPENYAM_ROBOT = DamiaoRobot(
    buses=("openyam",),
    arms=(
        ArmSpec(
            name="arm",
            bus="openyam",
            joints=tuple(
                JointDrive(
                    name=f"yam_joint{index}",
                    motor="DM4340" if index <= 3 else "DM4310",
                    send_id=index,
                    recv_id=index + 0x10,
                    kp=80.0 if index <= 3 else 10.0,
                    kd=5.0 if index <= 3 else 1.5,
                )
                for index in range(1, 7)
            ),
        ),
    ),
    grippers=(
        GripperSpec(
            name="gripper",
            bus="openyam",
            motor="DM4310",
            send_id=0x08,
            recv_id=0x18,
            direction="decreasing_position",
            current=0.15,
        ),
    ),
    physical_model=RobotModel.from_file(
        LfsPath("yam_description") / "urdf/yam_gripper_gravity.urdf"
    ),
    planning=make_openyam_model_config(),
    mock_positions=(0.0,) * 7,
    gravity_comp=True,
)


class OpenYamConnectionConfig(DamiaoConnectionConfig):
    source: str = "yam"
    can_port: str | None = None

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        if self.can_port is not None and not self.can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if self.model is None:
            self.model = local_control_model(OPENYAM_ROBOT)
        return self


class OpenYamConnection(DamiaoConnection):
    config: OpenYamConnectionConfig
    robot: ClassVar[DamiaoRobot] = OPENYAM_ROBOT

    def _can_devices(self) -> dict[str, str] | None:
        return None if self.config.can_port is None else {"openyam": self.config.can_port}
