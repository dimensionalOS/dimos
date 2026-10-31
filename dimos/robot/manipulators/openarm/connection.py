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

"""OpenArm on the shared Damiao connection lifecycle."""

from __future__ import annotations

from dataclasses import replace

from pydantic import model_validator
from typing_extensions import Self

from dimos.control.components import HardwareComponent
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.common.damiao_connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
    damiao_control_model,
)
from dimos.robot.manipulators.openarm.config import (
    OPENARM_ARM_JOINTS,
    OPENARM_BIMANUAL_MODEL,
    openarm_hardware,
)


def openarm_connection_model(*, robot: str | None = None) -> RobotModel:
    """The existing arm model and normalized grippers, optionally namespaced."""
    model = damiao_control_model(OPENARM_BIMANUAL_MODEL, ("left_gripper", "right_gripper"))
    if robot is not None:
        names = (*OPENARM_ARM_JOINTS, *("left_gripper", "right_gripper"))
        model = model.with_renamed_joints({name: f"{robot}/{name}" for name in names})
    return model


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
            self.model = openarm_connection_model()
        return self


class OpenArmConnection(DamiaoConnection):
    config: OpenArmConnectionConfig

    def hardware_component(self) -> HardwareComponent:
        component = openarm_hardware(
            left_can_port=self.config.left_can_port, right_can_port=self.config.right_can_port
        )
        return replace(component, joints=[*OPENARM_ARM_JOINTS, *("left_gripper", "right_gripper")])
