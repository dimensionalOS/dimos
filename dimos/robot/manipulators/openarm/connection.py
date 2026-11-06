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

"""OpenArm connection settings: two CAN buses and the OpenArm robot data."""

from __future__ import annotations

from pydantic import model_validator
from typing_extensions import Self

from dimos.robot.manipulators.damiao.connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
)
from dimos.robot.manipulators.openarm.config import OPENARM


class OpenArmConnectionConfig(DamiaoConnectionConfig):
    source: str = "openarm"
    left_can_port: str
    right_can_port: str

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        if not self.left_can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if not self.right_can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if self.left_can_port == self.right_can_port:
            raise ValueError("OpenArm requires distinct CAN selectors")
        if self.model is None:
            self.model = OpenArmConnection.control_model()
        return self


class OpenArmConnection(DamiaoConnection):
    """Both OpenArm arms and jaws, one bus per side."""

    config: OpenArmConnectionConfig
    robot = OPENARM

    def can_ports(self) -> list[str]:
        return [self.config.left_can_port, self.config.right_can_port]
