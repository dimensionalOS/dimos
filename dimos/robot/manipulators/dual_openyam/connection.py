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

"""Dual OpenYAM connection settings: two CAN buses for the arms, two serial ports for the leaders."""

from __future__ import annotations

from pathlib import Path

from pydantic import model_validator
from typing_extensions import Self

from dimos.robot.manipulators.damiao.connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
)
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM, DUAL_OPENYAM_LEADERS
from dimos.robot.manipulators.feetech.connection import (
    FeetechConnection,
    FeetechConnectionConfig,
)


class DualOpenYamConnectionConfig(DamiaoConnectionConfig):
    source: str = "dual_openyam"
    left_can_port: str
    right_can_port: str

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        ports = (self.left_can_port, self.right_can_port)
        if any(not port.strip() for port in ports):
            raise ValueError("CAN selector must not be empty")
        if self.left_can_port == self.right_can_port:
            raise ValueError("Dual OpenYAM requires distinct CAN selectors")
        if self.model is None:
            self.model = DUAL_OPENYAM.model
        return self


class DualOpenYamConnection(DamiaoConnection):
    """Two independent OpenYAM arms and jaws on two explicitly chosen buses."""

    config: DualOpenYamConnectionConfig
    robot = DUAL_OPENYAM

    def can_ports(self) -> list[str]:
        return [self.config.left_can_port, self.config.right_can_port]


class DualOpenYamLeaderConnectionConfig(FeetechConnectionConfig):
    """Serial ports and calibration files of the two YAM leader arms.

    Attributes:
        left_serial_port: The left leader's serial port, e.g. "/dev/ttyACM1";
            empty leaves that leader unread.
        right_serial_port: The right leader's serial port; empty leaves it unread.
        left_calibration_path: The left leader's calibration file, if not the default.
        right_calibration_path: The right leader's calibration file, if not the default.
    """

    left_serial_port: str = ""
    right_serial_port: str = ""
    left_calibration_path: Path | None = None
    right_calibration_path: Path | None = None


class DualOpenYamLeaderConnection(FeetechConnection):
    """The two YAM leader arms; the serial ports given select which are read."""

    config: DualOpenYamLeaderConnectionConfig
    robot = DUAL_OPENYAM_LEADERS

    def serial_ports(self) -> dict[str, str]:
        return {"left": self.config.left_serial_port, "right": self.config.right_serial_port}

    def calibration_path(self, bus: str) -> Path:
        explicit = {
            "left": self.config.left_calibration_path,
            "right": self.config.right_calibration_path,
        }.get(bus)
        return explicit if explicit is not None else super().calibration_path(bus)
