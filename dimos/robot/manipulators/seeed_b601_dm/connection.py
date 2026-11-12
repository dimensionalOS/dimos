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

"""Seeed B601-DM connection settings: one CAN bus and the B601-DM robot data.

SAFETY: the arm has no brakes. Disabling the motors (deactivation, a
disconnect, or a feedback fault) lets a raised arm fall, and the arm motors
carry no torque while activation calibrates the gripper. Rest the arm in its
folded pose first, clear the workspace, and keep the 24 V switch within reach.
"""

from __future__ import annotations

from pydantic import model_validator
from typing_extensions import Self

from dimos.robot.manipulators.damiao.connection import (
    DamiaoConnection,
    DamiaoConnectionConfig,
)
from dimos.robot.manipulators.seeed_b601_dm.config import B601_DM


class B601DMConnectionConfig(DamiaoConnectionConfig):
    """One explicitly selected CAN interface or Damiao USB-CAN serial port."""

    source: str = "b601_dm"
    can_port: str

    @model_validator(mode="after")
    def _set_model(self) -> Self:
        if not self.can_port.strip():
            raise ValueError("CAN selector must not be empty")
        if self.model is None:
            self.model = B601_DM.model
        return self


class B601DMConnection(DamiaoConnection):
    """Six arm motors and a calibrated gripper on one Damiao CAN bus.

    DM4340 and DM4340P share the DM4340 driver profile. ``can_port`` is a
    SocketCAN interface (``can0``) by default; point it at the serial port of
    the Damiao USB-CAN bridge shipped with the arm (``/dev/ttyACM0``) to use
    that bridge instead.
    """

    config: B601DMConnectionConfig
    robot = B601_DM

    def can_ports(self) -> list[str]:
        return [self.config.can_port]
