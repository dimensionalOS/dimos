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

"""OpenArm Mini connection settings: two serial ports for the two leader arms."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

from dimos.core.stream import In
from dimos.robot.manipulators.feetech.connection import (
    FeetechConnection,
    FeetechConnectionConfig,
)
from dimos.robot.manipulators.openarm.config import OPENARM_MINI_LEADERS
from dimos.teleop.webxr.controller_types import Buttons


class OpenArmMiniConnectionConfig(FeetechConnectionConfig):
    """Serial ports of the two OpenArm Mini leader arms.

    Attributes:
        left_serial_port: The left leader's serial port, e.g. "/dev/ttyACM0";
            empty leaves that leader unread.
        right_serial_port: The right leader's serial port; empty leaves it unread.
    """

    left_serial_port: str = ""
    right_serial_port: str = ""


class OpenArmMiniConnection(FeetechConnection):
    """The two OpenArm Mini leader arms; the serial ports given select which are read."""

    config: OpenArmMiniConnectionConfig
    robot = OPENARM_MINI_LEADERS

    def serial_ports(self) -> dict[str, str]:
        return {"left": self.config.left_serial_port, "right": self.config.right_serial_port}


class OpenArmMiniJoyConConnection(OpenArmMiniConnection):
    """Arm-only leaders whose grippers and pause button are supplied by Joy-Cons."""

    robot = replace(
        OPENARM_MINI_LEADERS,
        buses=tuple(replace(bus, trigger=None) for bus in OPENARM_MINI_LEADERS.buses),
    )
    teleop_buttons: In[Buttons]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._enabled = True
        self._menu_down = False

    async def handle_teleop_buttons(self, buttons: Buttons) -> None:
        with self._hardware_lock:
            down = bool(buttons.right_menu)
            if down and not self._menu_down:
                self._enabled = not self._enabled
                self._filtered.clear()
            self._menu_down = down

    def publish_reading(self, readings: dict[str, dict[str, float]]) -> None:
        with self._hardware_lock:
            if self._enabled:
                super().publish_reading(readings)
