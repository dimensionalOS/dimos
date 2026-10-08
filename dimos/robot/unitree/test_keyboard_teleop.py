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

import pygame
import pytest

from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop


class _Pressed:
    def __init__(self, keys: set[int]) -> None:
        self.keys = keys

    def __getitem__(self, key: int) -> bool:
        return key in self.keys


def test_joystick_publishes_held_keys_on_change(monkeypatch: pytest.MonkeyPatch) -> None:
    teleop = KeyboardTeleop.__new__(KeyboardTeleop)
    teleop._last_buttons = None
    sent: list[Joy] = []
    monkeypatch.setattr(
        teleop, "joystick", type("Port", (), {"publish": lambda _, j: sent.append(j)})()
    )
    held: set[int] = set()
    mods = {"value": 0}
    monkeypatch.setattr(pygame.key, "get_pressed", lambda: _Pressed(held))
    monkeypatch.setattr(pygame.key, "get_mods", lambda: mods["value"])

    teleop._publish_joystick()
    held.add(pygame.K_w)
    teleop._publish_joystick()
    teleop._publish_joystick()
    mods["value"] = pygame.KMOD_LSHIFT
    teleop._publish_joystick()
    held.clear()
    mods["value"] = 0
    teleop._publish_joystick()

    assert [j.buttons for j in sent] == [
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]
