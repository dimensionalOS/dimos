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

from multiprocessing import Pipe

import pygame
import pytest

from dimos.protocol.encode.cbor import decode
from dimos.robot.unitree.keyboard_teleop import GATE_ADVANCE, JOY_BUTTONS, _DriveWindow, joy_buttons


def test_joy_buttons_follow_held_keys() -> None:
    assert joy_buttons(set()) == [0] * len(JOY_BUTTONS)
    assert joy_buttons({pygame.K_w, pygame.K_RSHIFT}) == [1, 0, 0, 0, 0, 0, 1, 0, 0]
    assert joy_buttons({pygame.K_d, pygame.K_LCTRL, pygame.K_SPACE}) == [0, 0, 0, 1, 0, 0, 0, 1, 1]


def test_drive_window_publishes_typed_controls(monkeypatch) -> None:
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    receive, send = Pipe()
    with receive, send:
        window = _DriveWindow(send, 1.25, 0.8, 2.0, 0.5, True, False, "robot0")
        try:
            window.open()
            events = [
                pygame.event.Event(pygame.KEYDOWN, key=key)
                for key in (pygame.K_w, pygame.K_RETURN, pygame.K_3)
            ]
            batches = iter([events, [pygame.event.Event(pygame.QUIT)]])
            monkeypatch.setattr(pygame.event, "get", lambda: next(batches))
            window.run()

            messages = {}
            while receive.poll():
                name, message = decode(receive.recv_bytes())
                messages[name] = message
            assert messages["cmd_vel"].linear.x == 1.25
            assert list(messages["joystick"].buttons) == [1, 0, 0, 0, 0, 0, 0, 0, 0]
            assert messages["operator_command"].data == GATE_ADVANCE
            assert messages["e_max"].data == pytest.approx(0.3)
        finally:
            pygame.quit()
