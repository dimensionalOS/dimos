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

from dimos.robot.unitree.keyboard_teleop import JOY_BUTTONS, joy_buttons


def test_joy_buttons_follow_held_keys() -> None:
    assert joy_buttons(set()) == [0] * len(JOY_BUTTONS)
    assert joy_buttons({pygame.K_w, pygame.K_RSHIFT}) == [1, 0, 0, 0, 0, 0, 1, 0, 0]
    assert joy_buttons({pygame.K_d, pygame.K_LCTRL, pygame.K_SPACE}) == [0, 0, 0, 1, 0, 0, 0, 1, 1]
