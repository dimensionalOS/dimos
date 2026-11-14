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

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.keyboard_teleop import JOY_BUTTONS, KeyboardTeleop, joy_buttons


def test_joy_buttons_follow_held_keys() -> None:
    assert joy_buttons(set()) == [0] * len(JOY_BUTTONS)
    assert joy_buttons({pygame.K_w, pygame.K_RSHIFT}) == [1, 0, 0, 0, 0, 0, 1, 0, 0]
    assert joy_buttons({pygame.K_d, pygame.K_LCTRL, pygame.K_SPACE}) == [0, 0, 0, 1, 0, 0, 0, 1, 1]


def test_window_forwarding_ends_on_a_stop(mocker) -> None:
    """The window's last message can be a movement, e.g. with a key held at shutdown."""
    module = KeyboardTeleop()
    try:
        publish = mocker.patch.object(module.cmd_vel, "publish")
        moving = Twist(linear=Vector3(0.5, 0, 0))
        module._window = mocker.Mock(stdout=[f"cmd_vel {moving.lcm_encode().hex()}\n"])

        module._publish_window_messages()
        module._stop_event.set()  # stop() was asked for: the movement is not forwarded
        module._publish_window_messages()

        assert [call.args[0].linear.x for call in publish.call_args_list] == [0.5, 0.0, 0.0]
    finally:
        module.stop()
