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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

from types import SimpleNamespace
from unittest.mock import Mock

from dimos_generated.geometry_msgs.msg import Twist, Vector3
from dimos_generated.std_msgs.msg import Float32, Int8
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import pytest

from dimos.control.benchmarking.gate import GATE_ADVANCE, GATE_QUIT, GATE_SKIP
from dimos.robot.unitree import keyboard_teleop as keyboard
from dimos.robot.unitree.g1.skill_container import UnitreeG1SkillContainer


def test_keyboard_generated_operator_gates_and_slider_without_window(monkeypatch):
    module = keyboard.KeyboardTeleop(disable_movement=True)
    module._keys_held = set()
    module._thread = Mock()
    received = []
    slider = []
    motion = []
    monkeypatch.setattr(module.operator_command, "publish", received.append)
    monkeypatch.setattr(module.e_max, "publish", slider.append)
    monkeypatch.setattr(module.cmd_vel, "publish", motion.append)
    monkeypatch.setattr(module, "_update_display", lambda message: None)
    monkeypatch.setattr(keyboard.pygame, "init", Mock())
    monkeypatch.setattr(keyboard.pygame, "quit", Mock())
    monkeypatch.setattr(keyboard.pygame.display, "set_mode", lambda *args: Mock())
    monkeypatch.setattr(keyboard.pygame.display, "set_caption", Mock())
    monkeypatch.setattr(keyboard.pygame.font, "Font", lambda *args: Mock())
    monkeypatch.setattr(
        keyboard.pygame.time,
        "Clock",
        lambda: SimpleNamespace(tick=lambda hz: module._stop_event.set()),
    )
    keys = [
        keyboard.pygame.K_RETURN,
        keyboard.pygame.K_k,
        keyboard.pygame.K_BACKSPACE,
        keyboard.pygame.K_7,
    ]
    events = [SimpleNamespace(type=keyboard.pygame.KEYDOWN, key=key) for key in keys]
    monkeypatch.setattr(keyboard.pygame.event, "get", lambda: events)
    try:
        module._pygame_loop()
        assert [cdr_decode(cdr_encode(value), Int8).data for value in received] == [
            GATE_ADVANCE,
            GATE_SKIP,
            GATE_QUIT,
        ]
        assert cdr_decode(cdr_encode(slider[0]), Float32).data == pytest.approx(0.7)
        assert not motion  # disable_movement keeps this an operator-only interface
    finally:
        module.stop()
    assert len(motion) == 1 and cdr_decode(cdr_encode(motion[0]), Twist) == Twist(
        linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
    )


def test_g1_skill_constructs_generated_twist_at_captured_rpc_boundary():
    module = UnitreeG1SkillContainer()
    gateway = Mock()
    module._connection = gateway
    try:
        module.move(x=0.25, y=-0.125, yaw=0.5, duration=2)
        gateway.move.assert_called_once()
        args, kwargs = gateway.move.call_args
        assert type(args[0]) is Twist
        value = cdr_decode(cdr_encode(args[0]), Twist)
        assert value.linear.x == 0.25 and value.linear.y == -0.125
        assert value.angular.z == 0.5 and value.linear.z == 0
        assert kwargs == {"duration": 2}
    finally:
        module.stop()
