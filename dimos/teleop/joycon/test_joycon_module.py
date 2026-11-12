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

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.teleop.joycon.joycon_module import JoyConTeleopModule, JoyConTeleopModuleConfig
from dimos.teleop.joycon.mapping import JoyConReading, JoyConSide
from dimos.teleop.webxr.controller_types import Buttons


class _FakeBackend:
    def __init__(self) -> None:
        self.readings: dict[JoyConSide, JoyConReading | None] = {
            "left": JoyConReading(),
            "right": JoyConReading(),
        }
        self.opened = False
        self.closed = False

    def open(self) -> None:
        self.opened = True

    def read(self) -> dict[JoyConSide, JoyConReading | None]:
        return dict(self.readings)

    def close(self) -> None:
        self.closed = True


@contextmanager
def _module(
    mocker: MockerFixture, **config: Any
) -> Iterator[tuple[JoyConTeleopModule, _FakeBackend, dict[str, Any]]]:
    module = JoyConTeleopModule(**JoyConTeleopModuleConfig(**config).model_dump())
    backend = _FakeBackend()
    module.use_backend(backend)
    published = {
        name: mocker.patch.object(getattr(module, name), "publish")
        for name in ("teleop_buttons", "left_gripper_command", "right_gripper_command", "cmd_vel")
    }
    backend.open()
    try:
        yield module, backend, published
    finally:
        module.stop()


def test_tick_publishes_buttons_and_both_openings(mocker: MockerFixture) -> None:
    with _module(mocker, gripper_slew_per_s=100.0) as (module, backend, published):
        backend.readings["left"] = JoyConReading(pressed=frozenset({"ZL"}))
        backend.readings["right"] = JoyConReading(pressed=frozenset({"B"}))
        module.tick(now=0.0)
        module.tick(now=0.02)

    buttons = published["teleop_buttons"].call_args.args[0]
    assert isinstance(buttons, Buttons) and buttons.left_trigger and buttons.right_secondary
    left = published["left_gripper_command"].call_args.args[0]
    right = published["right_gripper_command"].call_args.args[0]
    assert isinstance(left, Float32) and left.data == 0.0
    assert isinstance(right, Float32) and right.data == 1.0
    published["cmd_vel"].assert_not_called()


def test_a_dropped_joycon_keeps_its_last_opening_and_the_other_side_runs(
    mocker: MockerFixture,
) -> None:
    with _module(mocker, gripper_slew_per_s=100.0) as (module, backend, published):
        backend.readings["left"] = JoyConReading(pressed=frozenset({"ZL"}))
        module.tick(now=0.0)
        module.tick(now=0.02)
        assert published["left_gripper_command"].call_args.args[0].data == 0.0

        backend.readings["left"] = None
        backend.readings["right"] = JoyConReading(pressed=frozenset({"ZR"}))
        left_calls = published["left_gripper_command"].call_count
        module.tick(now=0.04)
        module.tick(now=0.06)

    assert published["left_gripper_command"].call_count == left_calls
    assert published["right_gripper_command"].call_args.args[0].data == 0.0
    assert not published["teleop_buttons"].call_args.args[0].left_trigger


def test_cmd_vel_follows_the_left_stick_when_enabled(mocker: MockerFixture) -> None:
    with _module(mocker, publish_cmd_vel=True, max_linear_m_s=0.5) as (module, backend, published):
        backend.readings["left"] = JoyConReading(stick=(0.0, -1.0))
        module.tick(now=0.0)

    twist = published["cmd_vel"].call_args.args[0]
    assert isinstance(twist, Twist)
    assert twist.linear.x == pytest.approx(0.5) and twist.angular.z == 0.0


def test_start_and_stop_open_and_close_the_backend(mocker: MockerFixture) -> None:
    module = JoyConTeleopModule(**JoyConTeleopModuleConfig().model_dump())
    backend = _FakeBackend()
    module.use_backend(backend)
    for name in ("teleop_buttons", "left_gripper_command", "right_gripper_command", "cmd_vel"):
        mocker.patch.object(getattr(module, name), "publish")

    module.start()
    try:
        assert backend.opened and module._thread is not None and module._thread.is_alive()
    finally:
        module.stop()
    assert backend.closed and module._thread is None


def test_config_validates_mode_and_rates() -> None:
    config = JoyConTeleopModuleConfig(gripper_mode="toggle", deadzone=0.2)
    assert config.button_index("right")["ZR"] == 313
    assert config.axis_index("right") == {"x": 3, "y": 4}
    assert config.joystick_name("left") == "Joy-Con (L)"
    with pytest.raises(ValueError):
        JoyConTeleopModuleConfig(gripper_slew_per_s=0.0)
    with pytest.raises(ValueError):
        JoyConTeleopModuleConfig(deadzone=1.0)
