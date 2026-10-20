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
from contextlib import ExitStack
import threading

import pytest

from dimos.core.cdr_transport import CdrTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import TwistStamped
from dimos.messages.std_msgs.msg import Float32
import dimos.teleop.keyboard.keyboard_teleop_module as keyboard_mod
from dimos.teleop.keyboard.keyboard_teleop_module import (
    KeyboardTeleopModule,
    _twist_from_keys,
)


class PressedKeys:
    def __init__(self, *keys: int) -> None:
        self._keys = set(keys)

    def __getitem__(self, key: int) -> bool:
        return key in self._keys


@pytest.fixture
def module() -> Iterator[KeyboardTeleopModule]:
    module = KeyboardTeleopModule()
    try:
        yield module
    finally:
        module.stop()


def test_publish_twist_emits_unaddressed_twist_stamped(
    module: KeyboardTeleopModule, mocker
) -> None:
    publish = mocker.patch.object(module.ee_twist_command, "publish")

    module._publish_twist(linear=(0.1, 0.2, 0.3), angular=(0.4, 0.5, 0.6))

    msg = publish.call_args.args[0]
    assert isinstance(msg, TwistStamped)
    assert msg.header.frame_id == ""  # no task-name address in the payload
    assert [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z] == [0.1, 0.2, 0.3]
    assert [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z] == [0.4, 0.5, 0.6]


def test_publish_twist_defaults_to_zero_twist(module: KeyboardTeleopModule, mocker) -> None:
    publish = mocker.patch.object(module.ee_twist_command, "publish")

    module._publish_twist()

    msg = publish.call_args.args[0]
    assert [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z] == [0.0, 0.0, 0.0]
    assert [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z] == [0.0, 0.0, 0.0]


def test_twist_from_keys_maps_translation_keys_to_eef_linear_twist() -> None:
    linear, angular = _twist_from_keys(
        PressedKeys(keyboard_mod.pygame.K_w, keyboard_mod.pygame.K_d, keyboard_mod.pygame.K_q),
        linear_speed=0.05,
        angular_speed=0.5,
    )

    assert linear == (0.05, -0.05, 0.05)
    assert angular == (0.0, 0.0, 0.0)


def test_twist_from_keys_maps_rotation_keys_to_eef_angular_twist() -> None:
    linear, angular = _twist_from_keys(
        PressedKeys(keyboard_mod.pygame.K_r, keyboard_mod.pygame.K_g, keyboard_mod.pygame.K_y),
        linear_speed=0.05,
        angular_speed=0.5,
    )

    assert linear == (0.0, 0.0, 0.0)
    assert angular == (0.5, -0.5, 0.5)


def test_final_key_release_publishes_zero_velocity(module: KeyboardTeleopModule, mocker) -> None:
    publish = mocker.patch.object(module.ee_twist_command, "publish")
    held = {keyboard_mod.pygame.K_w}
    event = keyboard_mod.pygame.event.Event(keyboard_mod.pygame.KEYUP, key=keyboard_mod.pygame.K_w)

    assert not module._handle_pygame_event(event, held)

    assert held == set()
    assert publish.call_count == 1
    msg = publish.call_args.args[0]
    assert [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z] == [0.0, 0.0, 0.0]
    assert [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z] == [0.0, 0.0, 0.0]


def test_keyup_preserves_remaining_motion_key(module: KeyboardTeleopModule, mocker) -> None:
    publish = mocker.patch.object(module.ee_twist_command, "publish")
    held = {keyboard_mod.pygame.K_w, keyboard_mod.pygame.K_a}
    event = keyboard_mod.pygame.event.Event(keyboard_mod.pygame.KEYUP, key=keyboard_mod.pygame.K_w)

    module._handle_pygame_event(event, held)

    assert held == {keyboard_mod.pygame.K_a}
    assert publish.call_count == 1
    msg = publish.call_args.args[0]
    assert [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z] == [0.0, 0.05, 0.0]


def test_keyup_publishes_directly_without_timeout_wait(
    module: KeyboardTeleopModule, mocker
) -> None:
    publish = mocker.patch.object(module.ee_twist_command, "publish")
    held = {keyboard_mod.pygame.K_w}
    event = keyboard_mod.pygame.event.Event(keyboard_mod.pygame.KEYUP, key=keyboard_mod.pygame.K_w)

    module._handle_pygame_event(event, held)

    publish.assert_called_once()


def test_gripper_keys_publish_normalized_opening_only_when_it_changes(
    module: KeyboardTeleopModule, mocker
) -> None:
    publish = mocker.patch.object(module.gripper_command, "publish")
    held: set[int] = set()
    open_event = keyboard_mod.pygame.event.Event(
        keyboard_mod.pygame.KEYDOWN, key=keyboard_mod.pygame.K_LEFTBRACKET
    )
    close_event = keyboard_mod.pygame.event.Event(
        keyboard_mod.pygame.KEYDOWN, key=keyboard_mod.pygame.K_RIGHTBRACKET
    )

    module._handle_pygame_event(open_event, held)
    publish.assert_called_once()
    assert publish.call_args.args[0].data == pytest.approx(1.0)

    publish.reset_mock()
    module._handle_pygame_event(open_event, held)
    publish.assert_not_called()

    module._handle_pygame_event(close_event, held)
    publish.assert_called_once()
    assert publish.call_args.args[0].data == pytest.approx(0.0)


@pytest.mark.parametrize("backend", ["lcm", "zenoh"])
def test_keyboard_output_delivers_cdr_and_key_release_stops_velocity(module, mocker, backend):
    mocker.patch("dimos.msgs.time.time.time_ns", return_value=1_789_654_321_000_000_001)
    config = (
        {"url": "memq://"}
        if backend == "lcm"
        else {
            "scouting": False,
            "multicast": False,
            "gossip": False,
            "connect": [],
            "listen": ["tcp/127.0.0.1:0"],
        }
    )
    received = []
    ready = threading.Event()

    def receive(message):
        received.append(message)
        ready.set()

    assert module.outputs["ee_twist_command"].type is TwistStamped
    assert module.outputs["gripper_command"].type is Float32
    twist = CdrTransport("ee_twist_command", TwistStamped, backend=backend, **config)
    gripper = CdrTransport("gripper_command", Float32, backend=backend, **config)
    module.set_transport("ee_twist_command", twist)
    module.set_transport("gripper_command", gripper)
    # The module fixture owns bound transports and closes them after its safe zero.
    with ExitStack() as resources:
        resources.callback(twist.subscribe(receive))
        resources.callback(gripper.subscribe(receive))
        module._publish_twist(linear=(0.05, 0.0, 0.0))
        assert ready.wait(5), "velocity command did not arrive"
        assert type(received[-1]) is TwistStamped
        assert received[-1].twist.linear.x == 0.05
        assert received[-1].header.stamp == Time(1_789_654_321, 1)
        ready.clear()
        held = {keyboard_mod.pygame.K_w}
        event = keyboard_mod.pygame.event.Event(
            keyboard_mod.pygame.KEYUP, key=keyboard_mod.pygame.K_w
        )
        module._handle_pygame_event(event, held)
        assert ready.wait(5), "key-release zero did not arrive"
        assert received[-1].twist.linear.x == 0.0
        assert held == set()
        ready.clear()
        module._publish_gripper_command(opening=1.0)
        assert ready.wait(5), "gripper command did not arrive"
        assert received[-1] == Float32(1.0)
