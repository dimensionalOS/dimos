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

import pytest

from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
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
    assert msg.frame_id == ""  # no task-name address in the payload
    assert [msg.linear.x, msg.linear.y, msg.linear.z] == [0.1, 0.2, 0.3]
    assert [msg.angular.x, msg.angular.y, msg.angular.z] == [0.4, 0.5, 0.6]


def test_publish_twist_defaults_to_zero_twist(module: KeyboardTeleopModule, mocker) -> None:
    publish = mocker.patch.object(module.ee_twist_command, "publish")

    module._publish_twist()

    msg = publish.call_args.args[0]
    assert [msg.linear.x, msg.linear.y, msg.linear.z] == [0.0, 0.0, 0.0]
    assert [msg.angular.x, msg.angular.y, msg.angular.z] == [0.0, 0.0, 0.0]


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
    assert [msg.linear.x, msg.linear.y, msg.linear.z] == [0.0, 0.0, 0.0]
    assert [msg.angular.x, msg.angular.y, msg.angular.z] == [0.0, 0.0, 0.0]


def test_keyup_preserves_remaining_motion_key(module: KeyboardTeleopModule, mocker) -> None:
    publish = mocker.patch.object(module.ee_twist_command, "publish")
    held = {keyboard_mod.pygame.K_w, keyboard_mod.pygame.K_a}
    event = keyboard_mod.pygame.event.Event(keyboard_mod.pygame.KEYUP, key=keyboard_mod.pygame.K_w)

    module._handle_pygame_event(event, held)

    assert held == {keyboard_mod.pygame.K_a}
    assert publish.call_count == 1
    msg = publish.call_args.args[0]
    assert [msg.linear.x, msg.linear.y, msg.linear.z] == [0.0, 0.05, 0.0]


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
