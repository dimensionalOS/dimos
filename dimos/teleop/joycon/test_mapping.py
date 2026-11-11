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

import pytest

from dimos.teleop.joycon.mapping import (
    DEFAULT_BUTTON_INDEX,
    GripperState,
    JoyConReading,
    apply_deadzone,
    buttons_message,
    pressed_labels,
    step_gripper,
    stick_to_base_velocity,
)
from dimos.teleop.webxr.controller_types import BUTTON_ALIASES, Buttons


def test_buttons_message_lands_on_the_webxr_bits_and_aliases() -> None:
    left = JoyConReading(pressed=frozenset({"ZL", "minus", "stick"}))
    right = JoyConReading(pressed=frozenset({"ZR", "B", "Y", "plus"}))

    buttons = buttons_message(left, right)

    assert buttons.left_trigger and buttons.right_trigger
    assert buttons.left_menu and buttons.right_menu and buttons.left_thumbstick
    assert getattr(buttons, BUTTON_ALIASES["B"]) and getattr(buttons, BUTTON_ALIASES["Y"])
    assert not getattr(buttons, BUTTON_ALIASES["A"]) and not getattr(buttons, BUTTON_ALIASES["X"])
    assert buttons.left_trigger_analog == 1.0 and buttons.right_trigger_analog == 1.0
    assert buttons.data & Buttons.DIGITAL_MASK


def test_buttons_message_with_one_side_missing_is_just_the_other_side() -> None:
    buttons = buttons_message(None, JoyConReading(pressed=frozenset({"A"})))
    assert buttons.right_primary and not buttons.left_trigger
    assert buttons.left_trigger_analog == 0.0
    assert buttons_message(None, None).data == 0


def test_hold_mode_closes_while_held_and_slews() -> None:
    state = GripperState()
    assert step_gripper(state, True, mode="hold", slew_per_s=2.0, dt=0.1) == pytest.approx(0.8)
    assert step_gripper(state, True, mode="hold", slew_per_s=2.0, dt=1.0) == 0.0
    assert step_gripper(state, False, mode="hold", slew_per_s=2.0, dt=0.25) == pytest.approx(0.5)
    assert step_gripper(state, False, mode="hold", slew_per_s=2.0, dt=1.0) == 1.0


def test_toggle_mode_flips_on_the_rising_edge_only() -> None:
    state = GripperState()
    step_gripper(state, True, mode="toggle", slew_per_s=100.0, dt=1.0)
    assert state.closed and state.opening == 0.0
    step_gripper(state, True, mode="toggle", slew_per_s=100.0, dt=1.0)
    assert state.closed
    step_gripper(state, False, mode="toggle", slew_per_s=100.0, dt=1.0)
    assert state.closed
    step_gripper(state, True, mode="toggle", slew_per_s=100.0, dt=1.0)
    assert not state.closed and state.opening == 1.0
    with pytest.raises(ValueError):
        step_gripper(state, True, mode="hold", slew_per_s=0.0, dt=0.1)


def test_deadzone_rescales_to_full_deflection() -> None:
    assert apply_deadzone(0.1, 0.15) == 0.0
    assert apply_deadzone(-0.1, 0.15) == 0.0
    assert apply_deadzone(1.0, 0.15) == 1.0
    assert apply_deadzone(-1.0, 0.15) == -1.0
    assert 0.0 < apply_deadzone(0.5, 0.15) < 0.5
    with pytest.raises(ValueError):
        apply_deadzone(0.5, 1.0)


def test_left_stick_drives_forward_and_left_or_yaw_when_pressed() -> None:
    forward = JoyConReading(stick=(0.0, -1.0))
    assert stick_to_base_velocity(
        forward, deadzone=0.1, max_linear_m_s=0.3, max_angular_rad_s=0.6
    ) == pytest.approx((0.3, 0.0, 0.0))
    left = JoyConReading(stick=(-1.0, 0.0))
    assert stick_to_base_velocity(
        left, deadzone=0.1, max_linear_m_s=0.3, max_angular_rad_s=0.6
    ) == pytest.approx((0.0, 0.3, 0.0))
    yaw = JoyConReading(stick=(1.0, 0.0), pressed=frozenset({"stick"}))
    assert stick_to_base_velocity(
        yaw, deadzone=0.1, max_linear_m_s=0.3, max_angular_rad_s=0.6
    ) == pytest.approx((0.0, 0.0, -0.6))
    assert stick_to_base_velocity(
        None, deadzone=0.1, max_linear_m_s=0.3, max_angular_rad_s=0.6
    ) == (0.0, 0.0, 0.0)


def test_pressed_labels_translate_key_codes() -> None:
    index = DEFAULT_BUTTON_INDEX["right"]
    down = {index["ZR"]: True, index["B"]: True, 99: True}
    assert pressed_labels(down, index) == frozenset({"ZR", "B"})
