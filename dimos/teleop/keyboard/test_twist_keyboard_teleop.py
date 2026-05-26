# Copyright 2025-2026 Dimensional Inc.
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

from dataclasses import FrozenInstanceError

import pytest

from dimos.teleop.keyboard.twist_keyboard_teleop import (
    DEFAULT_ANGULAR_SPEED,
    DEFAULT_BOOST_MULTIPLIER,
    DEFAULT_LINEAR_SPEED,
    DEFAULT_SLOW_MULTIPLIER,
    KeyboardState,
    VelocityConfig,
    keyboard_state_to_twist,
)


@pytest.fixture
def cfg() -> VelocityConfig:
    return VelocityConfig()


class TestKeyboardState:
    def test_defaults_all_false(self) -> None:
        state = KeyboardState()
        assert state.forward is False
        assert state.backward is False
        assert state.strafe_left is False
        assert state.strafe_right is False
        assert state.turn_left is False
        assert state.turn_right is False
        assert state.boost is False
        assert state.slow is False

    def test_frozen(self) -> None:
        state = KeyboardState(forward=True)
        with pytest.raises(FrozenInstanceError):
            state.forward = False  # type: ignore[misc]


class TestVelocityConfig:
    def test_defaults_match_module_constants(self) -> None:
        cfg = VelocityConfig()
        assert cfg.linear_speed == DEFAULT_LINEAR_SPEED
        assert cfg.angular_speed == DEFAULT_ANGULAR_SPEED
        assert cfg.boost_multiplier == DEFAULT_BOOST_MULTIPLIER
        assert cfg.slow_multiplier == DEFAULT_SLOW_MULTIPLIER

    def test_custom_values(self) -> None:
        cfg = VelocityConfig(linear_speed=1.0, angular_speed=2.0)
        assert cfg.linear_speed == 1.0
        assert cfg.angular_speed == 2.0

    def test_mutable(self) -> None:
        cfg = VelocityConfig()
        cfg.linear_speed = 9.9
        assert cfg.linear_speed == 9.9


class TestKeyboardStateToTwist:
    """Tests for the pure key-to-twist mapping function."""

    def test_no_keys_returns_zero_twist(self, cfg: VelocityConfig) -> None:
        twist = keyboard_state_to_twist(KeyboardState(), cfg)
        assert twist.linear.x == 0.0
        assert twist.linear.y == 0.0
        assert twist.linear.z == 0.0
        assert twist.angular.x == 0.0
        assert twist.angular.y == 0.0
        assert twist.angular.z == 0.0

    def test_forward(self, cfg: VelocityConfig) -> None:
        twist = keyboard_state_to_twist(KeyboardState(forward=True), cfg)
        assert twist.linear.x == pytest.approx(cfg.linear_speed)
        assert twist.linear.y == 0.0
        assert twist.angular.z == 0.0

    def test_backward(self, cfg: VelocityConfig) -> None:
        twist = keyboard_state_to_twist(KeyboardState(backward=True), cfg)
        assert twist.linear.x == pytest.approx(-cfg.linear_speed)

    def test_strafe_left(self, cfg: VelocityConfig) -> None:
        twist = keyboard_state_to_twist(KeyboardState(strafe_left=True), cfg)
        assert twist.linear.y == pytest.approx(cfg.linear_speed)
        assert twist.linear.x == 0.0

    def test_strafe_right(self, cfg: VelocityConfig) -> None:
        twist = keyboard_state_to_twist(KeyboardState(strafe_right=True), cfg)
        assert twist.linear.y == pytest.approx(-cfg.linear_speed)

    def test_turn_left(self, cfg: VelocityConfig) -> None:
        twist = keyboard_state_to_twist(KeyboardState(turn_left=True), cfg)
        assert twist.angular.z == pytest.approx(cfg.angular_speed)
        assert twist.linear.x == 0.0
        assert twist.linear.y == 0.0

    def test_turn_right(self, cfg: VelocityConfig) -> None:
        twist = keyboard_state_to_twist(KeyboardState(turn_right=True), cfg)
        assert twist.angular.z == pytest.approx(-cfg.angular_speed)

    # --- Opposing keys: negative direction wins ---

    def test_forward_and_backward_backward_wins(self, cfg: VelocityConfig) -> None:
        twist = keyboard_state_to_twist(KeyboardState(forward=True, backward=True), cfg)
        assert twist.linear.x == pytest.approx(-cfg.linear_speed)

    def test_strafe_left_and_right_right_wins(self, cfg: VelocityConfig) -> None:
        twist = keyboard_state_to_twist(KeyboardState(strafe_left=True, strafe_right=True), cfg)
        assert twist.linear.y == pytest.approx(-cfg.linear_speed)

    def test_turn_left_and_right_right_wins(self, cfg: VelocityConfig) -> None:
        twist = keyboard_state_to_twist(KeyboardState(turn_left=True, turn_right=True), cfg)
        assert twist.angular.z == pytest.approx(-cfg.angular_speed)

    # --- Speed modifiers ---

    def test_boost_scales_all_axes(self, cfg: VelocityConfig) -> None:
        state = KeyboardState(forward=True, strafe_left=True, turn_left=True, boost=True)
        twist = keyboard_state_to_twist(state, cfg)
        assert twist.linear.x == pytest.approx(cfg.linear_speed * cfg.boost_multiplier)
        assert twist.linear.y == pytest.approx(cfg.linear_speed * cfg.boost_multiplier)
        assert twist.angular.z == pytest.approx(cfg.angular_speed * cfg.boost_multiplier)

    def test_slow_scales_all_axes(self, cfg: VelocityConfig) -> None:
        state = KeyboardState(forward=True, strafe_left=True, turn_left=True, slow=True)
        twist = keyboard_state_to_twist(state, cfg)
        assert twist.linear.x == pytest.approx(cfg.linear_speed * cfg.slow_multiplier)
        assert twist.linear.y == pytest.approx(cfg.linear_speed * cfg.slow_multiplier)
        assert twist.angular.z == pytest.approx(cfg.angular_speed * cfg.slow_multiplier)

    def test_boost_and_slow_boost_wins(self, cfg: VelocityConfig) -> None:
        state = KeyboardState(forward=True, boost=True, slow=True)
        twist = keyboard_state_to_twist(state, cfg)
        assert twist.linear.x == pytest.approx(cfg.linear_speed * cfg.boost_multiplier)

    def test_modifier_without_movement_stays_zero(self, cfg: VelocityConfig) -> None:
        twist = keyboard_state_to_twist(KeyboardState(boost=True), cfg)
        assert twist.linear.x == 0.0
        assert twist.linear.y == 0.0
        assert twist.angular.z == 0.0

    # --- Combined movement ---

    def test_forward_and_turn_left(self, cfg: VelocityConfig) -> None:
        state = KeyboardState(forward=True, turn_left=True)
        twist = keyboard_state_to_twist(state, cfg)
        assert twist.linear.x == pytest.approx(cfg.linear_speed)
        assert twist.angular.z == pytest.approx(cfg.angular_speed)
        assert twist.linear.y == 0.0

    def test_all_positive_directions(self, cfg: VelocityConfig) -> None:
        state = KeyboardState(forward=True, strafe_left=True, turn_left=True)
        twist = keyboard_state_to_twist(state, cfg)
        assert twist.linear.x == pytest.approx(cfg.linear_speed)
        assert twist.linear.y == pytest.approx(cfg.linear_speed)
        assert twist.angular.z == pytest.approx(cfg.angular_speed)

    # --- Custom config ---

    def test_custom_config(self) -> None:
        custom = VelocityConfig(
            linear_speed=1.0,
            angular_speed=2.0,
            boost_multiplier=3.0,
            slow_multiplier=0.1,
        )
        state = KeyboardState(backward=True, turn_right=True, boost=True)
        twist = keyboard_state_to_twist(state, custom)
        assert twist.linear.x == pytest.approx(-3.0)
        assert twist.angular.z == pytest.approx(-6.0)

    # --- Untouched axes stay zero ---

    def test_linear_z_always_zero(self, cfg: VelocityConfig) -> None:
        state = KeyboardState(forward=True, strafe_left=True, turn_left=True, boost=True)
        twist = keyboard_state_to_twist(state, cfg)
        assert twist.linear.z == 0.0

    def test_angular_xy_always_zero(self, cfg: VelocityConfig) -> None:
        state = KeyboardState(forward=True, strafe_left=True, turn_left=True, boost=True)
        twist = keyboard_state_to_twist(state, cfg)
        assert twist.angular.x == 0.0
        assert twist.angular.y == 0.0
