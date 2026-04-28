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

"""Tests for TUIControlModule."""

import threading

import pytest

from dimos.navigation.nav_stack.modules.tui_control.tui_control import TUIControlModule


class TestTUIControl:
    """Test TUI controller key handling and output."""

    @pytest.fixture(autouse=True)
    def _create_module(self):
        self.module = TUIControlModule(max_speed=2.0, max_yaw_rate=1.5)
        # _lock is normally created by start(); these unit tests poke
        # _handle_key() directly without starting the publish/input
        # threads, so initialise the lock by hand.
        self.module._lock = threading.Lock()
        yield
        self.module.stop()

    def test_initial_state_zero(self):
        """All velocities should start at zero."""
        module = self.module
        assert module._fwd == 0.0
        assert module._left == 0.0
        assert module._yaw == 0.0

    def test_forward_key(self):
        """'w' key should set forward motion."""
        module = self.module
        module._handle_key("w")
        assert module._fwd == 1.0
        assert module._left == 0.0
        assert module._yaw == 0.0

    def test_backward_key(self):
        """'s' key should set backward motion."""
        module = self.module
        module._handle_key("s")
        assert module._fwd == -1.0

    def test_strafe_left_key(self):
        """'a' key should set left strafe."""
        module = self.module
        module._handle_key("a")
        assert module._left == 1.0
        assert module._fwd == 0.0

    def test_strafe_right_key(self):
        """'d' key should set right strafe."""
        module = self.module
        module._handle_key("d")
        assert module._left == -1.0

    def test_rotate_left_key(self):
        """'q' key should set left rotation."""
        module = self.module
        module._handle_key("q")
        assert module._yaw == 1.0
        assert module._fwd == 0.0
        assert module._left == 0.0

    def test_rotate_right_key(self):
        """'e' key should set right rotation."""
        module = self.module
        module._handle_key("e")
        assert module._yaw == -1.0

    def test_stop_key(self):
        """Space should stop all motion."""
        module = self.module
        module._handle_key("w")
        assert module._fwd == 1.0
        module._handle_key(" ")
        assert module._fwd == 0.0
        assert module._left == 0.0
        assert module._yaw == 0.0

    def test_speed_increase(self):
        """'+' key should increase speed scale."""
        module = self.module
        # First decrease from the default (1.0) so there is room to increase
        module._handle_key("-")
        lowered_scale = module._speed_scale
        module._handle_key("+")
        assert module._speed_scale > lowered_scale

    def test_speed_decrease(self):
        """'-' key should decrease speed scale."""
        module = self.module
        module._handle_key("-")
        assert module._speed_scale < 1.0

    def test_speed_scale_bounds(self):
        """Speed scale should be bounded [0.1, 1.0]."""
        module = self.module
        # Try to go below minimum
        for _ in range(20):
            module._handle_key("-")
        assert module._speed_scale >= 0.1

        # Try to go above maximum
        for _ in range(20):
            module._handle_key("+")
        assert module._speed_scale <= 1.0

    def test_waypoint_publish(self):
        """send_waypoint should publish a PointStamped message."""
        module = self.module

        results = []
        unsub = module.way_point.subscribe(lambda msg: results.append(msg))
        try:
            module.send_waypoint(5.0, 10.0, 0.0)

            assert len(results) == 1
            assert results[0].x == 5.0
            assert results[0].y == 10.0
            assert results[0].frame_id == "map"
        finally:
            unsub()
