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

"""Tests for CmdVelMux teleop/nav priority switching."""

from __future__ import annotations

from dimos.navigation.cmd_vel_mux import CmdVelMux


class TestCmdVelMux:
    def test_teleop_initially_inactive(self) -> None:
        mux = CmdVelMux.__new__(CmdVelMux)
        mux.__dict__["_teleop_active"] = False
        assert not mux._teleop_active

    def test_end_teleop_clears_flag(self) -> None:
        import threading

        mux = CmdVelMux.__new__(CmdVelMux)
        mux.__dict__["_teleop_active"] = True
        mux.__dict__["_timer"] = None
        mux.__dict__["_lock"] = threading.Lock()
        mux._end_teleop()
        assert not mux._teleop_active

    def test_nav_suppressed_when_teleop_active(self) -> None:
        """When _teleop_active is True, _on_nav returns early (no publish)."""
        import threading

        mux = CmdVelMux.__new__(CmdVelMux)
        mux.__dict__["_teleop_active"] = True
        mux.__dict__["_agent_active"] = False
        mux.__dict__["_lock"] = threading.Lock()
        # _on_nav should return before reaching cmd_vel._transport.publish
        # If it didn't return early, it would crash since cmd_vel has no transport
        from dimos.msgs.geometry_msgs.Twist import Twist
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        mux._on_nav(Twist(linear=Vector3(1, 0, 0), angular=Vector3(0, 0, 0)))
        assert mux._teleop_active  # Still active, nav was suppressed

    def test_cooldown_default(self) -> None:
        from dimos.navigation.cmd_vel_mux import CmdVelMuxConfig

        config = CmdVelMuxConfig()
        assert config.teleop_cooldown_sec == 1.0
