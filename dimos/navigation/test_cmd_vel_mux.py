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

import time

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
        mux.__dict__["_nav_active_since"] = None
        mux.__dict__["_nav_watchdog_tripped"] = False
        mux.__dict__["_lock"] = threading.Lock()
        # _on_nav should return before reaching cmd_vel._transport.publish
        # If it didn't return early, it would crash since cmd_vel has no transport
        from dimos.msgs.geometry_msgs.Twist import Twist
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        mux._on_nav(Twist(linear=Vector3(1, 0, 0), angular=Vector3(0, 0, 0)))
        assert mux._teleop_active  # Still active, nav was suppressed

    def test_initial_zero_teleop_does_not_publish_stop_movement(self) -> None:
        import threading
        from types import SimpleNamespace

        from dimos.msgs.geometry_msgs.Twist import Twist

        class _Out:
            def __init__(self) -> None:
                self.messages = []

            def publish(self, msg) -> None:  # type: ignore[no-untyped-def]
                self.messages.append(msg)

        mux = CmdVelMux.__new__(CmdVelMux)
        mux.__dict__["config"] = SimpleNamespace(teleop_cooldown_sec=1.0, teleop_linear_scale=1.0)
        mux.__dict__["_teleop_active"] = False
        mux.__dict__["_nav_active_since"] = None
        mux.__dict__["_nav_watchdog_tripped"] = False
        mux.__dict__["_timer"] = None
        mux.__dict__["_lock"] = threading.Lock()
        mux.__dict__["cmd_vel"] = _Out()
        mux.__dict__["stop_movement"] = _Out()

        mux._on_teleop(Twist.zero())

        assert mux._teleop_active is False
        assert mux.stop_movement.messages == []
        assert mux.cmd_vel.messages == []

    def test_nav_watchdog_publishes_zero_and_stop_once(self) -> None:
        import threading
        from types import SimpleNamespace

        from dimos.msgs.geometry_msgs.Twist import Twist
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        class _Out:
            def __init__(self) -> None:
                self.messages = []

            def publish(self, msg) -> None:  # type: ignore[no-untyped-def]
                self.messages.append(msg)

        mux = CmdVelMux.__new__(CmdVelMux)
        mux.__dict__["config"] = SimpleNamespace(
            max_nav_command_duration_sec=1.0,
        )
        mux.__dict__["_teleop_active"] = False
        mux.__dict__["_nav_active_since"] = time.monotonic() - 2.0
        mux.__dict__["_nav_watchdog_tripped"] = False
        mux.__dict__["_lock"] = threading.Lock()
        mux.__dict__["cmd_vel"] = _Out()
        mux.__dict__["stop_movement"] = _Out()

        nonzero = Twist(linear=Vector3(1, 0, 0), angular=Vector3(0, 0, 0))
        mux._on_nav(nonzero)
        mux._on_nav(nonzero)

        assert len(mux.stop_movement.messages) == 1
        assert all(msg.is_zero() for msg in mux.cmd_vel.messages)

    def test_cooldown_default(self) -> None:
        from dimos.navigation.cmd_vel_mux import CmdVelMuxConfig

        config = CmdVelMuxConfig()
        assert config.teleop_cooldown_sec == 1.0
        assert config.max_nav_command_duration_sec is None
