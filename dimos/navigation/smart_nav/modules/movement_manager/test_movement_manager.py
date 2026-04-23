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

"""Tests for MovementManager: click-to-goal + teleop/nav velocity mux."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, cast
from unittest.mock import MagicMock, patch

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.smart_nav.modules.movement_manager.movement_manager import (
    MovementManager,
    MovementManagerConfig,
)


def _make_mgr(cooldown: float = 0.1) -> Any:
    """Build a MovementManager with mocked output streams."""
    with patch.object(MovementManager, "__init__", lambda self: None):
        mgr = cast("Any", MovementManager.__new__(MovementManager))
    mgr.config = MovementManagerConfig(tele_cooldown_sec=cooldown)
    mgr._teleop_active = False
    mgr._lock = threading.Lock()
    mgr._last_teleop_time = 0.0
    mgr._robot_x = 0.0
    mgr._robot_y = 0.0
    mgr._robot_z = 0.0
    mgr.cmd_vel = MagicMock()
    mgr.stop_movement = MagicMock()
    mgr.goal = MagicMock()
    mgr.way_point = MagicMock()
    return mgr


def _twist(lx: float = 0.0, az: float = 0.0) -> Twist:
    return Twist(linear=Vector3(lx, 0, 0), angular=Vector3(0, 0, az))


def _click(x: float = 1.0, y: float = 2.0, z: float = 0.0) -> PointStamped:
    return PointStamped(ts=time.time(), frame_id="map", x=x, y=y, z=z)


# ── Nav passthrough ───────────────────────────────────────────────────────


class TestNavPassthrough:
    def test_nav_passes_through_when_no_teleop(self) -> None:
        mgr = _make_mgr()
        mgr._on_nav(_twist(lx=0.5))
        mgr.cmd_vel.publish.assert_called_once()
        mgr.stop_movement.publish.assert_not_called()

    def test_nav_suppressed_while_teleop_active(self) -> None:
        mgr = _make_mgr(cooldown=10.0)
        mgr._on_teleop(_twist(lx=0.3))
        mgr.cmd_vel.publish.reset_mock()

        mgr._on_nav(_twist(lx=0.9))
        mgr.cmd_vel.publish.assert_not_called()

    def test_nav_resumes_after_cooldown(self) -> None:
        mgr = _make_mgr(cooldown=0.05)
        mgr._on_teleop(_twist(lx=0.3))
        time.sleep(0.1)
        mgr.cmd_vel.publish.reset_mock()

        mgr._on_nav(_twist(lx=0.9))
        mgr.cmd_vel.publish.assert_called_once()


# ── Teleop mux behaviour ───────────────────────────────────────────────────


class TestTeleop:
    def test_first_teleop_publishes_stop_movement(self) -> None:
        mgr = _make_mgr()
        mgr._on_teleop(_twist(lx=0.3))
        mgr.stop_movement.publish.assert_called_once()

    def test_subsequent_teleop_does_not_republish_stop_movement(self) -> None:
        mgr = _make_mgr(cooldown=10.0)
        mgr._on_teleop(_twist(lx=0.3))
        mgr._on_teleop(_twist(lx=0.4))
        mgr._on_teleop(_twist(lx=0.5))
        assert mgr.stop_movement.publish.call_count == 1

    def test_teleop_publishes_to_cmd_vel(self) -> None:
        mgr = _make_mgr()
        mgr._on_teleop(_twist(lx=0.5, az=0.1))
        mgr.cmd_vel.publish.assert_called_once()

    def test_teleop_forwards_msg_unchanged(self) -> None:
        mgr = _make_mgr()
        msg = _twist(lx=0.7)
        mgr._on_teleop(msg)
        assert mgr.cmd_vel.publish.call_args[0][0] is msg

    def test_first_teleop_cancels_goal(self) -> None:
        """MovementManager publishes NaN goal to cancel active navigation."""
        mgr = _make_mgr()
        mgr._on_teleop(_twist(lx=0.3))
        assert mgr.goal.publish.call_count == 1
        cancel_msg = mgr.goal.publish.call_args[0][0]
        assert math.isnan(cancel_msg.x)
        assert math.isnan(cancel_msg.y)
        assert math.isnan(cancel_msg.z)

    def test_teleop_reactivates_after_cooldown(self) -> None:
        """After cooldown expires and nav resumes, new teleop fires stop again."""
        mgr = _make_mgr(cooldown=0.05)
        mgr._on_teleop(_twist(lx=0.3))
        assert mgr.stop_movement.publish.call_count == 1

        time.sleep(0.1)
        # Nav message clears teleop_active after cooldown
        mgr._on_nav(_twist(lx=0.1))

        # New teleop should fire stop_movement again
        mgr._on_teleop(_twist(lx=0.4))
        assert mgr.stop_movement.publish.call_count == 2


# ── Click-to-goal ──────────────────────────────────────────────────────────


class TestClickToGoal:
    def test_valid_click_publishes_goal_and_waypoint(self) -> None:
        mgr = _make_mgr()
        click = _click(x=5.0, y=3.0, z=0.1)
        mgr._on_click(click)
        mgr.goal.publish.assert_called_once_with(click)
        mgr.way_point.publish.assert_called_once_with(click)

    def test_nan_click_rejected(self) -> None:
        mgr = _make_mgr()
        mgr._on_click(_click(x=float("nan"), y=1.0, z=0.0))
        mgr.goal.publish.assert_not_called()

    def test_inf_click_rejected(self) -> None:
        mgr = _make_mgr()
        mgr._on_click(_click(x=float("inf"), y=1.0, z=0.0))
        mgr.goal.publish.assert_not_called()

    def test_out_of_range_click_rejected(self) -> None:
        mgr = _make_mgr()
        mgr._on_click(_click(x=600.0, y=1.0, z=0.0))
        mgr.goal.publish.assert_not_called()

    def test_boundary_click_accepted(self) -> None:
        mgr = _make_mgr()
        mgr._on_click(_click(x=500.0, y=500.0, z=50.0))
        mgr.goal.publish.assert_called_once()


# ── Config defaults ────────────────────────────────────────────────────────


class TestConfigDefaults:
    def test_cooldown_default(self) -> None:
        config = MovementManagerConfig()
        assert config.tele_cooldown_sec == 1.0
