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
import time
from unittest.mock import MagicMock

import pytest

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.smart_nav.modules.movement_manager.movement_manager import (
    MovementManager,
)


@pytest.fixture()
def manager() -> MovementManager:
    """Create a real MovementManager and mock the publish methods on its output streams."""
    module = MovementManager(tele_cooldown_sec=0.1)
    module.cmd_vel.publish = MagicMock()
    module.stop_movement.publish = MagicMock()
    module.goal.publish = MagicMock()
    module.way_point.publish = MagicMock()
    yield module
    module._close_module()


def _twist(lx: float = 0.0) -> Twist:
    return Twist(linear=Vector3(lx, 0, 0), angular=Vector3(0, 0, 0))


def _click(x: float = 1.0, y: float = 2.0, z: float = 0.0) -> PointStamped:
    return PointStamped(ts=time.time(), frame_id="map", x=x, y=y, z=z)


def test_teleop_suppresses_nav_and_cancels_goal(manager: MovementManager) -> None:
    """Teleop arriving should suppress nav, publish stop_movement, and cancel the goal with NaN."""
    manager.config.tele_cooldown_sec = 10.0
    manager._on_teleop(_twist(lx=0.3))

    # Nav is suppressed
    manager.cmd_vel.publish.reset_mock()  # type: ignore[union-attr]
    manager._on_nav(_twist(lx=0.9))
    manager.cmd_vel.publish.assert_not_called()  # type: ignore[union-attr]

    # stop_movement fired
    manager.stop_movement.publish.assert_called_once()  # type: ignore[union-attr]

    # Goal cancelled with NaN
    cancel_msg = manager.goal.publish.call_args[0][0]  # type: ignore[union-attr]
    assert math.isnan(cancel_msg.x)


def test_nav_resumes_after_cooldown(manager: MovementManager) -> None:
    """After the cooldown expires, nav commands pass through again."""
    manager.config.tele_cooldown_sec = 0.05
    manager._on_teleop(_twist(lx=0.3))
    time.sleep(0.1)
    manager.cmd_vel.publish.reset_mock()  # type: ignore[union-attr]

    manager._on_nav(_twist(lx=0.9))
    manager.cmd_vel.publish.assert_called_once()  # type: ignore[union-attr]


def test_valid_click_publishes_goal(manager: MovementManager) -> None:
    """A valid click should publish to both goal and way_point."""
    click = _click(x=5.0, y=3.0, z=0.1)
    manager._on_click(click)
    manager.goal.publish.assert_called_once_with(click)  # type: ignore[union-attr]
    manager.way_point.publish.assert_called_once_with(click)  # type: ignore[union-attr]


def test_invalid_clicks_rejected(manager: MovementManager) -> None:
    """NaN, Inf, and out-of-range clicks should not publish."""
    for bad_click in [
        _click(x=float("nan")),
        _click(x=float("inf")),
        _click(x=600.0),
    ]:
        manager._on_click(bad_click)
    manager.goal.publish.assert_not_called()  # type: ignore[union-attr]


def test_tele_cmd_vel_scaling() -> None:
    """tele_cmd_vel_scaling multiplies each teleop twist component independently."""
    scaling = Twist(Vector3(0.5, 2.0, 0.0), Vector3(1.0, 1.0, 0.25))
    module = MovementManager(tele_cooldown_sec=10.0, tele_cmd_vel_scaling=scaling)
    module.cmd_vel.publish = MagicMock()
    module.stop_movement.publish = MagicMock()
    module.goal.publish = MagicMock()
    module.way_point.publish = MagicMock()

    module._on_teleop(Twist(Vector3(1, 1, 1), Vector3(1, 1, 1)))

    published = module.cmd_vel.publish.call_args[0][0]  # type: ignore[union-attr]
    assert published.linear.x == pytest.approx(0.5)
    assert published.linear.y == pytest.approx(2.0)
    assert published.linear.z == pytest.approx(0.0)
    assert published.angular.z == pytest.approx(0.25)
    module._close_module()
