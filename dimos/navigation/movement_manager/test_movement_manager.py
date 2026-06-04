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

from collections.abc import Generator
from dataclasses import dataclass, field
import math
import time

import pytest

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.movement_manager.movement_manager import (
    MovementManager,
)

_COOLDOWN_SLEEP_SEC = 0.1


@dataclass
class Captured:
    """Captures messages published by a MovementManager via real subscribers."""

    cmd_vel: list = field(default_factory=list)
    stop_movement: list = field(default_factory=list)
    goal: list = field(default_factory=list)
    way_point: list = field(default_factory=list)


def _attach(module):
    """Subscribe to every Out port; return (captured, unsubscribers)."""
    captured = Captured()
    unsubs = [
        module.cmd_vel.subscribe(captured.cmd_vel.append),
        module.stop_movement.subscribe(captured.stop_movement.append),
        module.goal.subscribe(captured.goal.append),
        module.way_point.subscribe(captured.way_point.append),
    ]
    return captured, unsubs


@pytest.fixture()
def manager_and_captured() -> Generator[tuple[MovementManager, Captured], None, None]:
    module = MovementManager(tele_cooldown_sec=0.1)
    captured, unsubs = _attach(module)
    try:
        yield module, captured
    finally:
        for unsub in unsubs:
            unsub()
        module._close_module()


def _twist(lx=0.0):
    return Twist(linear=Vector3(lx, 0, 0), angular=Vector3(0, 0, 0))


def _click(x=1.0, y=2.0, z=0.0):
    return PointStamped(ts=time.time(), frame_id="map", x=x, y=y, z=z)


def test_teleop_suppresses_nav_and_cancels_goal(manager_and_captured):
    """Teleop arriving should suppress nav, publish stop_movement, and cancel the goal with NaN."""
    manager, captured = manager_and_captured
    manager.config.tele_cooldown_sec = 10.0
    manager._on_teleop(_twist(lx=0.3))

    cmd_count_after_teleop = len(captured.cmd_vel)
    manager._on_nav(_twist(lx=0.9))
    # Nav was suppressed: no new cmd_vel
    assert len(captured.cmd_vel) == cmd_count_after_teleop

    # stop_movement fired
    assert len(captured.stop_movement) == 1

    # Goal cancelled with NaN
    assert len(captured.goal) == 1
    assert math.isnan(captured.goal[0].x)


def test_nav_remains_suppressed_after_teleop_until_new_click(manager_and_captured):
    """Teleop cancels the active goal; nav may only resume after a fresh click."""
    manager, captured = manager_and_captured
    manager.config.tele_cooldown_sec = 0.05
    manager._on_click(_click())
    manager._on_teleop(_twist(lx=0.3))
    time.sleep(_COOLDOWN_SLEEP_SEC)
    cmd_count_before = len(captured.cmd_vel)

    manager._on_nav(_twist(lx=0.9))
    assert len(captured.cmd_vel) == cmd_count_before

    manager._on_click(_click(x=3.0, y=4.0))
    manager._on_nav(_twist(lx=0.9))
    assert len(captured.cmd_vel) == cmd_count_before + 1


def test_idle_zero_teleop_is_ignored(manager_and_captured):
    """Viewer-connect zero twists should not cancel goals or publish stop commands."""
    manager, captured = manager_and_captured

    manager._on_teleop(Twist.zero())

    assert captured.cmd_vel == []
    assert captured.stop_movement == []
    assert captured.goal == []
    assert captured.way_point == []


def test_zero_teleop_cancels_active_goal_and_latches_nav(manager_and_captured):
    """The viewer stop button sends zero teleop; while navigating it is a hard cancel."""
    manager, captured = manager_and_captured
    manager.config.tele_cooldown_sec = 0.05

    manager._on_click(_click())
    manager._on_teleop(Twist.zero())

    assert captured.cmd_vel[-1].is_zero()
    assert len(captured.stop_movement) == 1
    assert math.isnan(captured.goal[-1].x)

    cmd_count_before = len(captured.cmd_vel)
    time.sleep(_COOLDOWN_SLEEP_SEC)
    manager._on_nav(_twist(lx=0.9))
    assert len(captured.cmd_vel) == cmd_count_before


def test_valid_click_publishes_goal(manager_and_captured):
    """A valid click should publish to both goal and way_point."""
    manager, captured = manager_and_captured
    click = _click(x=5.0, y=3.0, z=0.1)
    manager._on_click(click)
    assert captured.goal == [click]
    assert captured.way_point == [click]


def test_invalid_clicks_rejected(manager_and_captured):
    """NaN, Inf, and out-of-range clicks should not publish."""
    manager, captured = manager_and_captured
    for bad_click in [
        _click(x=float("nan")),
        _click(x=float("inf")),
        _click(x=600.0),
    ]:
        manager._on_click(bad_click)
    assert captured.goal == []


def test_tele_cmd_vel_scaling(manager_and_captured):
    """tele_cmd_vel_scaling multiplies each teleop twist component independently."""
    manager, captured = manager_and_captured
    scaling = Twist(Vector3(0.5, 2.0, 0.0), Vector3(1.0, 1.0, 0.25))
    manager.config.tele_cmd_vel_scaling = scaling
    manager.config.tele_cooldown_sec = 10.0

    manager._on_teleop(Twist(Vector3(1, 1, 1), Vector3(1, 1, 1)))

    assert len(captured.cmd_vel) == 1
    published = captured.cmd_vel[0]
    assert published.linear.x == pytest.approx(0.5)
    assert published.linear.y == pytest.approx(2.0)
    assert published.linear.z == pytest.approx(0.0)
    assert published.angular.z == pytest.approx(0.25)
