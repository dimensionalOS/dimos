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

from dataclasses import dataclass, field
import math
import time

import pytest

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.nav_stack.modules.movement_manager.movement_manager import (
    MovementManager,
)


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
def manager_and_captured():
    """Yield a MovementManager and a Captured collector for its outputs."""
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


def test_nav_resumes_after_cooldown(manager_and_captured):
    """After the cooldown expires, nav commands pass through again."""
    manager, captured = manager_and_captured
    manager.config.tele_cooldown_sec = 0.05
    manager._on_teleop(_twist(lx=0.3))
    time.sleep(0.1)
    cmd_count_before = len(captured.cmd_vel)

    manager._on_nav(_twist(lx=0.9))
    assert len(captured.cmd_vel) == cmd_count_before + 1


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


def test_tele_cmd_vel_scaling():
    """tele_cmd_vel_scaling multiplies each teleop twist component independently."""
    scaling = Twist(Vector3(0.5, 2.0, 0.0), Vector3(1.0, 1.0, 0.25))
    module = MovementManager(tele_cooldown_sec=10.0, tele_cmd_vel_scaling=scaling)
    captured, unsubs = _attach(module)
    try:
        module._on_teleop(Twist(Vector3(1, 1, 1), Vector3(1, 1, 1)))

        assert len(captured.cmd_vel) == 1
        published = captured.cmd_vel[0]
        assert published.linear.x == pytest.approx(0.5)
        assert published.linear.y == pytest.approx(2.0)
        assert published.linear.z == pytest.approx(0.0)
        assert published.angular.z == pytest.approx(0.25)
    finally:
        for unsub in unsubs:
            unsub()
        module._close_module()
