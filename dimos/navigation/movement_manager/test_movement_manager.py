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
from dimos.navigation.movement_manager.cmd_vel_mux_native import (
    CmdVelMuxNative,
    CmdVelMuxNativeConfig,
)
from dimos.navigation.movement_manager.movement_manager import (
    MovementManager,
)


@dataclass
class Captured:
    """Captures messages published by a MovementManager via real subscribers."""

    goal: list = field(default_factory=list)
    way_point: list = field(default_factory=list)


def _attach(module):
    """Subscribe to every Out port; return (captured, unsubscribers)."""
    captured = Captured()
    unsubs = [
        module.goal.subscribe(captured.goal.append),
        module.way_point.subscribe(captured.way_point.append),
    ]
    return captured, unsubs


@pytest.fixture()
def manager_and_captured() -> Generator[tuple[MovementManager, Captured], None, None]:
    module = MovementManager()
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


def test_teleop_cancels_the_goal_with_nan(manager_and_captured):
    """Teleop cancels nav on the laptop side; the rust mux owns the velocity half."""
    manager, captured = manager_and_captured
    manager._on_teleop(_twist(lx=0.3))

    assert len(captured.goal) == 1
    assert math.isnan(captured.goal[0].x)
    assert len(captured.way_point) == 1
    assert math.isnan(captured.way_point[0].x)


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


def test_velocity_ports_moved_to_the_mux(manager_and_captured):
    """The split: velocities are the mux's, clicks and the cancel are the manager's.

    Both keep tele_cmd_vel — one keystroke has to land on both halves.
    """
    manager, _ = manager_and_captured
    assert set(manager.inputs) == {"clicked_point", "tele_cmd_vel"}
    assert set(manager.outputs) == {"goal", "way_point"}

    mux = CmdVelMuxNative()
    try:
        assert set(mux.inputs) == {"nav_cmd_vel", "tele_cmd_vel"}
        assert set(mux.outputs) == {"cmd_vel", "stop_movement"}
    finally:
        mux._close_module()


def test_mux_config_crosses_the_boundary_whole():
    """Every rust config field, every time — a missing key is a startup error."""
    assert set(CmdVelMuxNativeConfig().to_config_dict()) == {
        "tele_cooldown_sec",
        "tele_scale_linear",
        "tele_scale_angular",
        "nav_stale_s",
    }
