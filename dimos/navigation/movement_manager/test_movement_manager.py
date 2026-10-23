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
from contextlib import ExitStack
from dataclasses import dataclass, field
import math
from threading import Event

from dimos_cdrgen.registry import decode, encode
import pytest

from dimos.core.transport import LCMTransport
from dimos.message_utils.time import time_from_nanoseconds
from dimos.messages.geometry_msgs.msg import Point, PointStamped, Twist, Vector3
from dimos.messages.std_msgs.msg import Bool, Header
from dimos.navigation.movement_manager import movement_manager as movement_module
from dimos.navigation.movement_manager.movement_manager import (
    MovementManager,
)
from dimos.protocol.rpc.pubsubrpc import LCMRPC


class MemoryRPC(LCMRPC):
    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


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
    module = MovementManager(tele_cooldown_sec=0.1, rpc_transport=MemoryRPC)
    captured, unsubs = _attach(module)
    try:
        yield module, captured
    finally:
        for unsub in unsubs:
            unsub()
        module.stop()


def _twist(lx=0.0):
    return Twist(linear=Vector3(lx, 0, 0), angular=Vector3(0, 0, 0))


def _click(x=1.0, y=2.0, z=0.0):
    return PointStamped(
        header=Header(frame_id="map", stamp=time_from_nanoseconds(1700000000123456789)),
        point=Point(x=x, y=y, z=z),
    )


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
    assert math.isnan(captured.goal[0].point.x)


def test_nav_resumes_after_cooldown(manager_and_captured, monkeypatch):
    """After the cooldown expires, nav commands pass through again."""
    manager, captured = manager_and_captured
    now = [0.0]
    monkeypatch.setattr(movement_module, "monotonic", lambda: now[0])
    manager.config.tele_cooldown_sec = 0.05
    manager._on_teleop(_twist(lx=0.3))
    now[0] = 0.05
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


@pytest.mark.parametrize("stamp", [0, -1, 1700000000123456789])
def test_cancel_goal_preserves_exact_wall_time_and_existing_nan_fallback(
    manager_and_captured, monkeypatch, stamp
):
    manager, captured = manager_and_captured
    monkeypatch.setattr(movement_module, "time_ns", lambda: stamp)
    manager._on_teleop(_twist(0.3))
    assert captured.stop_movement == [Bool(data=True)]
    cancel = decode(encode(captured.goal[0]), PointStamped)
    assert cancel.header == Header(frame_id="map", stamp=time_from_nanoseconds(stamp))
    assert all(math.isnan(value) for value in (cancel.point.x, cancel.point.y, cancel.point.z))


@pytest.mark.parametrize("position", [(500.0, -500.0, 50.0), (-500.0, 500.0, -50.0)])
def test_click_limit_boundaries_keep_exact_source_header(manager_and_captured, position):
    manager, captured = manager_and_captured
    click = _click(*position)
    manager._on_click(click)
    assert captured.goal == [click] and captured.way_point == [click]
    assert decode(encode(captured.goal[0]), PointStamped) == click


@pytest.mark.parametrize(
    "position",
    [
        (0.0, 501.0, 0.0),
        (0.0, 0.0, 51.0),
        (0.0, 0.0, -51.0),
        (0.0, float("inf"), 0.0),
        (0.0, 0.0, float("nan")),
    ],
)
def test_other_invalid_click_components_are_rejected(manager_and_captured, position):
    manager, captured = manager_and_captured
    manager._on_click(_click(*position))
    assert captured.goal == [] and captured.way_point == []


def test_actual_cdr_movement_ports_keep_teleop_priority_and_resume_nav_at_cooldown(monkeypatch):
    now = [0.0]
    monkeypatch.setattr(movement_module, "monotonic", lambda: now[0])
    monkeypatch.setattr(movement_module, "time_ns", lambda: 1700000000123456789)
    with ExitStack() as cleanup:
        manager = MovementManager(rpc_transport=MemoryRPC, tele_cooldown_sec=1.0)
        wires = {}
        for name in [
            "clicked_point",
            "nav_cmd_vel",
            "tele_cmd_vel",
            "goal",
            "way_point",
            "cmd_vel",
            "stop_movement",
        ]:
            port = getattr(manager, name)
            wire = LCMTransport("movement_" + name, port.type, url="memq://")
            port.transport = wire
            wires[name] = wire
            cleanup.callback(wire.stop)
        cleanup.callback(manager.stop)
        commands = []
        cancellations = []
        stops = []
        received = Event()
        nav_ack = Event()
        cancelled = Event()
        stopped = Event()
        on_nav = manager._on_nav

        def handle_nav(value):
            on_nav(value)
            nav_ack.set()

        def cancel(value):
            cancellations.append(value)
            cancelled.set()

        def stop(value):
            stops.append(value)
            stopped.set()

        monkeypatch.setattr(manager, "_on_nav", handle_nav)

        def receive(value):
            commands.append(value)
            received.set()

        cleanup.callback(wires["cmd_vel"].subscribe(receive))
        cleanup.callback(wires["goal"].subscribe(cancel))
        cleanup.callback(wires["stop_movement"].subscribe(stop))
        manager.start()
        wires["tele_cmd_vel"].publish(_twist(0.3))
        assert received.wait(5)
        assert cancelled.wait(5) and stopped.wait(5)
        assert commands == [_twist(0.3)]
        assert stops == [Bool(data=True)]
        assert cancellations[0].header.stamp == time_from_nanoseconds(1700000000123456789)
        now[0] = 0.5
        wires["nav_cmd_vel"].publish(_twist(0.9))
        assert nav_ack.wait(5)
        assert commands == [_twist(0.3)]
        received.clear()
        now[0] = 1.0
        wires["nav_cmd_vel"].publish(_twist(0.9))
        assert received.wait(5)
        assert commands == [_twist(0.3), _twist(0.9)]
