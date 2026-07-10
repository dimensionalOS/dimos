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

"""``DanHolonomicTC`` module integration: cancel inputs, path hot-swap, arrival."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field
import math
import time
from typing import Any, Literal

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
import pytest

from dimos.core.stream import Stream, Transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.base import NavigationState
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC

_CancelVia = Literal["empty_path", "stop_movement"]


class _DirectTransport(Transport):  # type: ignore[type-arg]
    """Synchronous in-process transport so ``start()`` can wire the inputs."""

    def __init__(self) -> None:
        self._subscribers: list[Callable[[Any], Any]] = []

    def broadcast(self, _selfstream: Any, value: Any) -> None:
        for callback in list(self._subscribers):
            callback(value)

    def subscribe(
        self, callback: Callable[[Any], Any], _selfstream: Stream[Any] | None = None
    ) -> Callable[[], None]:
        self._subscribers.append(callback)

        def _unsubscribe() -> None:
            if callback in self._subscribers:
                self._subscribers.remove(callback)

        return _unsubscribe

    def start(self) -> None: ...

    def stop(self) -> None:
        self._subscribers.clear()


@dataclass
class _Captured:
    cmd_vel: list[Twist] = field(default_factory=list)
    goal_reached: list[Bool] = field(default_factory=list)


def _yaw_quaternion(yaw_rad: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def _odom(x: float, y: float, yaw_rad: float, *, ts: float = 1.0) -> PoseStamped:
    return PoseStamped(
        ts=ts,
        frame_id="map",
        position=[x, y, 0.0],
        orientation=_yaw_quaternion(yaw_rad),
    )


def _path_from_points(points: list[tuple[float, float]]) -> Path:
    poses: list[PoseStamped] = []
    for index, point in enumerate(points):
        if index + 1 < len(points):
            next_point = points[index + 1]
            yaw = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
        else:
            prev_point = points[index - 1]
            yaw = math.atan2(point[1] - prev_point[1], point[0] - prev_point[0])
        poses.append(
            PoseStamped(
                ts=1.0,
                frame_id="map",
                position=[point[0], point[1], 0.0],
                orientation=_yaw_quaternion(yaw),
            )
        )
    return Path(frame_id="map", poses=poses)


def _is_zero_twist(cmd: Twist) -> bool:
    return (
        abs(float(cmd.linear.x)) < 1e-9
        and abs(float(cmd.linear.y)) < 1e-9
        and abs(float(cmd.angular.z)) < 1e-9
    )


def _wait_until(predicate: Callable[[], bool], timeout: float = 2.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


class _ModuleHarness:
    def __init__(
        self, module: DanHolonomicTC, captured: _Captured, unsubs: list[Callable[[], None]]
    ) -> None:
        self.module = module
        self.captured = captured
        self._unsubs = unsubs

    def feed_odom(self, x: float, y: float, yaw_rad: float, *, ts: float = 1.0) -> None:
        self.module.odom.transport.broadcast(None, _odom(x, y, yaw_rad, ts=ts))

    def feed_path(self, path: Path) -> None:
        self.module.path.transport.broadcast(None, path)

    def feed_empty_path(self) -> None:
        self.module.path.transport.broadcast(None, Path(frame_id="map", poses=[]))

    def feed_stop(self, value: bool = True) -> None:
        self.module.stop_movement.transport.broadcast(None, Bool(value))

    def close(self) -> None:
        for unsub in self._unsubs:
            unsub()
        self.module.stop()


@contextmanager
def _running_module(**config: Any) -> Iterator[_ModuleHarness]:
    module = DanHolonomicTC(**config)
    module.path.transport = _DirectTransport()
    module.odom.transport = _DirectTransport()
    module.stop_movement.transport = _DirectTransport()
    captured = _Captured()
    unsubs = [
        module.nav_cmd_vel.subscribe(captured.cmd_vel.append),
        module.goal_reached.subscribe(captured.goal_reached.append),
    ]
    module.start()
    harness = _ModuleHarness(module, captured, unsubs)
    try:
        yield harness
    finally:
        harness.close()


def _assert_route_cleared(h: _ModuleHarness) -> None:
    assert _wait_until(lambda: h.module.get_state() == NavigationState.IDLE)
    assert _wait_until(lambda: bool(h.captured.cmd_vel) and _is_zero_twist(h.captured.cmd_vel[-1]))
    assert not h.captured.goal_reached


@pytest.mark.parametrize("cancel_via", ["empty_path", "stop_movement"])
def test_cancel_publishes_zero_twist_and_clears_route(cancel_via: _CancelVia) -> None:
    with _running_module(goal_tolerance=0.2) as h:
        h.feed_odom(0.0, 0.0, 0.0)
        h.feed_path(_path_from_points([(0.0, 0.0), (2.0, 0.0)]))

        if cancel_via == "empty_path":
            h.feed_empty_path()
        else:
            h.feed_stop(True)

        _assert_route_cleared(h)


def test_hot_update_path_swaps_route_to_new_goal() -> None:
    # Robot at (1, 0): old goal (2, 0) is still out of tolerance; after the
    # swap the terminus is (1, 0), so arrival proves the new route took effect.
    with _running_module(goal_tolerance=0.2) as h:
        h.feed_odom(1.0, 0.0, 0.0)
        h.feed_path(_path_from_points([(0.0, 0.0), (2.0, 0.0)]))
        assert not _wait_until(lambda: bool(h.captured.goal_reached), timeout=0.2)

        h.feed_path(_path_from_points([(0.0, 0.0), (1.0, 0.0)]))

        assert _wait_until(lambda: bool(h.captured.goal_reached))
        assert h.captured.goal_reached[-1].data is True


def test_arrival_publishes_goal_reached_without_final_spin() -> None:
    # Odom on the goal with misaligned heading; align_goal_yaw=False must report
    # arrival on position alone, never spinning to align the final yaw.
    with _running_module(goal_tolerance=0.2, align_goal_yaw=False) as h:
        h.feed_odom(1.0, 0.0, 1.2)
        h.feed_path(_path_from_points([(0.0, 0.0), (1.0, 0.0)]))

        assert _wait_until(lambda: bool(h.captured.goal_reached))
        assert h.captured.goal_reached[-1].data is True
        assert h.captured.cmd_vel
        assert all(abs(float(cmd.angular.z)) < 1e-6 for cmd in h.captured.cmd_vel)
