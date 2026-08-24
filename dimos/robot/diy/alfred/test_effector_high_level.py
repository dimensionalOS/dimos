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

import asyncio
import math
import sys
import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.diy.alfred import effector_high_level
from dimos.robot.diy.alfred.effector_high_level import (
    AlfredHighLevel,
    _rpc_result,
    _stop_and_close,
)


class Future:
    def __init__(self, value: Any = None, error: Exception | None = None) -> None:
        self._value = value
        self._error = error

    def done(self) -> bool:
        return True

    def result(self, timeout: float | None = None) -> Any:
        if self._error is not None:
            raise self._error
        return self._value


class Client:
    def __init__(self, *readings: Any, stop: asyncio.Event | None = None) -> None:
        self._readings = list(readings)
        self._stop = stop
        self.sent: list[Any] = []
        self.closed = False

    def get_odometry(self, request: Any) -> Future:
        if not self._readings:
            return Future(reading(0.0, 0.0, 0.0))
        answer = self._readings.pop(0)
        if not self._readings and self._stop is not None:
            self._stop.set()
        if isinstance(answer, Exception):
            return Future(error=answer)
        return Future(answer)

    def set_target_velocity(self, command: Any) -> Future:
        self.sent.append(command)
        return Future(True)

    def close(self, timeout: float | None = None) -> None:
        self.closed = True


def reading(x: float, y: float, yaw: float) -> dict[str, Any]:
    return {"translation": (x, y), "rotation": yaw}


@pytest.fixture()
def module():
    alfred = AlfredHighLevel()
    alfred.config.wheel_odometry_hz = 1000.0
    alfred.wheel_odometry = MagicMock()
    try:
        yield alfred
    finally:
        alfred.stop()


def poll(module: AlfredHighLevel, *readings: Any) -> list:
    client = Client(*readings, stop=module._odometry_stop)
    asyncio.run(module._poll_wheel_odometry(client))
    return [call.args[0] for call in module.wheel_odometry.publish.call_args_list]


def freeze_stamps(monkeypatch: Any, *stamps: float) -> None:
    clock = iter(stamps)
    monkeypatch.setattr(effector_high_level, "time", SimpleNamespace(time=lambda: next(clock)))


def test_the_controllers_inverted_y_frame_is_negated_back_to_ros(module):
    (published,) = poll(module, reading(1.0, 2.0, 0.5))

    assert (published.pose.position.x, published.pose.position.y) == (1.0, -2.0)
    assert published.pose.orientation.to_euler().z == pytest.approx(-0.5)
    assert published.frame_id == "wheel_odom"
    assert published.child_frame_id == "base_link"


def test_the_first_reading_publishes_a_zero_twist(module):
    (published,) = poll(module, reading(1.0, 2.0, 0.5))

    assert published.twist.linear.to_tuple() == (0.0, 0.0, 0.0)
    assert published.twist.angular.to_tuple() == (0.0, 0.0, 0.0)


def test_the_delta_splits_on_the_mid_interval_heading(module, monkeypatch):
    # A quarter circle driven at constant speed: the chord is 45 degrees off both
    # endpoint headings, so either one alone would leak half the motion into left.
    quarter = math.pi / 2
    freeze_stamps(monkeypatch, 0.0, 1.0)
    published = poll(module, reading(0.0, 0.0, 0.0), reading(1.0, -1.0, -quarter))[-1]

    chord = math.sqrt(2.0)
    assert published.twist.linear.x == pytest.approx(chord)
    assert published.twist.linear.y == pytest.approx(0.0, abs=1e-9)
    assert published.twist.angular.z == pytest.approx(quarter)


def test_a_yaw_that_crosses_pi_turns_the_short_way(module, monkeypatch):
    freeze_stamps(monkeypatch, 0.0, 1.0)
    published = poll(module, reading(0.0, 0.0, -3.1), reading(0.0, 0.0, 3.1))[-1]

    assert published.twist.angular.z == pytest.approx(2.0 * math.pi - 6.2)


def test_a_repeated_stamp_publishes_rather_than_dividing_by_zero(module, monkeypatch):
    freeze_stamps(monkeypatch, 1.0, 1.0)
    published = poll(module, reading(0.0, 0.0, 0.0), reading(1.0, 0.0, 0.0))[-1]

    assert published.pose.position.x == 1.0
    assert published.twist.linear.to_tuple() == (0.0, 0.0, 0.0)


def test_a_failing_poll_keeps_polling_and_logs_once_per_interval(module, monkeypatch):
    errors = MagicMock()
    monkeypatch.setattr(effector_high_level.logger, "error", errors)

    published = poll(module, RuntimeError("connection is dead"), RuntimeError("still dead"))

    assert published == []
    assert errors.call_count == 1


def test_the_wait_ends_the_instant_the_stop_is_set(module):
    async def stop_midwait() -> float:
        loop = asyncio.get_running_loop()
        started = loop.time()
        loop.call_soon(module._odometry_stop.set)
        await module._wait_or_stop(30.0)
        return loop.time() - started

    assert asyncio.run(stop_midwait()) < 1.0


def test_the_connection_closes_even_when_the_stop_command_fails():
    client = Client()
    client.set_target_velocity = MagicMock(side_effect=RuntimeError("unreachable"))

    _stop_and_close(client)

    assert client.closed


def test_a_controller_that_answers_nothing_still_closes_within_the_teardown_join():
    class Unanswered(Future):
        def done(self) -> bool:
            return False

        def result(self, timeout: float | None = None) -> Any:
            assert timeout is not None, "an unbounded stop starves the close below it"
            time.sleep(timeout)
            raise TimeoutError

    class Unreachable(Client):
        def set_target_velocity(self, command: Any) -> Future:
            return Unanswered()

        def close(self, timeout: float | None = None) -> None:
            # Portal joins its socket thread for the timeout, then kills it regardless.
            assert timeout is not None, "portal waits out a send queue a dead peer never drains"
            time.sleep(timeout)
            self.closed = True

    client = Unreachable()
    started = time.monotonic()
    _stop_and_close(client)

    assert client.closed
    assert time.monotonic() - started < effector_high_level.DEFAULT_THREAD_JOIN_TIMEOUT


def test_a_pending_rpc_is_awaited_rather_than_blocking_the_loop():
    class Pending(Future):
        def __init__(self) -> None:
            super().__init__("answered")
            self.polls = 0

        def done(self) -> bool:
            self.polls += 1
            return self.polls > 3

    future = Pending()
    assert asyncio.run(_rpc_result(future)) == "answered"


def test_move_negates_y_and_yaw_into_the_alfred_frame(module):
    client = Client()
    module._client = client

    assert module.move(Twist(linear=Vector3(0.3, 0.2, 0.0), angular=Vector3(0, 0, 0.4)), 10.0)

    assert list(client.sent[0]["target_velocity"]) == pytest.approx([0.3, -0.2, -0.4])
    assert module.get_state() == "MOVING"


def test_a_move_without_a_connection_is_refused(module):
    assert not module.move(Twist(linear=Vector3(1.0, 0.0, 0.0)))
    assert module.get_state() == "DISCONNECTED"


def test_a_rejected_command_neither_rearms_the_watchdog_nor_counts_as_moving(module):
    client = Client()
    client.set_target_velocity = MagicMock(side_effect=RuntimeError("unreachable"))
    module._client = client

    assert not module.move(Twist(linear=Vector3(1.0, 0.0, 0.0)))
    assert module._stop_task is None
    assert module.get_state() == "STOPPED"


def test_the_watchdog_stops_the_platform_once_the_duration_runs_out(module):
    client = Client()
    module._client = client

    assert module.move(Twist(linear=Vector3(0.3, 0.0, 0.0)), 0.01)
    deadline = time.monotonic() + 5.0
    while len(client.sent) < 2 and time.monotonic() < deadline:
        time.sleep(0.01)

    assert list(client.sent[-1]["target_velocity"]) == [0.0, 0.0, 0.0]
    assert module.get_state() == "STOPPED"


def test_the_teardown_stops_the_platform_and_closes_the_connection(module, monkeypatch):
    client = Client()
    monkeypatch.setitem(sys.modules, "portal", SimpleNamespace(Client=lambda address: client))

    async def lifecycle() -> None:
        running = module.main()
        await running.__anext__()
        assert module._client is client
        with pytest.raises(StopAsyncIteration):
            await running.__anext__()

    asyncio.run(lifecycle())

    assert list(client.sent[-1]["target_velocity"]) == [0.0, 0.0, 0.0]
    assert client.closed
    assert module._client is None
    assert module._odometry_task.done()
