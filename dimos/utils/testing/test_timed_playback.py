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

from collections.abc import Iterator
from contextlib import closing
import sqlite3
import threading
from types import SimpleNamespace
from typing import Any

import pytest
from pytest_mock import MockerFixture
from reactivex import operators as ops
from reactivex.abc import DisposableBase, ScheduledAction
from reactivex.disposable import Disposable
from reactivex.testing import TestScheduler

from dimos.utils.testing.replay import timed_playback


class EagerScheduler(TestScheduler):
    """Runs a zero-delay action before returning its handle, as a fast Timer thread can."""

    def __init__(self) -> None:
        super().__init__()
        self.timers: list[DisposableBase] = []

    def schedule_relative(
        self, duetime: Any, action: ScheduledAction[Any], state: Any = None
    ) -> DisposableBase:
        if duetime == 0:
            action(self, state)
            return Disposable()
        timer = super().schedule_relative(duetime, action, state)
        self.timers.append(timer)
        return timer


def frames() -> Iterator[tuple[float, str]]:
    # "b" is due at once, so it is emitted before its own timer handle is stored.
    return iter([(0.0, "a"), (0.0, "b"), (100.0, "c")])


def test_emission_before_its_timer_is_stored_does_not_stop_the_replay() -> None:
    scheduler = EagerScheduler()
    received: list[str] = []
    completed: list[bool] = []
    with timed_playback(frames).subscribe(
        received.append, on_completed=lambda: completed.append(True), scheduler=scheduler
    ):
        assert received == ["a", "b"]
        scheduler.advance_by(1000)

    assert received == ["a", "b", "c"]
    assert completed == [True]


def test_dispose_cancels_the_timer_scheduled_by_an_early_emission() -> None:
    scheduler = EagerScheduler()
    received: list[str] = []
    with timed_playback(frames).subscribe(received.append, scheduler=scheduler):
        [timer] = scheduler.timers
        assert not timer.is_disposed

    assert timer.is_disposed
    scheduler.advance_by(1000)
    assert received == ["a", "b"]


@pytest.mark.parametrize("valid_frames", [0, 1, 3])
@pytest.mark.parametrize("virtual", [False, True], ids=["timer", "virtual"])
@pytest.mark.filterwarnings("error::pytest.PytestUnhandledThreadExceptionWarning")
def test_source_failure_reports_one_error(
    valid_frames: int, virtual: bool, mocker: MockerFixture
) -> None:
    scheduler = TestScheduler() if virtual else None
    error = OSError("recording read failed")
    received: list[int] = []
    finished = threading.Event()
    on_error = mocker.Mock(side_effect=lambda _: finished.set())
    completed = mocker.Mock(side_effect=finished.set)
    closed = mocker.Mock()

    def source() -> Iterator[tuple[float, int]]:
        try:
            for value in range(valid_frames):
                yield value * 0.01, value
            raise error
        finally:
            closed()

    with timed_playback(source).subscribe(
        received.append, on_error, completed, scheduler=scheduler
    ):
        if scheduler is not None:
            scheduler.advance_by(1)
        assert finished.wait(2), "playback did not deliver a terminal event"

    assert received == list(range(valid_frames))
    on_error.assert_called_once_with(error)
    completed.assert_not_called()
    closed.assert_called_once_with()


@pytest.mark.parametrize("frame_count", [1, 2])
def test_completion_can_close_the_source_database(frame_count: int, mocker: MockerFixture) -> None:
    scheduler = TestScheduler()
    received: list[str] = []
    on_error = mocker.Mock()

    with closing(sqlite3.connect(":memory:")) as connection:
        completed = mocker.Mock(side_effect=connection.close)

        def source() -> Iterator[tuple[float, str]]:
            return connection.execute(
                "SELECT 0.0, 'first' UNION ALL SELECT 0.01, 'second' LIMIT ?",
                (frame_count,),
            )

        with timed_playback(source).subscribe(
            received.append, on_error, completed, scheduler=scheduler
        ):
            scheduler.advance_by(1)

    assert received == ["first", "second"][:frame_count]
    completed.assert_called_once_with()
    on_error.assert_not_called()


def test_slow_observer_leaves_a_gap_before_the_next_pass(mocker: MockerFixture) -> None:
    clock = TestScheduler()
    mocker.patch("dimos.utils.testing.replay.time", SimpleNamespace(time=lambda: clock.clock))
    times = []

    def receive(_value: int) -> None:
        times.append(clock.clock)
        if len(times) == 2:
            clock.sleep(1.0)

    replay = timed_playback(lambda: iter([(10.0, 1), (10.2, 2)]), loop=True)
    with replay.pipe(ops.take(4)).subscribe(receive, scheduler=clock):
        clock.advance_to(2)
    assert times == pytest.approx([0.0, 0.2, 1.25, 1.45])
