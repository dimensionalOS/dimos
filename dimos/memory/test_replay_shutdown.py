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
from pathlib import Path
from typing import Any

import pytest
from pytest_mock import MockerFixture
from reactivex.abc import DisposableBase, ScheduledAction
from reactivex.disposable import Disposable
from reactivex.testing import TestScheduler

from dimos.memory.store.sqlite import SqliteStore


@pytest.fixture
def recording(tmp_path: Path) -> Iterator[SqliteStore]:
    # A page per row forces advancing the iterator to access SQLite again.
    with SqliteStore(path=str(tmp_path / "replay.db"), page_size=1) as store:
        stream = store.stream("data", bytes)
        stream.append(b"first", ts=1.0)
        stream.append(b"second", ts=2.0)
        yield store


@pytest.mark.filterwarnings("error::pytest.PytestUnraisableExceptionWarning")
def test_dispose_cancels_pending_emission(recording: SqliteStore, mocker: MockerFixture) -> None:
    scheduler = TestScheduler()
    schedule = mocker.spy(scheduler, "schedule_relative")
    received: list[bytes] = []
    replay = recording.replay()
    replay.pin_anchor(0.0)
    with replay.stream("data").observable().subscribe(received.append, scheduler=scheduler):
        scheduled = schedule.spy_return
        assert not scheduled.is_disposed

    assert scheduled.is_disposed
    recording.stop()
    scheduler.advance_by(10)
    assert received == []


@pytest.mark.filterwarnings("error::pytest.PytestUnraisableExceptionWarning")
def test_dispose_from_callback_allows_closing_store(recording: SqliteStore) -> None:
    scheduler = TestScheduler()
    received: list[bytes] = []

    def on_next(value: bytes) -> None:
        received.append(value)
        subscription.dispose()
        recording.stop()

    with (
        recording.replay()
        .stream("data")
        .observable()
        .subscribe(on_next, scheduler=scheduler) as subscription
    ):
        scheduler.advance_by(10)

    assert received == [b"first"]


def test_immediate_emission_preserves_next_timer_for_disposal(
    recording: SqliteStore, mocker: MockerFixture
) -> None:
    scheduler = TestScheduler()
    schedule_relative = scheduler.schedule_relative
    timers = []

    def schedule(delay: float, action: ScheduledAction[Any], state: Any = None) -> DisposableBase:
        if delay == 0:
            # The first emission schedules the next one before returning its
            # own cancellation handle, as a fast Timer thread can do.
            action(scheduler, state)
            return Disposable()
        timer = schedule_relative(delay, action, state)
        timers.append(timer)
        return timer

    mocker.patch.object(scheduler, "schedule_relative", side_effect=schedule)
    received: list[bytes] = []
    with (
        recording.replay()
        .stream("data")
        .observable()
        .subscribe(received.append, scheduler=scheduler)
    ):
        assert received == [b"first"]
        assert len(timers) == 1
        assert not timers[0].is_disposed

    assert timers[0].is_disposed
    scheduler.advance_by(10)
    assert received == [b"first"]


def test_completion_delivers_all_frames_and_releases_timer(
    recording: SqliteStore, mocker: MockerFixture
) -> None:
    scheduler = TestScheduler()
    schedule = mocker.spy(scheduler, "schedule_relative")
    received: list[bytes] = []
    completed = mocker.Mock()
    with (
        recording.replay()
        .stream("data")
        .observable()
        .subscribe(received.append, on_completed=completed, scheduler=scheduler)
    ):
        scheduler.advance_by(10)
        assert schedule.spy_return.is_disposed

    assert received == [b"first", b"second"]
    completed.assert_called_once_with()
