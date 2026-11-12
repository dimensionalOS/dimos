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
from typing import Any

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
