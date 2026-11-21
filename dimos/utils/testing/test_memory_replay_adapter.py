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
from types import SimpleNamespace

import pytest
from pytest_mock import MockerFixture
from reactivex import operators as ops
from reactivex.testing import TestScheduler

from dimos.memory.store.sqlite import SqliteStore
from dimos.utils.testing.replay import MemoryReplayAdapter


@pytest.fixture
def replay(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[MemoryReplayAdapter[int]]:
    with SqliteStore(path=str(tmp_path / "recording.db")) as store:
        stream = store.stream("odom", int)
        for i, ts in enumerate([10.02, 10.0, 10.01]):
            stream.append(i, ts=ts)
        monkeypatch.setattr("dimos.utils.testing.replay._get_store", lambda _: store)
        yield MemoryReplayAdapter("recording/odom")


def test_first_timestamp_uses_earliest_source_timestamp(replay: MemoryReplayAdapter[int]) -> None:
    assert replay.first_timestamp() == 10.0


def test_seek_uses_earliest_source_timestamp(replay: MemoryReplayAdapter[int]) -> None:
    assert list(replay.iterate_ts(seek=0.005)) == [(10.02, 0), (10.01, 2)]


def test_duration_uses_earliest_source_timestamp(replay: MemoryReplayAdapter[int]) -> None:
    assert list(replay.iterate_ts(duration=0.015)) == [(10.0, 1), (10.01, 2)]


def test_find_closest_seek_uses_earliest_source_timestamp(replay: MemoryReplayAdapter[int]) -> None:
    assert replay.find_closest_seek(0.005) == 0


def test_equal_timestamp_frames_stay_together_on_each_pass(
    tmp_path: Path, mocker: MockerFixture
) -> None:
    with SqliteStore(path=str(tmp_path / "camera.db")) as sqlite_store:
        stream = sqlite_store.stream("camera", int)
        for value in (1, 2):
            stream.append(value, ts=10.0)
        mocker.patch("dimos.utils.testing.replay._get_store", return_value=sqlite_store)
        clock = TestScheduler()
        mocker.patch("dimos.utils.testing.replay.time", SimpleNamespace(time=lambda: clock.clock))
        replay = MemoryReplayAdapter[int]("recording/camera")
        events = []
        with (
            replay.stream(loop=True)
            .pipe(ops.take(4))
            .subscribe(lambda value: events.append((clock.clock, value)), scheduler=clock)
        ):
            clock.advance_to(1)
        assert events == [(0, 1), (0, 2), (0.05, 1), (0.05, 2)]
