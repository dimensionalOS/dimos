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
import threading
import time

import pytest
from pytest_mock import MockerFixture

from dimos.constants import HUNG_TEST_TIMEOUT
from dimos.memory.replay import Playback, Replay
from dimos.memory.store.sqlite import SqliteStore

pytestmark = [
    pytest.mark.filterwarnings("error::pytest.PytestUnhandledThreadExceptionWarning"),
    pytest.mark.filterwarnings("error::pytest.PytestUnraisableExceptionWarning"),
]

# Recorded seconds; odom and lidar interleave, camera starts first.
_RECORDING = {
    "odom": [1.00, 1.02, 1.04],
    "lidar": [1.01, 1.03],
    "camera": [0.99, 1.05],
}


@pytest.fixture
def store(tmp_path: Path) -> Iterator[SqliteStore]:
    # A page per row forces each advance to read SQLite again.
    with SqliteStore(path=str(tmp_path / "replay.db"), page_size=1) as store:
        for name, timestamps in _RECORDING.items():
            stream = store.stream(name, float)
            for ts in timestamps:
                stream.append(ts, ts=ts)
        yield store


def _record(
    replay: Replay, names: list[str]
) -> tuple[Playback, list[tuple[str, float]], threading.Event]:
    """A playback of *names* logging (stream, frame) in publish order, and an event set when all end."""
    playback = replay.playback()
    published: list[tuple[str, float]] = []
    remaining = set(names)
    done = threading.Event()

    def ended(name: str) -> None:
        remaining.discard(name)
        if not remaining:
            done.set()

    for name in names:
        playback.add(replay.stream(name)).subscribe(
            lambda frame, name=name: published.append((name, frame)),
            on_completed=lambda name=name: ended(name),
        )
    return playback, published, done


def test_streams_publish_in_recorded_order(store: SqliteStore) -> None:
    playback, published, done = _record(store.replay(speed=10.0), list(_RECORDING))
    playback.play()
    assert done.wait(HUNG_TEST_TIMEOUT)
    playback.stop()

    expected = sorted((ts, name) for name, stamps in _RECORDING.items() for ts in stamps)
    assert published == [(name, ts) for ts, name in expected]


def test_nothing_publishes_before_play(store: SqliteStore) -> None:
    playback, published, done = _record(store.replay(), ["odom"])
    playback.load()
    assert not done.wait(0.2)
    assert published == []
    playback.play()
    assert done.wait(HUNG_TEST_TIMEOUT)
    playback.stop()
    assert published == [("odom", ts) for ts in _RECORDING["odom"]]


def test_no_frame_publishes_before_its_recorded_offset(store: SqliteStore) -> None:
    replay = store.replay(speed=0.5)
    playback = replay.playback()
    arrivals: list[float] = []
    done = threading.Event()
    playback.add(replay.stream("odom")).subscribe(
        lambda _frame: arrivals.append(time.monotonic()), on_completed=done.set
    )
    started = time.monotonic()
    playback.play()
    assert done.wait(HUNG_TEST_TIMEOUT)
    playback.stop()

    # At half speed the 20 ms recorded gaps take 40 ms. Timers only ever wake late.
    offsets = [arrival - started for arrival in arrivals]
    assert offsets[1] >= 0.04
    assert offsets[2] >= 0.08


def test_slow_subscriber_drops_nothing_and_warns_once(
    store: SqliteStore, mocker: MockerFixture
) -> None:
    log = mocker.patch("dimos.memory.replay.logger")
    replay = store.replay(lag_warning=0.05)
    playback = replay.playback()
    published: list[float] = []
    done = threading.Event()

    def slow(frame: float) -> None:
        published.append(frame)
        time.sleep(0.1)  # twice the lag warning, every frame

    playback.add(replay.stream("odom")).subscribe(slow, on_completed=done.set)
    playback.play()
    assert done.wait(HUNG_TEST_TIMEOUT)
    playback.stop()

    assert published == _RECORDING["odom"]
    assert log.warning.call_count == 1
    assert log.warning.call_args.args == ("Replay is behind schedule",)


def test_loop_replays_in_order_after_the_gap(store: SqliteStore) -> None:
    replay = store.replay(loop=True, loop_gap=0.3)
    playback = replay.playback()
    published: list[float] = []
    arrivals: list[float] = []
    enough = threading.Event()

    def on_frame(frame: float) -> None:
        published.append(frame)
        arrivals.append(time.monotonic())
        if len(published) == 2 * len(_RECORDING["odom"]):
            enough.set()

    playback.add(replay.stream("odom")).subscribe(on_frame)
    playback.play()
    assert enough.wait(HUNG_TEST_TIMEOUT)
    playback.stop()

    assert published[:6] == [*_RECORDING["odom"], *_RECORDING["odom"]]
    assert arrivals[3] - arrivals[2] >= 0.3


def test_stop_from_a_callback_allows_closing_the_store(store: SqliteStore) -> None:
    replay = store.replay()
    playback = replay.playback()
    published: list[float] = []

    stopped = threading.Event()

    def on_frame(frame: float) -> None:
        published.append(frame)
        playback.stop()
        store.stop()
        stopped.set()

    playback.add(replay.stream("odom")).subscribe(on_frame)
    try:
        playback.play()
        assert stopped.wait(HUNG_TEST_TIMEOUT)
    finally:
        playback.stop()

    assert published == [_RECORDING["odom"][0]]


def test_failing_subscriber_leaves_the_other_streams_playing(
    store: SqliteStore, mocker: MockerFixture
) -> None:
    log = mocker.patch("dimos.memory.replay.logger")
    replay = store.replay(speed=10.0)
    playback = replay.playback()
    odom: list[float] = []
    done = threading.Event()

    def broken(_frame: float) -> None:
        raise RuntimeError("subscriber bug")

    playback.add(replay.stream("lidar")).subscribe(broken)
    playback.add(replay.stream("odom")).subscribe(odom.append, on_completed=done.set)
    try:
        playback.play()
        assert done.wait(HUNG_TEST_TIMEOUT)
    finally:
        playback.stop()

    assert odom == _RECORDING["odom"]
    assert log.exception.call_count == len(_RECORDING["lidar"])
