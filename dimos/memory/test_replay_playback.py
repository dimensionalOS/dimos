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
from typing import Any

from pydantic import ValidationError
import pytest
from pytest_mock import MockerFixture

from dimos.memory.replay import Playback, Replay
from dimos.memory.store.sqlite import SqliteStore

_TIMEOUT = 5.0  # seconds a test waits on the replay before failing

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
    assert done.wait(_TIMEOUT)
    playback.stop()

    expected = sorted((ts, name) for name, stamps in _RECORDING.items() for ts in stamps)
    assert published == [(name, ts) for ts, name in expected]


def test_nothing_publishes_before_play(store: SqliteStore) -> None:
    playback, published, done = _record(store.replay(), ["odom"])
    playback.load()
    assert not done.wait(0.2)
    assert published == []
    playback.play()
    assert done.wait(_TIMEOUT)
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
    assert done.wait(_TIMEOUT)
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
    assert done.wait(_TIMEOUT)
    playback.stop()

    assert published == _RECORDING["odom"]
    assert log.warning.call_count == 1
    assert log.warning.call_args.args == ("Replay is behind schedule",)


def test_load_returns_with_the_buffer_full(store: SqliteStore, mocker: MockerFixture) -> None:
    replay = store.replay(buffer_size=2)
    playback, _published, _done = _record(replay, list(_RECORDING))
    buffer_type = type(playback._buffer)
    put = buffer_type.put
    third_put = threading.Event()
    calls = 0

    def counting_put(self: Any, item: Any) -> bool:
        nonlocal calls
        calls += 1
        if calls == 3:
            third_put.set()
        return put(self, item)

    mocker.patch.object(buffer_type, "put", counting_put)
    try:
        playback.load()
        assert len(playback._buffer._items) == 2
        assert third_put.wait(_TIMEOUT)
        assert len(playback._buffer._items) == 2  # the third waits on the full buffer
    finally:
        playback.stop()


def test_load_returns_when_the_recording_is_shorter_than_the_buffer(store: SqliteStore) -> None:
    playback, _published, _done = _record(store.replay(), ["lidar"])
    playback.load()
    # Both frames and the stream's end.
    assert len(playback._buffer._items) == 3
    playback.stop()


def test_loop_replays_in_order_after_the_last_frame(store: SqliteStore) -> None:
    replay = store.replay(loop=True, speed=10.0)
    playback = replay.playback()
    published: list[float] = []
    enough = threading.Event()

    def on_frame(frame: float) -> None:
        published.append(frame)
        if len(published) == 2 * len(_RECORDING["odom"]):
            enough.set()

    playback.add(replay.stream("odom")).subscribe(on_frame)
    playback.play()
    assert enough.wait(_TIMEOUT)
    playback.stop()

    assert published[:6] == [*_RECORDING["odom"], *_RECORDING["odom"]]


def test_empty_stream_completes(store: SqliteStore) -> None:
    store.stream("empty", float)
    playback, published, done = _record(store.replay(), ["empty", "lidar"])
    playback.play()
    assert done.wait(_TIMEOUT)
    playback.stop()
    assert published == [("lidar", ts) for ts in _RECORDING["lidar"]]


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
        assert stopped.wait(_TIMEOUT)
    finally:
        playback.stop()

    assert published == [_RECORDING["odom"][0]]


def test_stop_waits_for_an_in_flight_publish(store: SqliteStore) -> None:
    replay = store.replay()
    playback = replay.playback()
    publishing = threading.Event()
    published = threading.Event()

    def on_frame(_frame: float) -> None:
        publishing.set()
        time.sleep(0.1)
        published.set()

    playback.add(replay.stream("odom")).subscribe(on_frame)
    playback.play()
    assert publishing.wait(_TIMEOUT)
    playback.stop()
    assert published.is_set()


def test_stop_before_play_releases_the_reader(store: SqliteStore) -> None:
    playback, _published, _done = _record(store.replay(buffer_size=1), list(_RECORDING))
    playback.load()
    playback.stop()
    assert playback._reader is not None
    assert not playback._reader.is_alive()


def test_add_twice_or_after_load_is_refused(store: SqliteStore) -> None:
    replay = store.replay()
    playback = replay.playback()
    playback.add(replay.stream("odom"))
    with pytest.raises(ValueError):
        playback.add(replay.stream("odom"))
    playback.load()
    with pytest.raises(RuntimeError):
        playback.add(replay.stream("lidar"))
    playback.stop()


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
        assert done.wait(_TIMEOUT)
    finally:
        playback.stop()

    assert odom == _RECORDING["odom"]
    assert log.exception.call_count == len(_RECORDING["lidar"])


def test_play_again_returns_at_once(store: SqliteStore) -> None:
    replay = store.replay(buffer_size=1)
    playback, _published, done = _record(replay, list(_RECORDING))
    try:
        playback.play()
        # The publisher drains the buffer, so a load() that waited again could block.
        playback.play()
        playback.load()
        assert done.wait(_TIMEOUT)
    finally:
        playback.stop()


def test_buffer_size_must_be_positive(store: SqliteStore) -> None:
    with pytest.raises(ValidationError):
        store.replay(buffer_size=0)
