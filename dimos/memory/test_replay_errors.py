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
from functools import partial
import gc
from pathlib import Path
import sqlite3
import threading
from typing import Any
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture

from dimos.constants import HUNG_TEST_TIMEOUT
from dimos.memory.replay import Replay, ReplayStream
from dimos.memory.store.base import Store
from dimos.memory.store.mcap import McapStore
from dimos.memory.store.memory import MemoryStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

pytestmark = [
    pytest.mark.filterwarnings("error::pytest.PytestUnhandledThreadExceptionWarning"),
    pytest.mark.filterwarnings("error::pytest.PytestUnraisableExceptionWarning"),
]


@pytest.fixture(params=["sqlite", "sqlite-eager", "mcap"])
def corrupt_recording(request: pytest.FixtureRequest, tmp_path: Path) -> Iterator[Store]:
    poses = [PoseStamped(ts=1.0 + i * 0.01, frame_id="map") for i in range(3)]
    if request.param in {"sqlite", "sqlite-eager"}:
        path = tmp_path / "poses.db"
        with SqliteStore(path=path, page_size=1) as store:
            stream = store.stream("pose", PoseStamped, eager_blobs=request.param == "sqlite-eager")
            observations = [stream.append(pose) for pose in poses]
        with closing(sqlite3.connect(path)) as conn, conn:
            conn.execute(
                "UPDATE pose_blob SET data = ? WHERE id = ?", (b"bad-lcm", observations[1].id)
            )
        with SqliteStore(path=path, page_size=1) as store:
            yield store
    else:
        writer_module = pytest.importorskip("mcap.writer")
        path = tmp_path / "poses.mcap"
        with path.open("wb") as output:
            writer = writer_module.Writer(output)
            writer.start(profile="dimos")
            channel = writer.register_channel(
                topic="dimos/pose/geometry_msgs.PoseStamped", message_encoding="lcm", schema_id=0
            )
            for i, pose in enumerate(poses):
                writer.add_message(
                    channel_id=channel,
                    log_time=round(pose.ts * 1e9),
                    publish_time=round(pose.ts * 1e9),
                    data=b"bad-lcm" if i == 1 else pose.lcm_encode(),
                )
            writer.finish()
        with McapStore(path=str(path)) as store:
            yield store


def _play_until_terminal(
    replay: Replay, stream: ReplayStream[Any]
) -> tuple[list[Any], list[Exception], MagicMock]:
    """Play *stream* alone until it errors or completes; its frames, errors and on_completed."""
    playback = replay
    received: list[Any] = []
    errors: list[Exception] = []
    finished = threading.Event()
    completed = MagicMock(side_effect=finished.set)

    def on_error(error: Exception) -> None:
        errors.append(error)
        finished.set()

    playback.add(stream).subscribe(received.append, on_error, completed)
    playback.play()
    assert finished.wait(HUNG_TEST_TIMEOUT), "replay did not deliver a terminal event"
    playback.stop()
    return received, errors, completed


@pytest.mark.parametrize("loop", [False, True])
def test_corrupt_payload_reports_one_error(
    corrupt_recording: Store, loop: bool, mocker: MockerFixture
) -> None:
    replay = corrupt_recording.replay(loop=loop, speed=10.0)
    stream = replay.stream("pose")
    base = stream.rows
    closed = threading.Event()

    def rows() -> Iterator[Observation[PoseStamped]]:
        try:
            yield from base()
        finally:
            closed.set()

    mocker.patch.object(stream, "rows", side_effect=rows)
    received, errors, completed = _play_until_terminal(replay, stream)

    assert [pose.ts for pose in received] == [1.0]
    assert len(errors) == 1
    assert isinstance(errors[0], ValueError)
    assert str(errors[0]) == "Decode error"
    completed.assert_not_called()
    assert closed.is_set()
    corrupt_recording.stop()
    # A retained exception must not keep a cursor alive past store shutdown.
    # Collect it here so late cursor cleanup failures are caught by pytest.
    errors.clear()
    gc.collect()


@pytest.mark.parametrize(
    ("failure", "fail_at"),
    [("read", 0), ("read", 1), ("autocast", 0), ("autocast", 1)],
)
def test_read_or_autocast_failure_reports_one_error(
    failure: str, fail_at: int, mocker: MockerFixture
) -> None:
    error = OSError("frame failed")
    closed = threading.Event()

    def rows() -> Iterator[Observation[int]]:
        try:
            for value in range(3):
                if failure == "read" and value == fail_at:
                    raise error
                yield Observation(id=value, ts=1.0 + value * 0.01, _data=value)
        finally:
            closed.set()

    def autocast(value: int) -> int:
        if failure == "autocast" and value == fail_at:
            raise error
        return value

    with MemoryStore() as store:
        replay = store.replay(loop=True)
        stream = replay.stream("pose", autocast=autocast)
        mocker.patch.object(stream, "rows", side_effect=rows)
        received, errors, completed = _play_until_terminal(replay, stream)

    assert received == list(range(fail_at))
    assert errors == [error]
    completed.assert_not_called()
    assert closed.is_set()


def test_a_failed_stream_stops_the_whole_replay(mocker: MockerFixture) -> None:
    with MemoryStore() as store:
        for name in ("broken", "odom"):
            recorded = store.stream(name, int)
            for value in range(3):
                recorded.append(value, ts=1.0 + value * 0.01)
        replay = store.replay()
        broken = replay.stream("broken")
        mocker.patch.object(broken, "rows", side_effect=OSError("recording read failed"))
        playback = replay
        errors: dict[str, Exception] = {}
        both = threading.Event()

        def on_error(name: str, error: Exception) -> None:
            errors[name] = error
            if len(errors) == 2:
                both.set()

        odom: list[int] = []
        playback.add(broken).subscribe(on_error=partial(on_error, "broken"))
        playback.add(replay.stream("odom")).subscribe(odom.append, partial(on_error, "odom"))
        try:
            playback.play()
            assert both.wait(HUNG_TEST_TIMEOUT)
        finally:
            playback.stop()

    assert {name: str(error) for name, error in errors.items()} == {
        "broken": "recording read failed",
        "odom": "recording read failed",
    }
    assert odom == []
