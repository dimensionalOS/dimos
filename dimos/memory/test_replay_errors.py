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
import gc
from pathlib import Path
import sqlite3
import threading

import pytest
from pytest_mock import MockerFixture
from reactivex.testing import TestScheduler

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


@pytest.mark.parametrize("virtual", [False, True], ids=["timer", "virtual"])
@pytest.mark.parametrize("loop", [False, True])
def test_corrupt_payload_closes_iterator_and_reports_one_error(
    corrupt_recording: Store, virtual: bool, loop: bool, mocker: MockerFixture
) -> None:
    scheduler = TestScheduler() if virtual else None
    stream = corrupt_recording.replay(loop=loop).stream("pose")
    base = stream._base_stream
    closed = threading.Event()
    finished = threading.Event()
    received: list[PoseStamped] = []
    errors: list[Exception] = []
    completed = mocker.Mock(side_effect=finished.set)

    def rows() -> Iterator[Observation[PoseStamped]]:
        try:
            yield from base()
        finally:
            closed.set()

    mocker.patch.object(stream, "_base_stream", side_effect=rows)

    def on_error(error: Exception) -> None:
        errors.append(error)
        assert closed.is_set()
        corrupt_recording.stop()
        finished.set()

    with stream.observable().subscribe(received.append, on_error, completed, scheduler=scheduler):
        if scheduler is not None:
            scheduler.advance_by(1)
        assert finished.wait(2), "replay did not deliver a terminal event"

    assert [pose.ts for pose in received] == [1.0]
    assert len(errors) == 1
    assert isinstance(errors[0], ValueError)
    assert str(errors[0]) == "Decode error"
    completed.assert_not_called()
    # A retained exception must not keep a cursor alive past store shutdown.
    # Collect it here so late cursor cleanup failures are caught by pytest.
    errors.clear()
    gc.collect()


@pytest.mark.parametrize("phase", ["initial", "skip", "scheduled"])
def test_read_failure_reports_one_error(phase: str, mocker: MockerFixture) -> None:
    scheduler = TestScheduler()
    error = OSError("recording read failed")
    closed = threading.Event()
    received: list[int] = []
    on_error = mocker.Mock()
    completed = mocker.Mock()

    def rows() -> Iterator[Observation[int]]:
        try:
            if phase != "initial":
                yield Observation(id=0, ts=1.0, _data=42)
            raise error
        finally:
            closed.set()

    with MemoryStore() as store:
        replay = store.replay(loop=True)
        if phase == "skip":
            replay.pin_anchor(10.0)
        stream = replay.stream("pose")
        mocker.patch.object(stream, "_base_stream", side_effect=rows)
        with stream.observable().subscribe(
            received.append, on_error, completed, scheduler=scheduler
        ):
            scheduler.advance_by(10)

    assert received == ([42] if phase == "scheduled" else [])
    on_error.assert_called_once_with(error)
    completed.assert_not_called()
    assert closed.is_set()


@pytest.mark.parametrize("fail_at", [0, 1])
def test_autocast_failure_closes_retained_iterator(fail_at: int, mocker: MockerFixture) -> None:
    scheduler = TestScheduler()
    error = ValueError("cannot convert frame")
    received: list[int] = []
    closed = mocker.Mock()
    completed = mocker.Mock()

    def rows() -> Iterator[Observation[int]]:
        try:
            for value in range(3):
                yield Observation(id=value, ts=float(value), _data=value)
        finally:
            closed()

    def autocast(value: int) -> int:
        if value == fail_at:
            raise error
        return value

    def on_error(error: Exception) -> None:
        closed.assert_called_once_with()

    error_handler = mocker.Mock(side_effect=on_error)
    with MemoryStore() as store, closing(rows()) as iterator:
        stream = store.replay(loop=True).stream("pose", autocast=autocast)
        mocker.patch.object(stream, "_base_stream", return_value=iterator)
        with stream.observable().subscribe(
            received.append, error_handler, completed, scheduler=scheduler
        ):
            scheduler.advance_by(10)

    assert received == list(range(fail_at))
    error_handler.assert_called_once_with(error)
    completed.assert_not_called()
