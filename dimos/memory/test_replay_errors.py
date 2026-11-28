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

from contextlib import closing
import gc
from pathlib import Path
import sqlite3
import threading
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.constants import HUNG_TEST_TIMEOUT
from dimos.memory.replay_module import ReplayModule, replay_module
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.utils.time_window import RecordingWindow
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

pytestmark = [
    pytest.mark.filterwarnings("error::pytest.PytestUnhandledThreadExceptionWarning"),
    pytest.mark.filterwarnings("error::pytest.PytestUnraisableExceptionWarning"),
]


@pytest.fixture(params=["sqlite", "sqlite-eager", "mcap"])
def corrupt_recording(request: pytest.FixtureRequest, tmp_path: Path) -> str:
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
        return str(path)
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
        return str(path)


def _play_to_the_end(module: ReplayModule, port: str) -> list[Any]:
    """Frames *port* published before the replay ended or failed."""
    received: list[Any] = []
    module.outputs[port].subscribe(received.append)
    try:
        module.start()
        module.on_system_modules_ready([])
        assert module.replay_done.wait(HUNG_TEST_TIMEOUT)
    finally:
        module.stop()
    return received


@pytest.mark.parametrize("loop", [False, True])
def test_corrupt_payload_stops_the_replay(
    corrupt_recording: str, loop: bool, mocker: MockerFixture
) -> None:
    log = mocker.patch("dimos.memory.replay_module.logger")
    module = replay_module(corrupt_recording)(dataset=corrupt_recording, loop=loop, speed=10.0)
    received = _play_to_the_end(module, "pose")

    assert [pose.ts for pose in received] == [1.0]
    assert log.error.call_count == 1
    assert log.error.call_args.args == ("Replay stopped: recording could not be read",)
    # A retained exception must not keep a cursor alive past the recording's close.
    module._error = None
    gc.collect()


@pytest.mark.parametrize("fail_at", [0, 1])
def test_read_failure_stops_every_stream(
    tmp_path: Path, fail_at: int, mocker: MockerFixture
) -> None:
    path = str(tmp_path / "poses.db")
    with SqliteStore(path=path) as store:
        for name in ("broken", "odom"):
            for i in range(3):
                store.stream(name, PoseStamped).append(
                    PoseStamped(ts=1.0 + i * 0.01), ts=1.0 + i * 0.01
                )
    apply = RecordingWindow.apply
    closed = threading.Event()

    class Breaking:
        """The broken stream's rows: they fail to read at *fail_at*."""

        def __init__(self, rows: Any) -> None:
            self.rows = rows.order_by("ts")
            self.count, self.first, self.last = self.rows.count, self.rows.first, self.rows.last

        def order_by(self, _key: str) -> "Breaking":
            return self

        def __iter__(self) -> Any:
            try:
                for i, obs in enumerate(self.rows):
                    if i == fail_at:
                        raise OSError("recording read failed")
                    yield obs
            finally:
                closed.set()

    def failing(window: RecordingWindow, stream: Any) -> Any:
        rows = apply(window, stream)
        return Breaking(rows) if stream.name == "broken" else rows

    mocker.patch.object(RecordingWindow, "apply", failing)
    log = mocker.patch("dimos.memory.replay_module.logger")
    module = replay_module(path)(dataset=path)
    received = _play_to_the_end(module, "broken")

    assert [pose.ts for pose in received] == [1.0] * fail_at
    assert log.error.call_count == 1
    assert closed.is_set()
