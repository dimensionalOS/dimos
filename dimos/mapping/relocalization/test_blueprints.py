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
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sqlite3
import threading
from typing import Any

import numpy as np
import pytest
from pytest_mock import MockerFixture
from reactivex import Observable
from reactivex.disposable import Disposable

from dimos.experimental.robot.bosdyn.spot.replay import SpotReplay
from dimos.mapping.relocalization.blueprints import RecordingPlayer
from dimos.memory.replay import Playback, ReplayStream
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

pytestmark = [
    pytest.mark.filterwarnings("error::pytest.PytestUnraisableExceptionWarning"),
    pytest.mark.filterwarnings("error::pytest.PytestUnhandledThreadExceptionWarning"),
]


@pytest.fixture(
    params=[
        # 20 recorded seconds played in one, reading one frame ahead.
        pytest.param(
            (RecordingPlayer, "dataset", {"speed": 20.0, "buffer_size": 1}), id="recording"
        ),
        pytest.param(
            (SpotReplay, "dataset", {"loop": False, "speed": 20.0, "buffer_size": 1}), id="spot"
        ),
    ]
)
def player(
    request: pytest.FixtureRequest, tmp_path: Path
) -> Iterator[RecordingPlayer | SpotReplay]:
    path = tmp_path / "recording.db"
    with SqliteStore(path=str(path), page_size=1) as store:
        lidar = store.stream("pointlio_lidar", PointCloud2)
        odometry = store.stream("odometry", Odometry)
        tf = store.stream("tf", TFMessage)
        for ts in (1.0, 11.0, 21.0):
            lidar.append(PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32)), ts=ts)
            odometry.append(Odometry(ts=ts), ts=ts)
            tf.append(TFMessage(), ts=ts)

    player_type, path_param, kwargs = request.param
    module = player_type(**{path_param: str(path)}, **kwargs)
    yield module
    module.stop()


def test_player_stops_mid_replay(player: RecordingPlayer | SpotReplay) -> None:
    started = threading.Event()
    with Disposable(player.tf.subscribe(lambda _: started.set())):
        player.start()
        player.on_system_modules_ready([])
        assert started.wait(5)
        # Both streams still hold an open cursor here.
        player.stop()


def test_concurrent_stop_keeps_store_open_until_decode_finishes(
    player: RecordingPlayer | SpotReplay, mocker: MockerFixture
) -> None:
    entered = threading.Event()
    release = threading.Event()
    decoded = threading.Event()
    disposing = threading.Event()
    decode = ReplayStream._decode
    calls = 0

    def held_decode(self: ReplayStream[Any], obs: Any) -> Any:
        nonlocal calls
        calls += 1
        if calls == 1:  # the frame start() buffers
            return decode(self, obs)
        entered.set()
        assert release.wait(10), "decode was not released"
        value = decode(self, obs)
        decoded.set()
        return value

    mocker.patch.object(ReplayStream, "_decode", held_decode)
    replay = mocker.spy(SqliteStore, "replay")
    player.register_disposable(Disposable(disposing.set))
    with ThreadPoolExecutor(max_workers=2) as workers:
        try:
            player.start()
            store = replay.spy_return.store
            assert entered.wait(5), "the reader did not decode ahead of the buffer"
            stops = [workers.submit(player.stop) for _ in range(2)]
            assert not disposing.wait(0.1), "stop disposed before the in-flight decode finished"
            assert store.stream("tf").count() == 3
        finally:
            release.set()
        for stop in stops:
            stop.result(timeout=5)

    assert decoded.is_set()
    assert disposing.is_set()
    with pytest.raises(sqlite3.ProgrammingError, match="closed database"):
        store.list_streams()


def test_player_can_stop_from_replay_callback(
    player: RecordingPlayer | SpotReplay, mocker: MockerFixture
) -> None:
    replay = mocker.spy(SqliteStore, "replay")
    received = []
    stopped = threading.Event()

    def stop_on_frame(message: TFMessage) -> None:
        received.append(message)
        player.stop()
        stopped.set()

    with Disposable(player.tf.subscribe(stop_on_frame)):
        player.start()
        player.on_system_modules_ready([])
        assert stopped.wait(5)

    assert len(received) == 1
    with pytest.raises(sqlite3.ProgrammingError, match="closed database"):
        replay.spy_return.store.list_streams()


def test_player_closes_store_after_partial_startup(
    player: RecordingPlayer | SpotReplay, mocker: MockerFixture
) -> None:
    add = Playback.add

    def fail_tf(self: Playback, stream: ReplayStream[Any]) -> Observable[Any]:
        if stream.name == "tf":
            raise RuntimeError("cannot open tf replay")
        return add(self, stream)

    mocker.patch.object(Playback, "add", fail_tf)
    replay = mocker.spy(SqliteStore, "replay")
    with pytest.raises(RuntimeError, match="cannot open tf replay"):
        player.start()
    player.stop()

    with pytest.raises(sqlite3.ProgrammingError, match="closed database"):
        replay.spy_return.store.list_streams()


def test_player_logs_source_duration_and_completion(player, mocker):
    log = mocker.patch("dimos.memory.replay_progress.logger")
    finished = threading.Event()
    log.info.side_effect = lambda message, **_fields: (
        finished.set() if message == "Replay finished" else None
    )
    player.start()
    path = player.config.dataset
    log.info.assert_called_once_with(
        "Replay started",
        source=path,
        path=path,
        duration_s=20.0,
        speed=20.0,
        loop=False,
    )
    player.on_system_modules_ready([])
    assert finished.wait(5)
    assert log.info.call_args_list[1:] == [mocker.call("Replay finished", source=path)]
