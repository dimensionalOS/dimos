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
import sqlite3
import threading
from typing import Any

import numpy as np
import pytest
from pytest_mock import MockerFixture
from reactivex.disposable import Disposable

from dimos.constants import HUNG_TEST_TIMEOUT
from dimos.experimental.robot.bosdyn.spot.replay import SpotReplay
from dimos.mapping.relocalization.blueprints import RecordingPlayer
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation
from dimos.memory.utils.time_window import RecordingWindow
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

pytestmark = [
    pytest.mark.filterwarnings("error::pytest.PytestUnraisableExceptionWarning"),
    pytest.mark.filterwarnings("error::pytest.PytestUnhandledThreadExceptionWarning"),
]


@pytest.fixture(
    params=[
        # Reading one frame ahead, so a stop finds the reader mid-recording.
        pytest.param((RecordingPlayer, "dataset", {"buffer_size": 1}), id="recording"),
        pytest.param((SpotReplay, "dataset", {"loop": False, "buffer_size": 1}), id="spot"),
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
        for ts in (1.0, 1.1, 1.2):
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
        assert started.wait(HUNG_TEST_TIMEOUT)
        # Both streams still hold an open cursor here.
        player.stop()


def test_concurrent_stop_keeps_store_open_until_decode_finishes(
    player: RecordingPlayer | SpotReplay, mocker: MockerFixture
) -> None:
    entered = threading.Event()
    stops_waiting = threading.Event()
    decoded = threading.Event()
    decoded_at_dispose: list[bool] = []
    reader: list[threading.Thread] = []
    waiting: set[threading.Thread] = set()
    decode = Observation.data.fget
    join = threading.Thread.join
    calls = 0

    def held_decode(obs: Observation[Any]) -> Any:
        nonlocal calls
        calls += 1
        if calls == 1:  # the frame start() buffers
            return decode(obs)
        reader.append(threading.current_thread())
        entered.set()
        assert stops_waiting.wait(HUNG_TEST_TIMEOUT), "stop did not wait for the decode"
        value = decode(obs)
        decoded.set()
        return value

    def tracked_join(thread: threading.Thread, timeout: float | None = None) -> None:
        if thread in reader:
            waiting.add(threading.current_thread())
            if waiting == stoppers:
                stops_waiting.set()
        join(thread, timeout)

    mocker.patch.object(Observation, "data", property(held_decode))
    mocker.patch.object(threading.Thread, "join", tracked_join)
    player.register_disposable(Disposable(lambda: decoded_at_dispose.append(decoded.is_set())))
    first_stop = threading.Thread(target=player.stop)
    second_stop = threading.Thread(target=player.stop)
    stoppers = {first_stop, second_stop}
    player.start()
    store = player._store
    assert entered.wait(HUNG_TEST_TIMEOUT), "the reader did not decode ahead of the buffer"
    for stopper in stoppers:
        stopper.start()
    for stopper in stoppers:
        join(stopper, HUNG_TEST_TIMEOUT)
        assert not stopper.is_alive()

    assert all(decoded_at_dispose)
    with pytest.raises(sqlite3.ProgrammingError, match="closed database"):
        store.list_streams()


def test_player_can_stop_from_replay_callback(
    player: RecordingPlayer | SpotReplay, mocker: MockerFixture
) -> None:
    received = []
    stopped = threading.Event()

    def stop_on_frame(message: TFMessage) -> None:
        received.append(message)
        player.stop()
        stopped.set()

    with Disposable(player.tf.subscribe(stop_on_frame)):
        player.start()
        store = player._store
        player.on_system_modules_ready([])
        assert stopped.wait(HUNG_TEST_TIMEOUT)

    assert len(received) == 1
    with pytest.raises(sqlite3.ProgrammingError, match="closed database"):
        store.list_streams()


def test_player_closes_store_after_partial_startup(
    player: RecordingPlayer | SpotReplay, mocker: MockerFixture
) -> None:
    apply = RecordingWindow.apply
    opened = mocker.spy(SqliteStore, "start")

    def fail_tf(window: RecordingWindow, stream: Any) -> Any:
        if stream.name == "tf":
            raise RuntimeError("cannot open tf replay")
        return apply(window, stream)

    mocker.patch.object(RecordingWindow, "apply", fail_tf)
    with pytest.raises(RuntimeError, match="cannot open tf replay"):
        player.start()
    player.stop()

    (store,) = [call.args[0] for call in opened.call_args_list]
    with pytest.raises(sqlite3.ProgrammingError, match="closed database"):
        store.list_streams()


def test_player_logs_source_duration_and_completion(player, mocker):
    log = mocker.patch("dimos.memory.replay_module.logger")
    player.start()
    path = player.config.dataset
    log.info.assert_called_once_with(
        "Replay started",
        source=path,
        path=path,
        duration_s=pytest.approx(0.2),
        speed=1.0,
        loop=False,
    )
    player.on_system_modules_ready([])
    assert player.replay_done.wait(HUNG_TEST_TIMEOUT)
    assert log.info.call_args_list[1:] == [mocker.call("Replay finished", source=path)]
