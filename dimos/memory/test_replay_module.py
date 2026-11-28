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

from pathlib import Path
import threading
import time
from typing import Any
from unittest import mock

import pytest
from pytest_mock import MockerFixture
from reactivex import interval, merge
from reactivex.disposable import Disposable
from reactivex.testing import TestScheduler

from dimos.constants import HUNG_TEST_TIMEOUT
from dimos.core.stream import Out
from dimos.memory.replay import Playback, ReplayStream
from dimos.memory.replay_module import replay_module
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


@pytest.fixture
def recording(tmp_path: Path) -> str:
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    odom = store.stream("odom", PoseStamped)
    goal = store.stream("goal", PoseStamped)
    for ts in (1.0, 1.1, 1.2):
        odom.append(PoseStamped(ts=ts), ts=ts)
    goal.append(PoseStamped(ts=1.05), ts=1.05)
    store.stop()
    return str(path)


def test_ports_come_from_the_recording(recording: str) -> None:
    cls = replay_module(recording)
    assert cls.__annotations__ == {"goal": Out[PoseStamped], "odom": Out[PoseStamped]}
    assert replay_module(recording, topics="odom").__annotations__ == {"odom": Out[PoseStamped]}
    assert cls.stream_types == {"goal": PoseStamped, "odom": PoseStamped}
    assert replay_module("").__annotations__ == {}
    with pytest.raises(ValueError, match="matched none of: goal, odom"):
        replay_module(recording, topics="nope")


def test_module_publishes_recorded_messages(recording: str) -> None:
    cls = replay_module(recording)
    module = cls(dataset=recording)
    got: dict[str, list[float]] = {"odom": [], "goal": []}
    module.outputs["odom"].subscribe(lambda m: got["odom"].append(m.ts))
    module.outputs["goal"].subscribe(lambda m: got["goal"].append(m.ts))
    module.start()
    module.on_system_modules_ready([])
    deadline = time.time() + 5
    while time.time() < deadline and len(got["odom"]) < 3:
        time.sleep(0.01)
    module.stop()
    assert [round(t, 3) for t in got["odom"]] == [1.0, 1.1, 1.2]
    assert [round(t, 3) for t in got["goal"]] == [1.05]


def test_stop_waits_for_in_flight_decode(recording: str, mocker: MockerFixture) -> None:
    entered = threading.Event()
    stop_waiting = threading.Event()
    decoded = threading.Event()
    decoded_at_dispose: list[bool] = []
    reader: list[threading.Thread] = []
    decode = ReplayStream._decode
    join = threading.Thread.join
    calls = 0

    def held_decode(self: ReplayStream[Any], obs: Any) -> Any:
        nonlocal calls
        calls += 1
        if calls == 1:  # the frame start() buffers
            return decode(self, obs)
        reader.append(threading.current_thread())
        entered.set()
        assert stop_waiting.wait(HUNG_TEST_TIMEOUT), "stop did not wait for the decode"
        data = decode(self, obs)
        decoded.set()
        return data

    def tracked_join(thread: threading.Thread, timeout: float | None = None) -> None:
        if thread in reader and threading.current_thread() is stopper:
            stop_waiting.set()
        join(thread, timeout)

    mocker.patch.object(ReplayStream, "_decode", held_decode)
    mocker.patch.object(threading.Thread, "join", tracked_join)
    module = replay_module(recording, topics="odom")(
        dataset=recording, topics="odom", buffer_size=1
    )
    module.register_disposable(Disposable(lambda: decoded_at_dispose.append(decoded.is_set())))
    stopper = threading.Thread(target=module.stop)
    module.start()
    assert entered.wait(HUNG_TEST_TIMEOUT), "the reader did not decode ahead of the buffer"
    stopper.start()
    join(stopper, HUNG_TEST_TIMEOUT)

    assert not stopper.is_alive()
    assert decoded_at_dispose == [True]
    with SqliteStore(path=recording) as store:
        assert [obs.data.ts for obs in store.stream("odom")] == pytest.approx([1.0, 1.1, 1.2])


def test_portless_class_gets_ports_from_dataset(recording: str) -> None:
    """A worker imports the class without a recording; the instance still has the ports."""
    module = replay_module("")(dataset=recording)
    assert sorted(module.outputs) == ["goal", "odom"]
    module.stop()


def test_topic_filter_reaches_the_instance(recording: str) -> None:
    """Workers rebuild ports from config, so the filter must travel in the blueprint kwargs."""
    cls = replay_module(recording, topics="odom")
    assert cls.blueprint(dataset=recording).blueprints[0].kwargs["topics"] == "odom"
    module = replay_module("")(dataset=recording, topics="odom")
    assert sorted(module.outputs) == ["odom"]
    module.stop()


def test_stream_named_like_a_module_attribute_is_skipped(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from unittest.mock import MagicMock

    from dimos.memory import replay_module as rm

    log = MagicMock()
    monkeypatch.setattr(rm, "logger", log)
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    store.stream("start", PoseStamped).append(PoseStamped(ts=1.0), ts=1.0)
    store.stream("odom", PoseStamped).append(PoseStamped(ts=1.0), ts=1.0)
    store.stop()
    assert list(replay_module(str(path)).__annotations__) == ["odom"]
    module = replay_module("")(dataset=str(path))
    assert sorted(module.outputs) == ["odom"]
    module.stop()
    assert [c.args[1] for c in log.warning.call_args_list] == ["start", "start"]


def test_subclass_payload_gets_the_base_port_type(tmp_path: Path) -> None:
    """A recorded unitree Odometry is a PoseStamped on the wire; the port must say so or a
    PoseStamped consumer of the same name lands on a different topic."""
    from dimos.robot.unitree.type.odometry import Odometry

    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    store.stream("odom", Odometry).append(Odometry(ts=1.0), ts=1.0)
    store.stop()
    assert replay_module(str(path)).__annotations__ == {"odom": Out[PoseStamped]}


def test_single_timestamp_stream_is_republished_and_not_the_anchor(tmp_path: Path) -> None:
    """camera_info stamped at import sits 39 s before the run; the run must start at odom."""
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    info = store.stream("camera_info", PoseStamped)
    for _ in range(3):
        info.append(PoseStamped(ts=1.0), ts=1.0)
    store.stream("odom", PoseStamped).append(PoseStamped(ts=40.0), ts=40.0)
    store.stop()
    module = replay_module(str(path))(dataset=str(path))
    got: dict[str, int] = {"camera_info": 0, "odom": 0}

    def count(name: str) -> Any:
        def on_next(_m: Any) -> None:
            got[name] += 1

        return on_next

    for name in got:
        module.outputs[name].subscribe(count(name))
    module.start()
    module.on_system_modules_ready([])
    deadline = time.time() + 5
    while time.time() < deadline and (got["camera_info"] < 2 or got["odom"] < 1):
        time.sleep(0.05)
    module.stop()
    assert got["camera_info"] >= 2  # republished at 1 Hz
    assert got["odom"] == 1  # arrived within the 5 s deadline, not 39 s later


def test_slow_decode_delays_later_frames_but_drops_none(
    recording: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    from dimos.memory import replay as replay_mod

    real = replay_mod.ReplayStream._decode

    def slow(self: Any, obs: Any) -> Any:
        if obs.ts == 1.05:  # the goal frame, before the last two odom frames
            time.sleep(0.2)
        return real(self, obs)

    monkeypatch.setattr(replay_mod.ReplayStream, "_decode", slow)
    module = replay_module(recording)(dataset=recording)
    got: dict[str, list[float]] = {"odom": [], "goal": []}
    module.outputs["odom"].subscribe(lambda m: got["odom"].append(m.ts))
    module.outputs["goal"].subscribe(lambda m: got["goal"].append(m.ts))
    module.start()
    module.on_system_modules_ready([])
    deadline = time.time() + 5
    while time.time() < deadline and (len(got["odom"]) < 3 or not got["goal"]):
        time.sleep(0.01)
    module.stop()
    assert [round(t, 3) for t in got["odom"]] == [1.0, 1.1, 1.2]


def test_slow_stream_setup_does_not_skip_the_first_frames(recording: str) -> None:
    """Setup slower than the recording's first frames loses none of them."""
    real = ReplayStream.rows

    def slow(self: Any) -> Any:
        time.sleep(0.1)  # longer than the recording, on every query
        return real(self)

    got: dict[str, list[float]] = {"odom": [], "goal": []}
    with mock.patch("dimos.memory.replay.ReplayStream.rows", slow):
        module = replay_module(recording)(dataset=recording)
        module.outputs["odom"].subscribe(lambda m: got["odom"].append(m.ts))
        module.outputs["goal"].subscribe(lambda m: got["goal"].append(m.ts))
        module.start()
        module.on_system_modules_ready([])
    deadline = time.time() + 5
    while time.time() < deadline and (len(got["odom"]) < 3 or not got["goal"]):
        time.sleep(0.01)
    module.stop()
    assert [round(t, 3) for t in got["odom"]] == [1.0, 1.1, 1.2]
    assert [round(t, 3) for t in got["goal"]] == [1.05]


def test_recorded_rerun_config_from_run_dir(tmp_path: Path) -> None:
    from dimos.memory.replay_module import recorded_rerun_config

    db = tmp_path / "20260910-135235-unitree-go2" / "memory.db"
    cfg = recorded_rerun_config(str(db))
    assert "world/robot_body" in cfg["static"]
    assert callable(cfg["blueprint"])  # the live layout, so replay looks like the live run
    assert recorded_rerun_config(str(tmp_path / "downloads" / "x.db")) == {}
    assert recorded_rerun_config(str(tmp_path / "20260910-135235-no-such-bp" / "m.db")) == {}


def test_dataset_path_resolves_names_and_skips_the_default(recording: str) -> None:
    from dimos.memory.replay_module import dataset_path

    assert dataset_path(recording) == recording
    assert dataset_path(recording, explicit=False) == recording  # a real file always counts
    assert dataset_path("go2_short", explicit=False) == ""  # registry import: no LFS
    assert dataset_path("no-such-dataset-for-this-test") == ""


@pytest.mark.parametrize("static", [False, True])
def test_replay_logs_source_and_finishes_after_all_timed_streams(recording, mocker, static):
    # Static metadata must not extend the reported duration or prevent completion.
    with SqliteStore(path=recording) as store:
        if static:
            info = store.stream("camera_info", PoseStamped)
            for _ in range(2):
                info.append(PoseStamped(ts=0.0), ts=0.0)
    log = mocker.patch("dimos.memory.replay_progress.logger")
    module = replay_module(recording)(dataset=recording)
    add = mocker.spy(Playback, "add")
    odom: list[float] = []
    module.outputs["odom"].subscribe(lambda m: odom.append(m.ts))
    try:
        module.start()
        log.info.assert_called_once_with(
            "Replay started",
            source=recording,
            path=recording,
            duration_s=pytest.approx(0.2),
            speed=1.0,
            loop=False,
        )
        finished = threading.Event()
        merge(*add.spy_return_list).subscribe(on_completed=finished.set)
        module.on_system_modules_ready([])
        assert finished.wait(HUNG_TEST_TIMEOUT)
        assert len(odom) == 3  # goal ended first, with odom frames still to come
        assert log.info.call_args_list[1:] == [mocker.call("Replay finished", source=recording)]
    finally:
        module.stop()


def test_replay_with_only_static_metadata_reports_completion(recording, mocker):
    with SqliteStore(path=recording) as store:
        for _ in range(2):
            store.stream("camera_info", PoseStamped).append(PoseStamped(ts=0.0), ts=0.0)
    scheduler = TestScheduler()
    mocker.patch(
        "dimos.memory.replay_module.interval",
        side_effect=lambda period: interval(period, scheduler=scheduler),
    )
    log = mocker.patch("dimos.memory.replay_progress.logger")
    module = replay_module(recording, topics="camera_info")(dataset=recording, topics="camera_info")
    received = []
    with Disposable(module.outputs["camera_info"].subscribe(received.append)), module:
        module.on_system_modules_ready([])
        scheduler.advance_by(2)
    assert [message.ts for message in received] == [0.0, 0.0]
    assert log.info.call_args_list == [
        mocker.call(
            "Replay started",
            source=recording,
            path=recording,
            duration_s=0.0,
            speed=1.0,
            loop=False,
        ),
        mocker.call("Replay finished", source=recording),
    ]
