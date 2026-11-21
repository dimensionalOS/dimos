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

from contextlib import ExitStack, closing
from pathlib import Path
import threading
import time
from typing import Any
from unittest import mock

import pytest
from pytest_mock import MockerFixture
from reactivex import interval
from reactivex.disposable import Disposable
from reactivex.subject import Subject
from reactivex.testing import TestScheduler

from dimos.core.stream import Out, Transport
from dimos.memory import replay as replay_mod, replay_progress
from dimos.memory.replay import ReplayStream
from dimos.memory.replay_module import replay_module
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tap import TransportRecorder
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.rpc.zenohrpc import ZenohRPC


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
    deadline = time.time() + 5
    while time.time() < deadline and len(got["odom"]) < 3:
        time.sleep(0.01)
    module.stop()
    assert [round(t, 3) for t in got["odom"]] == [1.0, 1.1, 1.2]
    assert [round(t, 3) for t in got["goal"]] == [1.05]


def test_stop_waits_for_in_flight_decode(recording: str, monkeypatch: pytest.MonkeyPatch) -> None:
    entered = threading.Event()
    release = threading.Event()
    decoded = threading.Event()
    stopping = threading.Event()
    subscriptions_disposed = threading.Event()
    stopped = threading.Event()
    errors: list[Exception] = []
    decode = ReplayStream._decode

    def held_decode(self: ReplayStream[Any], obs: Any) -> Any:
        entered.set()
        try:
            assert release.wait(5), "decode was not released"
            data = decode(self, obs)
            decoded.set()
            return data
        except Exception as exc:
            errors.append(exc)
            raise

    monkeypatch.setattr(ReplayStream, "_decode", held_decode)
    module = replay_module(recording, topics="odom")(dataset=recording, topics="odom")
    module.register_disposable(Disposable(stopping.set))

    def stop() -> None:
        try:
            module.stop()
        except Exception as exc:
            errors.append(exc)
        finally:
            stopped.set()

    thread = threading.Thread(target=stop)
    try:
        module.start()
        module.register_disposable(Disposable(subscriptions_disposed.set))
        assert entered.wait(5), "no emission reached decode"
        thread.start()
        assert stopping.wait(5), "stop did not reach subscription disposal"
        assert not subscriptions_disposed.wait(0.1), "disposal did not wait for the active decode"
    finally:
        release.set()
        if thread.ident is not None:
            thread.join(timeout=5)
        module.stop()

    assert stopped.is_set()
    assert subscriptions_disposed.is_set()
    assert decoded.is_set()
    assert errors == []
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
    deadline = time.time() + 5
    while time.time() < deadline and (got["camera_info"] < 2 or got["odom"] < 1):
        time.sleep(0.05)
    module.stop()
    assert got["camera_info"] >= 2  # republished at 1 Hz
    assert got["odom"] == 1  # arrived within the 5 s deadline, not 39 s later


def test_constant_timestamp_tf_republishes_every_edge(sqlite_store, mocker):
    chain = TFMessage(
        Transform(frame_id="base", child_frame_id="camera", ts=42.25),
        Transform(frame_id="camera", child_frame_id="optical", ts=42.25),
    )
    with closing(TransportRecorder(sqlite_store)) as recorder, ExitStack() as subscriptions:
        transport = mocker.Mock(spec=Transport)
        unsubscribe = recorder.tap("tf", TFMessage, transport)
        assert unsubscribe is not None
        subscriptions.callback(unsubscribe)
        on_message = transport.subscribe.call_args.args[0]
        on_message(chain)
        on_message(chain)

    mocker.patch("dimos.core.module.get_loop", return_value=(None, None))
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    emitted = []
    path = sqlite_store.config.path
    with Subject() as ticks:
        mocker.patch.object(replay_progress, "interval", return_value=ticks)
        with replay_module(path)(dataset=path, rpc_transport=ZenohRPC, loop=True) as module:
            module.register_disposable(Disposable(module.outputs["tf"].subscribe(emitted.append)))
            ticks.on_next(0)
            ticks.on_next(1)

    assert [
        [(t.frame_id, t.child_frame_id, t.ts) for t in message.transforms] for message in emitted
    ] == [[("base", "camera", 42.25), ("camera", "optical", 42.25)]] * 2


@pytest.fixture
def mixed_tf_recording(sqlite_store: SqliteStore, mocker: MockerFixture) -> SqliteStore:
    with closing(TransportRecorder(sqlite_store)) as recorder, ExitStack() as subscriptions:
        transport = mocker.Mock(spec=Transport)
        unsubscribe = recorder.tap("tf", TFMessage, transport)
        assert unsubscribe is not None
        subscriptions.callback(unsubscribe)
        on_message = transport.subscribe.call_args.args[0]
        for ts in (1000.0, 1001.0):
            on_message(
                TFMessage(
                    Transform(frame_id="world", child_frame_id="base", ts=ts),
                    Transform(frame_id="base", child_frame_id="camera", ts=10.0),
                )
            )
            # Sort after TF to also catch priming that delays another subscription.
            sqlite_store.stream("z_odom", PoseStamped).append(PoseStamped(ts=ts), ts=ts)

    return sqlite_store


def test_mixed_tf_starts_dynamic_playback_without_losing_frozen_edges(
    mixed_tf_recording: SqliteStore, mocker: MockerFixture
) -> None:
    scheduler = TestScheduler()
    preparing = True
    decode = ReplayStream._decode

    def slow_decode(stream, observation):
        if preparing and observation.ts == 10.0:
            scheduler.sleep(0.1)  # Consume setup time without running queued callbacks.
        return decode(stream, observation)

    mocker.patch.object(ReplayStream, "_decode", side_effect=slow_decode, autospec=True)
    mocker.patch.object(replay_mod.time, "time", side_effect=lambda: scheduler.clock)
    mocker.patch.object(replay_mod, "TimeoutScheduler", return_value=scheduler)
    mocker.patch.object(
        replay_progress, "interval", side_effect=lambda period: interval(period, scheduler)
    )
    mocker.patch("dimos.core.module.get_loop", return_value=(None, None))
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    odom = []
    tf = []
    path = mixed_tf_recording.config.path
    with ExitStack() as cleanup:
        module = replay_module(path)(dataset=path, rpc_transport=ZenohRPC, loop=True)
        cleanup.callback(module.stop)
        module.register_disposable(Disposable(module.outputs["z_odom"].subscribe(odom.append)))
        module.register_disposable(
            Disposable(
                module.outputs["tf"].subscribe(lambda message: tf.extend(message.transforms))
            )
        )
        module.start()
        preparing = False
        started = scheduler.clock
        scheduler.advance_to(started + 0.01)
        assert [m.ts for m in odom] == [1000.0]
        assert [(t.frame_id, t.child_frame_id, t.ts) for t in tf] == [
            ("base", "camera", 10.0),
            ("world", "base", 1000.0),
        ]
        scheduler.advance_to(started + 1.01)
        assert [m.ts for m in odom] == [1000.0, 1001.0]
        assert [t.ts for t in tf if t.frame_id == "world"] == [1000.0, 1001.0]
        scheduler.advance_to(started + 2.06)
        assert [m.ts for m in odom] == [1000.0, 1001.0] * 2
        assert [t.ts for t in tf if t.frame_id == "world"] == [1000.0, 1001.0] * 2
        assert [t.ts for t in tf if t.frame_id == "base"] == [10.0] * 3


@pytest.mark.parametrize(
    ("seek", "duration", "expected_odom", "expected_tf"),
    [
        (None, 1.0, [999.5, 1000.0], [1000.0]),
        (0.5, None, [1000.0, 1001.0], [1000.0, 1001.0]),
        (990.0, 1.0, [], []),
    ],
)
def test_mixed_tf_window_starts_at_first_timed_row(
    mixed_tf_recording: SqliteStore,
    mocker: MockerFixture,
    seek: float | None,
    duration: float | None,
    expected_odom: list[float],
    expected_tf: list[float],
) -> None:
    # Odometry starts half a second before changing TF. Both share the same window.
    mixed_tf_recording.stream("z_odom", PoseStamped).append(PoseStamped(ts=999.5), ts=999.5)
    scheduler = TestScheduler()
    mocker.patch.object(replay_mod, "TimeoutScheduler", return_value=scheduler)
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    path = mixed_tf_recording.config.path
    module = replay_module(path)(dataset=path, rpc_transport=ZenohRPC, seek=seek, duration=duration)
    tf: list[Transform] = []
    odom: list[PoseStamped] = []
    with ExitStack() as cleanup:
        cleanup.callback(module.stop)
        cleanup.callback(module.outputs["tf"].subscribe(lambda msg: tf.extend(msg.transforms)))
        cleanup.callback(module.outputs["z_odom"].subscribe(odom.append))
        module.start()
        scheduler.advance_by(2)
        assert [msg.ts for msg in odom] == expected_odom
        assert [t.ts for t in tf if t.child_frame_id == "base"] == expected_tf
        assert (tf[0].child_frame_id, tf[0].ts) == ("camera", 10.0)


def test_seek_on_late_topic_uses_recording_origin(
    mixed_tf_recording: SqliteStore, mocker: MockerFixture
) -> None:
    late = mixed_tf_recording.stream("late", PoseStamped)
    for ts in (1010.0, 1010.25):
        late.append(PoseStamped(ts=ts), ts=ts)

    scheduler = TestScheduler()
    mocker.patch.object(replay_mod, "TimeoutScheduler", return_value=scheduler)
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    path = mixed_tf_recording.config.path
    cls = replay_module(path, topics="late")
    module = cls(dataset=path, topics="late", seek=10.0, duration=0.5, rpc_transport=ZenohRPC)
    received: list[float] = []
    with Disposable(module.outputs["late"].subscribe(lambda msg: received.append(msg.ts))), module:
        scheduler.advance_by(1)
    assert received == [1010.0, 1010.25]


def test_legacy_tf_batches_publish_fixed_edges_once_at_start(sqlite_store, mocker):
    rows = sqlite_store.stream("tf", TFMessage)
    fixed = Transform(frame_id="base", child_frame_id="camera", ts=0.0)
    for ts in (10.0, 11.0):
        rows.append(
            TFMessage(fixed, Transform(frame_id="world", child_frame_id="base", ts=ts)), ts=ts
        )
    scheduler = TestScheduler()
    mocker.patch.object(replay_mod.time, "time", side_effect=lambda: scheduler.clock)
    mocker.patch.object(replay_mod, "TimeoutScheduler", return_value=scheduler)
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    path = sqlite_store.config.path
    seen = []
    with ExitStack() as cleanup:
        module = replay_module(path)(dataset=path, rpc_transport=ZenohRPC)
        cleanup.callback(module.stop)
        module.register_disposable(Disposable(module.outputs["tf"].subscribe(seen.append)))
        module.start()
        scheduler.advance_to(0.01)
        assert [[t.child_frame_id for t in msg.transforms] for msg in seen] == [
            ["camera"],
            ["base"],
        ]


def test_slow_first_decode_does_not_skip_other_streams(
    recording: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Anchor is pinned after setup; a stream that is slow to prime must not make later streams late."""
    from dimos.memory import replay as replay_mod

    real = replay_mod.ReplayStream._decode

    def slow(self: Any, obs: Any) -> Any:
        if obs.ts == 1.05:  # the goal frame, subscribed first (sorted order)
            time.sleep(0.2)
        return real(self, obs)

    monkeypatch.setattr(replay_mod.ReplayStream, "_decode", slow)
    module = replay_module(recording)(dataset=recording)
    got: dict[str, list[float]] = {"odom": [], "goal": []}
    module.outputs["odom"].subscribe(lambda m: got["odom"].append(m.ts))
    module.outputs["goal"].subscribe(lambda m: got["goal"].append(m.ts))
    module.start()
    deadline = time.time() + 5
    while time.time() < deadline and (len(got["odom"]) < 3 or not got["goal"]):
        time.sleep(0.01)
    module.stop()
    assert [round(t, 3) for t in got["odom"]] == [1.0, 1.1, 1.2]


def test_slow_stream_setup_does_not_skip_the_first_frames(recording: str) -> None:
    """Slow setup should not lose initial frames."""
    real = ReplayStream._base_stream

    def slow(self: Any) -> Any:
        time.sleep(0.1)  # twice the late tolerance, on every query a subscribe makes
        return real(self)

    got: dict[str, list[float]] = {"odom": [], "goal": []}
    with mock.patch("dimos.memory.replay.ReplayStream._base_stream", slow):
        module = replay_module(recording)(dataset=recording)
        module.outputs["odom"].subscribe(lambda m: got["odom"].append(m.ts))
        module.outputs["goal"].subscribe(lambda m: got["goal"].append(m.ts))
        module.start()
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
    scheduler = TestScheduler()
    mocker.patch("dimos.memory.replay.TimeoutScheduler", return_value=scheduler)
    log = mocker.patch("dimos.memory.replay_progress.logger")
    module = replay_module(recording)(dataset=recording)
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
        scheduler.advance_by(0.1)  # goal has ended; odom still has a frame to publish
        assert log.info.call_count == 1
        scheduler.advance_by(1)
        assert log.info.call_args_list[1:] == [mocker.call("Replay finished", source=recording)]
    finally:
        module.stop()


def test_replay_with_only_static_metadata_reports_completion(recording, mocker):
    with SqliteStore(path=recording) as store:
        for _ in range(2):
            store.stream("camera_info", PoseStamped).append(PoseStamped(ts=0.0), ts=0.0)
    scheduler = TestScheduler()
    mocker.patch(
        "dimos.memory.replay_progress.interval",
        side_effect=lambda period: interval(period, scheduler=scheduler),
    )
    log = mocker.patch("dimos.memory.replay_progress.logger")
    module = replay_module(recording, topics="camera_info")(dataset=recording, topics="camera_info")
    received = []
    with Disposable(module.outputs["camera_info"].subscribe(received.append)), module:
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
