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

from collections.abc import Callable
from contextlib import ExitStack, closing
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.stream import In, Transport
from dimos.experimental.memory import rust_cli_recorder
from dimos.memory import tap
from dimos.memory.module import Recorder
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tap import TransportRecorder
from dimos.memory.type import recording
from dimos.msgs.control_msgs.ControlValues import ControlValues
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointCommand import JointCommand
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.rpc.zenohrpc import ZenohRPC


class _Transport(Transport[Any]):
    def __init__(self) -> None:
        self.callbacks: list[Callable[[Any], None]] = []

    def subscribe(
        self, callback: Callable[[Any], None], selfstream: Any = None
    ) -> Callable[[], None]:
        self.callbacks.append(callback)
        return lambda: self.callbacks.remove(callback)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def broadcast(self, selfstream: Any, msg: Any) -> None:
        for cb in self.callbacks:
            cb(msg)


def test_taps_matching_dimos_streams(tmp_path: Path) -> None:
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    rec = TransportRecorder(store, topics="/odom, lidar")
    odom, goal, raw = _Transport(), _Transport(), _Transport()
    unsub = rec.tap("odom", PoseStamped, odom)
    assert rec.tap("goal", PoseStamped, goal) is None  # filtered out
    assert rec.tap("lidar", dict, raw) is None  # not a dimos message type
    odom.publish(PoseStamped(ts=1.0))
    odom.publish(PoseStamped(ts=2.0))
    goal.publish(PoseStamped(ts=3.0))
    assert unsub is not None
    unsub()
    odom.publish(PoseStamped(ts=4.0))
    rec.close()
    store.stop()

    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    assert store.list_streams() == ["odom"]
    assert [o.ts for o in store.stream("odom", PoseStamped)] == [1.0, 2.0]
    store.stop()


@pytest.mark.parametrize("ts", [10, 10.0])
def test_records_tf_with_numeric_timestamp(tmp_path: Path, ts: float) -> None:
    path = tmp_path / "memory.db"
    transform = Transform(ts=ts, frame_id="world", child_frame_id="base")
    transport = _Transport()
    with SqliteStore(path=str(path)) as store:
        with closing(TransportRecorder(store)) as recorder, ExitStack() as subscriptions:
            unsubscribe = recorder.tap("tf", TFMessage, transport)
            assert unsubscribe is not None
            subscriptions.callback(unsubscribe)
            transport.publish(TFMessage(transform))
        assert recorder.dropped == 0

    with SqliteStore(path=str(path), must_exist=True) as store:
        rows = store.stream("tf", TFMessage).to_list()
        assert len(rows) == 1
        decoded = rows[0].data.transforms
        assert decoded == [transform]
        assert decoded[0].ts == ts
        assert decoded[0].frame_id == "world"
        assert decoded[0].child_frame_id == "base"


def test_depth_is_recorded_losslessly_and_colour_as_jpeg(tmp_path: Path) -> None:
    pytest.importorskip("turbojpeg")
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    rec = TransportRecorder(store)
    mm = np.arange(48 * 64, dtype=np.uint16).reshape(48, 64) * 7
    metres = mm.astype(np.float32) / 1000
    frames = {
        "depth16": Image(data=mm, format=ImageFormat.DEPTH16, frame_id="depth_optical", ts=1.0),
        "depth": Image(data=metres, format=ImageFormat.DEPTH, frame_id="depth_optical", ts=1.0),
        "color": Image(data=np.zeros((48, 64, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=1.0),
    }
    for name, frame in frames.items():
        transport = _Transport()
        rec.tap(name, Image, transport)
        transport.publish(frame)
    rec.close()
    store.stop()

    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    codecs = {name: store._registry.get(name)["codec_id"] for name in frames}  # type: ignore[index]
    assert codecs == {"depth16": "lz4+lcm", "depth": "lz4+lcm", "color": "jpeg"}
    for name in ("depth16", "depth"):
        decoded = store.stream(name, Image).first().data
        assert decoded.format == frames[name].format
        np.testing.assert_array_equal(decoded.data, frames[name].data)
    store.stop()


def test_full_queue_drops_and_counts(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    warnings: list[str] = []
    monkeypatch.setattr(tap.logger, "warning", lambda msg, *args: warnings.append(msg % args))
    store = SqliteStore(path=str(tmp_path / "memory.db"))
    store.start()
    rec = TransportRecorder(store, queue_size=1)
    rec._queue.put(None)  # stop the writer so nothing drains
    rec._writer.join()
    odom = _Transport()
    rec.tap("odom", PoseStamped, odom)
    odom.publish(PoseStamped(ts=1.0))
    odom.publish(PoseStamped(ts=2.0))
    odom.publish(PoseStamped(ts=3.0))
    assert rec.dropped == 2
    assert len(warnings) == 1
    store.stop()


def test_append_failure_does_not_kill_writer(tmp_path: Path) -> None:
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    rec = TransportRecorder(store)
    odom = _Transport()
    rec.tap("odom", PoseStamped, odom)
    odom.publish("not a PoseStamped")  # append raises TypeError inside the writer
    odom.publish(PoseStamped(ts=2.0))
    rec.close()  # returns: the writer is still draining
    store.stop()

    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    assert [o.ts for o in store.stream("odom", PoseStamped)] == [2.0]
    store.stop()


def test_recording_fails_loudly_when_no_stream_matches(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> None:
    monkeypatch.setattr(tap, "RECORDINGS_DIR", tmp_path)
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_topics", "color_image2")
    coordinator = mocker.Mock(
        spec=ModuleCoordinator,
        transports={("odom", PoseStamped): _Transport(), ("lidar", PoseStamped): _Transport()},
    )
    with pytest.raises(ValueError, match="matched none of: lidar, odom"):
        with tap.recording(coordinator):
            pass
    assert not any(tmp_path.rglob("memory.db"))


def test_recording_delegates_to_rust_session(
    monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> None:
    transport = _Transport()
    transports: dict[tuple[str, type], Transport[Any]] = {("odom", PoseStamped): transport}
    coordinator = mocker.Mock(spec=ModuleCoordinator, transports=transports)
    session = mocker.Mock()
    make_plan = mocker.patch.object(rust_cli_recorder, "make_plan", return_value="plan")
    create_session = mocker.patch.object(
        rust_cli_recorder, "RustRecordingSession", return_value=session
    )
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_engine", "rust")
    monkeypatch.setattr(global_config, "record_topics", "*")

    with tap.recording(coordinator):
        session.start.assert_called_once_with()

    make_plan.assert_called_once_with(transports)
    create_session.assert_called_once_with("plan")
    session.stop.assert_called_once_with()


def test_records_tf_and_odom_at_source_time(sqlite_store):
    ts = 42.25
    older = Transform(ts=ts, frame_id="base", child_frame_id="sensor")
    batches = [
        TFMessage(Transform(ts=ts + 1.0, child_frame_id="base"), older),
        TFMessage(Transform(ts=ts + 2.0, child_frame_id="base"), older),
    ]
    with closing(TransportRecorder(sqlite_store)) as recorder, ExitStack() as subscriptions:
        tf_transport, odom_transport = _Transport(), _Transport()
        for name, msg_type, transport in [
            ("tf", TFMessage, tf_transport),
            ("odom", PoseStamped, odom_transport),
        ]:
            unsubscribe = recorder.tap(name, msg_type, transport)
            assert unsubscribe is not None
            subscriptions.callback(unsubscribe)
        for batch in batches:
            tf_transport.publish(batch)
        odom_transport.publish(PoseStamped(ts=ts))

    rows = sqlite_store.stream("tf", TFMessage).order_by("id").to_list()
    assert [row.ts for row in rows] == [ts + 1.0, ts, ts + 2.0, ts]
    assert [row.data.lcm_encode() for row in rows] == [
        TFMessage(transform).lcm_encode() for batch in batches for transform in batch
    ]
    assert sqlite_store.stream("odom", PoseStamped).first().ts == ts
    # Repeated older members must not make the changing TF stream look static.
    replay = sqlite_store.replay().stream("tf")
    assert (replay.first_ts(), replay.last_ts()) == (ts, ts + 2.0)


def test_unset_stamps_share_the_wall_clock_replay_window(sqlite_store, mocker):
    wall = 1_800_000_000.0
    mocker.patch.object(tap.time, "time", return_value=wall)
    with closing(TransportRecorder(sqlite_store)) as recorder:
        odom, frames = _Transport(), _Transport()
        recorder.tap("odom", PoseStamped, odom)
        recorder.tap("tf", TFMessage, frames)
        odom.publish(PoseStamped(ts=0.0))
        frames.publish(TFMessage(Transform(ts=0.0), Transform(ts=wall + 0.25)))

    replay = sqlite_store.replay(duration=1.0)
    replay.pin_anchor()
    assert replay.first_ts() == wall
    assert replay._anchor is not None and replay._anchor[1] == wall
    assert [(ts, pose.ts) for ts, pose in replay.stream("odom").iterate_ts()] == [(wall, 0.0)]
    assert [(ts, batch.transforms[0].ts) for ts, batch in replay.stream("tf").iterate_ts()] == [
        (wall, 0.0),
        (wall + 0.25, wall + 0.25),
    ]


def test_zero_stamps_index_at_arrival_and_warn_once_per_topic(sqlite_store, mocker):
    mocker.patch.object(tap.time, "time", return_value=99.0)
    warning = mocker.patch.object(recording.logger, "warning")
    with closing(TransportRecorder(sqlite_store)) as recorder:
        odom, pose = _Transport(), _Transport()
        recorder.tap("odom", PoseStamped, odom)
        recorder.tap("pose", PoseStamped, pose)
        for _ in range(3):
            odom.publish(PoseStamped(ts=0.0))
        pose.publish(PoseStamped(ts=0.0))

    assert [row.ts for row in sqlite_store.stream("odom", PoseStamped)] == [99.0] * 3
    assert sorted(call.args[1] for call in warning.call_args_list) == ["odom", "pose"]


@pytest.mark.parametrize(
    "message",
    [
        Header(0.25),
        JointCommand(timestamp=0.25),
        ControlValues("test", source_ts=0.25),
        String("unstamped"),
    ],
)
def test_tap_indexes_other_source_clocks_and_unstamped_messages(sqlite_store, message, mocker):
    transport = _Transport()
    mocker.patch.object(tap.time, "time", return_value=99.0)
    with closing(TransportRecorder(sqlite_store)) as recorder:
        recorder.tap("samples", type(message), transport)
        transport.publish(message)
    row = sqlite_store.stream("samples", type(message)).first()
    assert row.ts == (99.0 if isinstance(message, String) else 0.25)


class _SourceRecorder(Recorder):
    odom: In[PoseStamped]
    frames: In[TFMessage]


def test_module_recorder_preserves_source_and_arrival_clocks(tmp_path, mocker, wait_until):
    path = tmp_path / "memory.db"
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    module = _SourceRecorder(
        db_path=path,
        record_tf=True,
        poseless_streams=["frames", "odom"],
        rpc_transport=ZenohRPC,
    )
    module.odom.transport = _Transport()
    module.frames.transport = _Transport()
    module.tf.transport = _Transport()
    with module:
        module.odom.transport.publish(PoseStamped(ts=0.0))
        batch = TFMessage(
            Transform(ts=0.0, frame_id="world", child_frame_id="base"),
            Transform(ts=42.25, frame_id="base", child_frame_id="camera"),
        )
        module.frames.transport.publish(batch)
        module.tf.transport.publish(batch)
        wait_until(
            lambda: module.store.stream("odom").count() == 1
            and module.store.stream("frames").count() == 2,
            timeout=3,
        )
    with SqliteStore(path=str(path), must_exist=True) as store:
        odom = store.stream("odom").first()
        assert odom.ts == odom.tags["reception_ts"]
        assert odom.data.ts == 0.0
        frames = store.stream("frames").order_by("id").to_list()
        assert [row.ts for row in frames] == [frames[0].tags["reception_ts"], 42.25]
        assert [row.data.transforms[0].ts for row in frames] == [0.0, 42.25]
        assert frames[0].tags["reception_ts"] == frames[1].tags["reception_ts"]
        dedicated = store.stream("tf", TFMessage).order_by("id").to_list()
        assert [row.data.transforms[0].ts for row in dedicated] == [0.0, 42.25]
        assert dedicated[1].ts == 42.25
        assert dedicated[0].ts > 42.25
