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
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.stream import Transport
from dimos.experimental.memory import rust_cli_recorder
from dimos.memory import tap
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tap import TransportRecorder
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.perception_msgs.EmbeddedFrame import EmbeddedFrame
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


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


def test_embedded_frames_are_recorded_with_frame_timestamps(tmp_path: Path) -> None:
    path = tmp_path / "memory.db"
    first = EmbeddedFrame(
        embedding=np.array([0.25, -0.5, 1.0], dtype=np.float32),
        sharpness=12.0,
        embedder="siglip",
        embed_hz=1.0,
        frame_id="cam",
        ts=10.5,
    )
    second = EmbeddedFrame(
        embedding=np.array([0.75, 0.125, -0.25], dtype=np.float32),
        sharpness=18.5,
        embedder="siglip",
        embed_hz=1.0,
        frame_id="cam",
        ts=11.5,
    )
    transport = _Transport()
    with SqliteStore(path=str(path)) as store:
        rec = TransportRecorder(store)
        try:
            rec.tap("frame_embeddings", EmbeddedFrame, transport)
            transport.publish(first)
            transport.publish(second)
        finally:
            rec.close()

    with SqliteStore(path=str(path), must_exist=True) as store:
        observations = store.stream("frame_embeddings", EmbeddedFrame).to_list()
        assert [obs.ts for obs in observations] == [10.5, 11.5]
        assert [obs.data.ts for obs in observations] == [10.5, 11.5]
        assert np.array_equal(observations[0].data.embedding, first.embedding)
        assert np.array_equal(observations[1].data.embedding, second.embedding)
