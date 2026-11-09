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
from dimos.message_utils import image as image_utils, time as message_time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.sensor_msgs.msg import Image
from dimos.messages.std_msgs.msg import Header


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
    odom.publish(
        PoseStamped(
            header=Header(stamp=message_time.time_from_seconds(1.0), frame_id=""),
            pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
        )
    )
    odom.publish(
        PoseStamped(
            header=Header(stamp=message_time.time_from_seconds(2.0), frame_id=""),
            pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
        )
    )
    goal.publish(
        PoseStamped(
            header=Header(stamp=message_time.time_from_seconds(3.0), frame_id=""),
            pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
        )
    )
    assert unsub is not None
    unsub()
    odom.publish(
        PoseStamped(
            header=Header(stamp=message_time.time_from_seconds(4.0), frame_id=""),
            pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
        )
    )
    rec.close()
    store.stop()

    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    assert store.list_streams() == ["odom"]
    assert [o.ts for o in store.stream("odom", PoseStamped)] == [1.0, 2.0]
    store.stop()


def test_generated_images_are_recorded_losslessly(tmp_path: Path) -> None:
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    rec = TransportRecorder(store)
    mm = np.arange(48 * 64, dtype=np.uint16).reshape(48, 64) * 7
    metres = mm.astype(np.float32) / 1000
    frames = {
        "depth16": image_utils.from_array(
            mm, encoding="16UC1", header=Header(message_time.time_from_seconds(1), "depth_optical")
        ),
        "depth": image_utils.from_array(
            metres,
            encoding="32FC1",
            header=Header(message_time.time_from_seconds(1), "depth_optical"),
        ),
        "color": image_utils.from_array(
            np.zeros((48, 64, 3), dtype=np.uint8),
            encoding="rgb8",
            header=Header(message_time.time_from_seconds(1), ""),
        ),
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
    assert codecs == {name: "cdr" for name in frames}
    for name in frames:
        decoded = store.stream(name, Image).first().data
        assert decoded.encoding == frames[name].encoding
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
    odom.publish(
        PoseStamped(
            header=Header(stamp=message_time.time_from_seconds(1.0), frame_id=""),
            pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
        )
    )
    odom.publish(
        PoseStamped(
            header=Header(stamp=message_time.time_from_seconds(2.0), frame_id=""),
            pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
        )
    )
    odom.publish(
        PoseStamped(
            header=Header(stamp=message_time.time_from_seconds(3.0), frame_id=""),
            pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
        )
    )
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
    odom.publish(
        PoseStamped(
            header=Header(stamp=message_time.time_from_seconds(2.0), frame_id=""),
            pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
        )
    )
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
