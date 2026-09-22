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

from dimos_generated.geometry_msgs.msg import PoseStamped, Twist
from dimos_generated.std_msgs.msg import Header
import pytest
from pytest_mock import MockerFixture

from dimos.core.global_config import global_config
from dimos.core.stream import Transport
from dimos.memory import tap
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tap import TransportRecorder
from dimos.msgs.time import time_from_seconds


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
    odom.publish(PoseStamped(header=Header(stamp=time_from_seconds(1.0))))
    odom.publish(PoseStamped(header=Header(stamp=time_from_seconds(2.0))))
    goal.publish(PoseStamped(header=Header(stamp=time_from_seconds(3.0))))
    assert unsub is not None
    unsub()
    odom.publish(PoseStamped(header=Header(stamp=time_from_seconds(4.0))))
    rec.close()
    store.stop()

    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    assert store.list_streams() == ["odom"]
    assert [o.ts for o in store.stream("odom", PoseStamped)] == [1.0, 2.0]
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
    odom.publish(PoseStamped(header=Header(stamp=time_from_seconds(1.0))))
    odom.publish(PoseStamped(header=Header(stamp=time_from_seconds(2.0))))
    odom.publish(PoseStamped(header=Header(stamp=time_from_seconds(3.0))))
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
    odom.publish(PoseStamped(header=Header(stamp=time_from_seconds(2.0))))
    rec.close()  # returns: the writer is still draining
    store.stop()

    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    assert [o.ts for o in store.stream("odom", PoseStamped)] == [2.0]
    store.stop()


def test_recording_fails_loudly_when_no_stream_matches(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(tap, "RECORDINGS_DIR", tmp_path)
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_topics", "color_image2")
    with pytest.raises(ValueError, match="matched none of: lidar, odom"):
        with tap.recording(
            {("odom", PoseStamped): _Transport(), ("lidar", PoseStamped): _Transport()}
        ):
            pass
    assert not any(tmp_path.rglob("memory.db"))


def test_recording_delegates_to_rust_session(
    monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> None:
    from dimos.experimental.memory import rust_cli_recorder

    transport = _Transport()
    transports: dict[tuple[str, type], Transport[Any]] = {("odom", PoseStamped): transport}
    session = mocker.Mock()
    make_plan = mocker.patch.object(rust_cli_recorder, "make_plan", return_value="plan")
    create_session = mocker.patch.object(
        rust_cli_recorder, "RustRecordingSession", return_value=session
    )
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_engine", "rust")
    monkeypatch.setattr(global_config, "record_topics", "*")

    with tap.recording(transports):
        session.start.assert_called_once_with()

    make_plan.assert_called_once_with(transports)
    create_session.assert_called_once_with("plan")
    session.stop.assert_called_once_with()


def test_zero_source_stamp_and_unstamped_arrival_time_are_distinct(tmp_path, monkeypatch):
    monkeypatch.setattr(tap.time, "time", lambda: 123.5)
    path = tmp_path / "time.db"
    pose_transport, twist_transport = _Transport(), _Transport()
    with SqliteStore(path=str(path)) as store:
        recorder = TransportRecorder(store)
        subscriptions = []
        try:
            subscriptions.append(recorder.tap("pose", PoseStamped, pose_transport))
            subscriptions.append(recorder.tap("twist", Twist, twist_transport))
            pose_transport.publish(PoseStamped())
            twist_transport.publish(Twist())
        finally:
            for unsubscribe in subscriptions:
                assert unsubscribe is not None
                unsubscribe()
            recorder.close()
    with SqliteStore(path=str(path), must_exist=True) as store:
        pose = store.stream("pose").first()
        twist = store.stream("twist").first()
        assert pose.ts == 0.0
        assert pose.data == PoseStamped()
        assert twist.ts == 123.5
        assert twist.data == Twist()
