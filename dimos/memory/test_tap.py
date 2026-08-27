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

import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tap import TransportRecorder, open_record_store
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


class _Transport:
    """Any dimos Transport: subscribe(callback) -> unsubscribe."""

    def __init__(self) -> None:
        self.callbacks: list[Callable[[Any], None]] = []

    def subscribe(
        self, callback: Callable[[Any], None], selfstream: Any = None
    ) -> Callable[[], None]:
        self.callbacks.append(callback)
        return lambda: self.callbacks.remove(callback)

    def publish(self, msg: Any) -> None:
        for cb in self.callbacks:
            cb(msg)


def test_taps_matching_dimos_streams(tmp_path: Path) -> None:
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    rec = TransportRecorder(store, topics="odom,lidar")
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


def test_mcap_recording_round_trips(tmp_path: Path) -> None:
    pytest.importorskip("mcap")
    from dimos.memory.store.mcap import McapStore

    store, path = open_record_store(tmp_path, "mcap")
    store.start()
    rec = TransportRecorder(store)
    odom = _Transport()
    rec.tap("odom", PoseStamped, odom)
    odom.publish(PoseStamped(ts=1.0))
    odom.publish(PoseStamped(ts=2.0))
    rec.close()
    store.stop()

    # No codec map: the schema name written by McapWriteStore is enough to decode.
    read = McapStore(path=str(path), codecs={})
    assert read.list_streams() == ["odom"]
    observations = list(read.stream("odom"))
    assert [o.ts for o in observations] == [1.0, 2.0]
    assert all(isinstance(o.data, PoseStamped) for o in observations)
    read.stop()


def test_full_queue_drops_and_counts(tmp_path: Path) -> None:
    store = SqliteStore(path=str(tmp_path / "memory.db"))
    store.start()
    rec = TransportRecorder(store, queue_size=1)
    rec._queue.put(None)  # stop the writer so nothing drains
    rec._writer.join()
    odom = _Transport()
    rec.tap("odom", PoseStamped, odom)
    odom.publish(PoseStamped(ts=1.0))
    odom.publish(PoseStamped(ts=2.0))
    assert rec.dropped == 1
    store.stop()
