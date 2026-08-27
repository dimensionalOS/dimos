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

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tap import BusRecorder
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.protocol.pubsub.impl.lcmpubsub import Topic


def test_records_matching_dimos_topics(tmp_path: Path) -> None:
    path = tmp_path / "memory.db"
    store = SqliteStore(path=str(path))
    store.start()
    rec = BusRecorder(store, topics="world_odom,lidar")
    rec.on_message(PoseStamped(ts=1.0), Topic("/world/odom", PoseStamped))
    rec.on_message(PoseStamped(ts=2.0), Topic("/world/odom", PoseStamped))
    rec.on_message(PoseStamped(ts=3.0), Topic("/goal", PoseStamped))  # filtered out
    rec.on_message(b"raw", Topic("/lidar"))  # no dimos type: skipped
    store.stop()

    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    assert store.list_streams() == ["world_odom"]
    assert [o.ts for o in store.stream("world_odom", PoseStamped)] == [1.0, 2.0]
    store.stop()
