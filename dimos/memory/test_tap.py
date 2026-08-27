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

import os
from pathlib import Path

import pytest

from dimos.core.global_config import global_config
from dimos.core.stream import Out
from dimos.memory import tap
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


class _Topic:
    def __init__(self, topic: str) -> None:
        self.topic = topic

    def __str__(self) -> str:
        return f"{self.topic}#some.Type"


class _Transport:
    def __init__(self, topic: str) -> None:
        self.topic = _Topic(topic)
        self.sent: list[object] = []

    def broadcast(self, _: object, msg: object) -> None:
        self.sent.append(msg)


@pytest.fixture
def recording(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    monkeypatch.setattr(tap, "RECORDINGS_DIR", tmp_path)
    monkeypatch.setattr(tap, "_store", None)
    monkeypatch.setattr(tap, "_streams", {})
    monkeypatch.setenv("DIMOS_RUN_ID", "run1")
    monkeypatch.setattr(global_config, "record", "sqlite")
    monkeypatch.setattr(global_config, "record_topics", "world_odom,lidar")
    return tmp_path / "run1" / f"memory-{os.getpid()}.db"


def test_publish_records_matching_topics(recording: Path) -> None:
    odom: Out[PoseStamped] = Out(PoseStamped, "odom", transport=_Transport("/world/odom"))
    other: Out[PoseStamped] = Out(PoseStamped, "goal", transport=_Transport("/goal"))
    odom.publish(PoseStamped(ts=1.0))
    odom.publish(PoseStamped(ts=2.0))
    other.publish(PoseStamped(ts=3.0))
    raw: Out[dict[str, str]] = Out(dict, "lidar", transport=_Transport("/lidar"))
    raw.publish({"not": "a message"})
    assert tap._store is not None
    tap._store.stop()

    store = SqliteStore(path=str(recording), must_exist=True)
    store.start()
    assert store.list_streams() == ["world_odom"]
    assert [o.ts for o in store.stream("world_odom", PoseStamped)] == [1.0, 2.0]
    store.stop()


def test_off_by_default(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(tap, "RECORDINGS_DIR", tmp_path)
    monkeypatch.setattr(tap, "_store", None)
    Out(PoseStamped, "odom", transport=_Transport("/world/odom")).publish(PoseStamped())
    assert tap._store is None and not list(tmp_path.iterdir())
