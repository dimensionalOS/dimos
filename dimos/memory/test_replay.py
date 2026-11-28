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

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest

from dimos.memory.replay import ReplayStream
from dimos.memory.store.base import StreamAccessor
from dimos.memory.utils.time_window import recording_start

if TYPE_CHECKING:
    from dimos.memory.store.sqlite import SqliteStore

pytestmark = pytest.mark.self_hosted


def _populate(store: SqliteStore, name: str, timestamps: list[float]) -> None:
    """Append integer payloads at each given ts to a named stream."""
    s = store.stream(name, int)
    for i, ts in enumerate(timestamps):
        s.append(i, ts=ts)


def test_streams_accessor_equivalence(sqlite_store: SqliteStore) -> None:
    _populate(sqlite_store, "lidar", [0.0, 0.1, 0.2])
    replay = sqlite_store.replay()
    assert isinstance(replay.streams, StreamAccessor)
    by_attr = replay.streams.lidar
    by_method: ReplayStream[int] = replay.stream("lidar")
    assert isinstance(by_attr, ReplayStream)
    assert isinstance(by_method, ReplayStream)
    assert by_attr.name == by_method.name == "lidar"


def test_recording_start_across_streams(sqlite_store: SqliteStore) -> None:
    _populate(sqlite_store, "lidar", [10.0, 11.0])
    _populate(sqlite_store, "odom", [5.0, 6.0])
    assert recording_start(sqlite_store) == 5.0


def test_recording_start_empty_store(sqlite_store: SqliteStore) -> None:
    assert recording_start(sqlite_store) is None


def test_seek_filters_frames_before_offset(sqlite_store: SqliteStore) -> None:
    _populate(sqlite_store, "lidar", [100.0, 100.1, 100.2, 100.3])
    replay = sqlite_store.replay(seek=0.2)
    # seek=0.2 from first_ts=100.0 → window starts at 100.2 inclusive.
    pairs = list((o.ts, o.data) for o in replay.streams.lidar.rows())
    assert [v for _, v in pairs] == [2, 3]
    assert [ts for ts, _ in pairs] == pytest.approx([100.2, 100.3], abs=1e-6)


def test_duration_bounds_window(sqlite_store: SqliteStore) -> None:
    _populate(sqlite_store, "lidar", [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3])
    replay = sqlite_store.replay(duration=0.12)
    # time_range is inclusive on both sides: 0.0, 0.05, 0.10 are in; 0.15 is past.
    assert [v for _, v in ((o.ts, o.data) for o in replay.streams.lidar.rows())] == [0, 1, 2]


def test_replay_stream_rows(sqlite_store: SqliteStore) -> None:
    _populate(sqlite_store, "lidar", [10.0, 10.5, 11.0])
    replay = sqlite_store.replay()
    pairs = list((o.ts, o.data) for o in replay.streams.lidar.rows())
    assert pairs == [(10.0, 0), (10.5, 1), (11.0, 2)]


def test_replay_stream_count_respects_seek(sqlite_store: SqliteStore) -> None:
    _populate(sqlite_store, "lidar", [0.0, 1.0, 2.0, 3.0])
    replay = sqlite_store.replay(seek=1.0)
    assert replay.streams.lidar.count() == 3
