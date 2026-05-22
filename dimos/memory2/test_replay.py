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

"""Tests for the multi-stream replay over SqliteStore (shared anchor)."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import pytest

from dimos.memory2.replay import ReplayStream
from dimos.memory2.store.base import StreamAccessor

if TYPE_CHECKING:
    from dimos.memory2.store.sqlite import SqliteStore


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
    by_method = replay.stream("lidar")
    assert isinstance(by_attr, ReplayStream)
    assert isinstance(by_method, ReplayStream)
    assert by_attr.name == by_method.name == "lidar"


def test_first_ts_across_streams(sqlite_store: SqliteStore) -> None:
    _populate(sqlite_store, "lidar", [10.0, 11.0])
    _populate(sqlite_store, "odom", [5.0, 6.0])
    replay = sqlite_store.replay()
    assert replay.first_ts() == 5.0


def test_first_ts_empty_store(sqlite_store: SqliteStore) -> None:
    replay = sqlite_store.replay()
    assert replay.first_ts() is None


def test_seek_pins_anchor_post_seek(sqlite_store: SqliteStore) -> None:
    _populate(sqlite_store, "lidar", [100.0, 100.1, 100.2, 100.3])
    replay = sqlite_store.replay(seek=0.2)
    obs = replay.streams.lidar.observable()

    seen: list[int] = []
    sub = obs.subscribe(on_next=seen.append)
    time.sleep(0.2)
    sub.dispose()

    assert replay._anchor is not None
    _, replay_t0 = replay._anchor
    # seek=0.2 means anchor pins at first_ts + 0.2 = 100.2, not 100.0
    assert replay_t0 == pytest.approx(100.2, abs=1e-6)
    # first emission was index 2 (ts=100.2), then 3 (ts=100.3)
    assert seen == [2, 3]


def test_shared_anchor_two_streams_in_sync(sqlite_store: SqliteStore) -> None:
    # Both streams start at ts=0, sample every 50ms.
    timestamps = [i * 0.05 for i in range(8)]  # 0.00..0.35
    _populate(sqlite_store, "lidar", timestamps)
    _populate(sqlite_store, "odom", timestamps)

    replay = sqlite_store.replay()
    lidar_seen: list[tuple[float, int]] = []
    odom_seen: list[tuple[float, int]] = []

    start_wall = time.time()
    sub_l = replay.streams.lidar.observable().subscribe(
        on_next=lambda v: lidar_seen.append((time.time() - start_wall, v)),
    )
    sub_o = replay.streams.odom.observable().subscribe(
        on_next=lambda v: odom_seen.append((time.time() - start_wall, v)),
    )

    time.sleep(0.55)
    sub_l.dispose()
    sub_o.dispose()

    # Both should have received the full set, at roughly the same wall times.
    assert [v for _, v in lidar_seen] == list(range(8))
    assert [v for _, v in odom_seen] == list(range(8))

    # Each pair of same-index frames must be within ~30ms on the wall clock —
    # the shared anchor means msg_ts → wall_ts is the same function for both.
    for (wl, vl), (wo, vo) in zip(lidar_seen, odom_seen, strict=True):
        assert abs(wl - wo) < 0.03, f"frame {vl}/{vo} desync: lidar@{wl:.3f}, odom@{wo:.3f}"


def test_loop_per_stream_with_shared_anchor(sqlite_store: SqliteStore) -> None:
    # Short stream of 4 frames over 100ms. With loop=True we should see
    # multiple passes within 350ms, and the wall-clock gap between consecutive
    # emissions should not collapse to zero on wrap.
    _populate(sqlite_store, "lidar", [0.00, 0.03, 0.06, 0.09])

    replay = sqlite_store.replay(loop=True)
    seen: list[tuple[float, int]] = []
    start = time.time()
    sub = replay.streams.lidar.observable().subscribe(
        on_next=lambda v: seen.append((time.time() - start, v)),
    )
    time.sleep(0.35)
    sub.dispose()

    # Must have looped at least once — i.e. seen value 0 more than once.
    values = [v for _, v in seen]
    assert values.count(0) >= 2, f"never wrapped: values={values}"

    # No two consecutive emissions should fire at exactly the same wall time.
    gaps = [seen[i + 1][0] - seen[i][0] for i in range(len(seen) - 1)]
    assert all(g >= 0 for g in gaps)
    # Median gap should be roughly the inter-frame interval (~30ms), not 0.
    sorted_gaps = sorted(gaps)
    median = sorted_gaps[len(sorted_gaps) // 2]
    assert median >= 0.01, f"wrap collapsed timing: gaps={gaps}"


def test_replay_speed_multiplier(sqlite_store: SqliteStore) -> None:
    # 5 frames spanning 0..0.4s of recording. At 2x speed, full playback in ~0.2s.
    _populate(sqlite_store, "lidar", [0.0, 0.1, 0.2, 0.3, 0.4])
    replay = sqlite_store.replay(speed=2.0)
    seen: list[float] = []
    start = time.time()
    sub = replay.streams.lidar.observable().subscribe(
        on_next=lambda _v: seen.append(time.time() - start),
    )
    time.sleep(0.35)
    sub.dispose()
    assert len(seen) == 5
    # Last frame at ts=0.4 should play at wall ~0.2s (with speed=2x).
    assert seen[-1] < 0.3, f"speed=2 didn't compress: last wall={seen[-1]:.3f}"


def test_duration_bounds_playback(sqlite_store: SqliteStore) -> None:
    _populate(sqlite_store, "lidar", [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3])
    replay = sqlite_store.replay(duration=0.12)
    seen: list[int] = []
    sub = replay.streams.lidar.observable().subscribe(on_next=seen.append)
    time.sleep(0.35)
    sub.dispose()
    # duration=0.12 includes ts=0.0, 0.05, 0.10 (time_range inclusive on both sides).
    # ts=0.12 is the cutoff; 0.15 is past it.
    assert seen == [0, 1, 2]


def test_from_timestamp_absolute(sqlite_store: SqliteStore) -> None:
    _populate(sqlite_store, "lidar", [100.0, 100.1, 100.2, 100.3])
    replay = sqlite_store.replay(from_timestamp=100.2)
    seen: list[int] = []
    sub = replay.streams.lidar.observable().subscribe(on_next=seen.append)
    time.sleep(0.3)
    sub.dispose()
    assert seen == [2, 3]


def test_replay_stream_iterate_ts(sqlite_store: SqliteStore) -> None:
    _populate(sqlite_store, "lidar", [10.0, 10.5, 11.0])
    replay = sqlite_store.replay()
    pairs = list(replay.streams.lidar.iterate_ts())
    assert pairs == [(10.0, 0), (10.5, 1), (11.0, 2)]


def test_replay_stream_count(sqlite_store: SqliteStore) -> None:
    _populate(sqlite_store, "lidar", [0.0, 1.0, 2.0, 3.0])
    replay = sqlite_store.replay(seek=1.0)
    # seek=1.0 from first_ts=0 → starts at ts=1.0 inclusive → 3 frames.
    assert replay.streams.lidar.count() == 3


def test_anchor_reset(sqlite_store: SqliteStore) -> None:
    _populate(sqlite_store, "lidar", [0.0, 0.1])
    replay = sqlite_store.replay()
    sub = replay.streams.lidar.observable().subscribe(on_next=lambda _: None)
    time.sleep(0.05)
    sub.dispose()
    assert replay._anchor is not None
    replay.reset_anchor()
    assert replay._anchor is None


def test_replay_anchor_thread_safe(sqlite_store: SqliteStore) -> None:
    """Concurrent subscribes resolve to the same anchor — no torn state."""
    import threading

    _populate(sqlite_store, "lidar", [0.0, 0.1, 0.2])
    _populate(sqlite_store, "odom", [0.0, 0.1, 0.2])
    replay = sqlite_store.replay()

    barrier = threading.Barrier(2)
    anchors: list[tuple[float, float] | None] = [None, None]

    def subscribe_and_capture(slot: int, stream_name: str) -> None:
        barrier.wait()
        sub = replay.stream(stream_name).observable().subscribe(on_next=lambda _: None)
        time.sleep(0.05)
        sub.dispose()
        anchors[slot] = replay._anchor

    t1 = threading.Thread(target=subscribe_and_capture, args=(0, "lidar"))
    t2 = threading.Thread(target=subscribe_and_capture, args=(1, "odom"))
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    assert anchors[0] is not None
    assert anchors[0] == anchors[1]
