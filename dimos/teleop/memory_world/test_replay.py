# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Iterator
import json
from pathlib import Path
import struct

import numpy as np
import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.teleop.memory_world.replay import (
    TAG_ADDED,
    TAG_REMOVED,
    ReplayRecorder,
    VoxelReplay,
)

VOXEL = 0.1


def _maps(seed: int, count: int, points: int = 300) -> list[np.ndarray]:
    """Random walls that drift a little, so successive maps overlap yet keep changing."""
    rng = np.random.default_rng(seed)
    maps = []
    for i in range(count):
        base = rng.uniform(-2, 2, size=(points, 3)).astype(np.float32)
        base[:, 2] = np.abs(base[:, 2])
        base[:, 0] += 0.02 * i
        maps.append(base)
    return maps


def _voxels(points: np.ndarray) -> set[tuple[int, ...]]:
    return {tuple(int(v) for v in row) for row in np.floor(points / VOXEL).astype(int)}


@pytest.fixture
def store(tmp_path: Path) -> Iterator[SqliteStore]:
    store = SqliteStore(path=str(tmp_path / "replay.db"))
    yield store
    store.stop()


def _record(store: SqliteStore, maps: list[np.ndarray], interval: float) -> ReplayRecorder:
    recorder = ReplayRecorder(store, voxel_size=VOXEL, keyframe_interval_s=interval)
    for i, snapshot in enumerate(maps):
        recorder.add_snapshot(snapshot, 100.0 + i * 0.1)
    return recorder


def test_diffs_replay_to_each_snapshot(store: SqliteStore) -> None:
    maps = _maps(2, 10)
    _record(store, maps, interval=100.0)

    state: set[tuple[int, ...]] = set()
    for snapshot, diff in zip(maps, store.streams["voxel_diff"], strict=True):
        points = diff.data.points_f32()
        tags = diff.data.tags_u8()
        state -= _voxels(points[tags == TAG_REMOVED])
        state |= _voxels(points[tags == TAG_ADDED])
        assert state == _voxels(snapshot)


def test_record_and_serve_segments(store: SqliteStore) -> None:
    _record(store, _maps(4, 30), interval=1.0)

    assert VoxelReplay.matches(store, voxel_size=VOXEL, keyframe_interval_s=1.0)
    assert not VoxelReplay.matches(store, voxel_size=VOXEL * 2, keyframe_interval_s=1.0)
    assert not VoxelReplay.matches(store, voxel_size=VOXEL, keyframe_interval_s=5.0)
    diffs = list(store.streams["voxel_diff"])
    assert len(diffs) == 30
    assert store.streams["voxel_keyframe"].count() == 3  # t=100.0, 101.0, 102.0

    replay = VoxelReplay(store)
    index = replay.index
    assert index.stream_tags["built_at"] > 0
    assert index.scan_at(100.55) == 5
    assert index.segment_of(5) == 0 and index.segment_of(10) == 1 and index.segment_of(29) == 2
    assert index.segment_scans(1) == (10, 20)
    assert replay.covers(102.9) and not replay.covers(103.0)

    # replaying a segment's diffs on the viewer's terms reaches the next keyframe
    header, payload = replay.segment(1)
    raw = VoxelReplay.encode_segment(header, payload)
    header_length = struct.unpack("<I", raw[:4])[0]
    parsed = json.loads(raw[4 : 4 + header_length])
    assert parsed == header
    assert (4 + header_length) % 4 == 0
    body = raw[4 + header_length :]
    table = np.frombuffer(body[: header["slots"] * 6], dtype="<i2").reshape(-1, 3)
    entries = sum(scan["n"] for scan in header["scans"])
    slots = np.frombuffer(body[header["slots_offset"] :][: entries * 4], dtype="<u4")
    ops = np.frombuffer(body[header["slots_offset"] + entries * 4 :][:entries], dtype=np.uint8)

    visible = np.zeros(header["slots"], dtype=bool)
    visible[: header["keyframe"]["n"]] = True
    cursor = 0
    for scan in header["scans"]:
        if scan["index"] > header["keyframe"]["scan"]:
            for e in range(cursor, cursor + scan["n"]):
                visible[slots[e]] = ops[e] == TAG_ADDED
        cursor += scan["n"]
    shown = {tuple(int(v) for v in row) for row in table[visible] + np.asarray(index.origin)}

    # the state after the last scan of segment 1 (scan 19) is one scan before keyframe 2 (scan 20)
    keyframe2 = store.streams["voxel_keyframe"].at(102.0, tolerance=1e-3).first().data.points_f32()
    scan20 = diffs[20].data
    tags20 = scan20.tags_u8()
    points20 = scan20.points_f32()
    expected = _voxels(keyframe2) - _voxels(points20[tags20 == TAG_ADDED]) | _voxels(
        points20[tags20 == TAG_REMOVED]
    )
    assert shown == expected


def test_extend_grows_the_open_segment(store: SqliteStore) -> None:
    maps = _maps(5, 12)
    recorder = _record(store, maps[:4], interval=0.5)
    replay = VoxelReplay(store)
    assert len(replay.index.scan_ts) == 4
    stale_header, _ = replay.segment(0)

    for i, snapshot in enumerate(maps[4:], start=4):
        keyframe = recorder.add_snapshot(snapshot, 100.0 + i * 0.1)
        replay.extend(100.0 + i * 0.1, keyframe)

    assert len(replay.index.scan_ts) == 12
    assert replay.index.keyframe_scan.tolist() == [0, 5, 10]
    header, _ = replay.segment(0)
    assert len(stale_header["scans"]) == 4
    assert [scan["index"] for scan in header["scans"]] == [0, 1, 2, 3, 4]
    assert [scan["index"] for scan in replay.segment(2)[0]["scans"]] == [10, 11]
    assert replay.covers(101.1)
