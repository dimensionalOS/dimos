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

import json
import struct

import numpy as np
import pytest

from dimos.mapping.voxels.impl.packed import PackedVoxels
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.teleop.memory_world.replay import (
    TAG_ADDED,
    TAG_REMOVED,
    ReplayGrid,
    VoxelReplay,
    build_replay_streams,
)

VOXEL = 0.1


def _scans(seed: int, count: int, points: int = 300) -> list[np.ndarray]:
    """Random walls that drift a little, so scans overlap yet keep changing."""
    rng = np.random.default_rng(seed)
    scans = []
    for i in range(count):
        base = rng.uniform(-2, 2, size=(points, 3)).astype(np.float32)
        base[:, 2] = np.abs(base[:, 2])  # above the floor
        base[:, 0] += 0.02 * i
        scans.append(base)
    return scans


def test_replay_grid_matches_packed_voxels_when_carving_immediately() -> None:
    grid = ReplayGrid(VOXEL, remove_after=1)
    packed = PackedVoxels(voxel_size=VOXEL, carve_columns=True)
    for scan in _scans(1, 12):
        grid.add_scan(scan)
        packed.add_frame(PointCloud2.from_numpy(scan, frame_id="world"))
    np.testing.assert_array_equal(grid.keys, packed._keys)
    np.testing.assert_allclose(grid.centres(), packed.points())


def test_diffs_replay_to_the_same_set() -> None:
    """Applying every (added, removed) pair from empty reproduces the grid."""
    grid = ReplayGrid(VOXEL)
    state: set[int] = set()
    for scan in _scans(2, 10):
        added, removed = grid.add_scan(scan)
        assert not (set(added.tolist()) & set(removed.tolist()))
        state -= set(removed.tolist())
        state |= set(added.tolist())
        assert state == set(grid.keys.tolist())


def test_hysteresis_keeps_missed_voxels_for_a_while() -> None:
    scans = _scans(3, 4)
    quick = ReplayGrid(VOXEL, remove_after=1)
    patient = ReplayGrid(VOXEL, remove_after=3)
    removed_quick = removed_patient = 0
    for scan in scans:
        removed_quick += len(quick.add_scan(scan)[1])
        removed_patient += len(patient.add_scan(scan)[1])
    assert removed_patient < removed_quick
    assert len(patient.keys) >= len(quick.keys)


@pytest.fixture
def store(tmp_path):  # type: ignore[no-untyped-def]
    store = SqliteStore(path=str(tmp_path / "replay.db"))
    lidar = store.stream("lidar", PointCloud2)
    for i, scan in enumerate(_scans(4, 30)):
        ts = 100.0 + i * 0.1
        lidar.append(PointCloud2.from_numpy(scan, frame_id="world", timestamp=ts), ts=ts)
    yield store
    store.stop()


def test_build_streams_and_serve_segments(store) -> None:  # type: ignore[no-untyped-def]
    stats = build_replay_streams(
        store,
        lidar_stream_name="lidar",
        to_world=lambda obs: obs.data,
        voxel_size=VOXEL,
        keyframe_interval_s=1.0,
    )
    assert stats.scans == 30
    assert stats.keyframes == 3  # t=100.0, 101.0, 102.0
    assert VoxelReplay.available(store, voxel_size=VOXEL, lidar_stream_name="lidar")
    assert not VoxelReplay.available(store, voxel_size=VOXEL * 2, lidar_stream_name="lidar")

    diffs = list(store.streams["voxel_diff"])
    assert len(diffs) == 30
    tags = diffs[5].data.tags_u8()
    assert set(tags.tolist()) <= {TAG_ADDED, TAG_REMOVED}

    replay = VoxelReplay(store)
    index = replay.index
    assert index.scan_at(100.55) == 5
    assert index.segment_of(5) == 0 and index.segment_of(10) == 1 and index.segment_of(29) == 2
    assert index.segment_scans(1) == (10, 20)

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

    def voxels(points: np.ndarray) -> set[tuple[int, ...]]:
        return {tuple(int(v) for v in row) for row in np.floor(points / VOXEL).astype(int)}

    # the state after the last scan of segment 1 (scan 19) is one scan before keyframe 2 (scan 20)
    keyframe2 = store.streams["voxel_keyframe"].at(102.0, tolerance=1e-3).first().data.points_f32()
    scan20 = diffs[20].data
    tags20 = scan20.tags_u8()
    points20 = scan20.points_f32()
    expected = voxels(keyframe2) - voxels(points20[tags20 == TAG_ADDED]) | voxels(
        points20[tags20 == TAG_REMOVED]
    )
    assert shown == expected
