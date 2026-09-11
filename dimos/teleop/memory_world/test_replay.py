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

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

pytest.importorskip("dimos_voxel_ray_tracing")

from dimos.teleop.memory_world.replay import (
    TAG_ADDED,
    TAG_REMOVED,
    RayTracedGrid,
    SensorScan,
    VoxelReplay,
    build_replay_streams,
    pack_keys,
    sensor_scan,
    unpack_centres,
)
from dimos.teleop.memory_world.tf_tree import pose_matrix

VOXEL = 0.1
AT_ORIGIN = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


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


def _wall(x: float) -> np.ndarray:
    """A wall at x facing the origin, every voxel of it hit: (N, 3) float32.

    Walls at different x subtend the same angles from the origin, so rays to
    a farther wall pass through every voxel of a nearer one.
    """
    half = x / 3.0
    side = np.arange(-half, half, VOXEL / 2, dtype=np.float32) + VOXEL / 4
    y, z = np.meshgrid(side, side + half)
    return np.stack([np.full(y.size, x, np.float32), y.ravel(), z.ravel()], axis=1)


def test_keys_round_trip_through_centres() -> None:
    points = np.array([[0.31, -0.29, 1.04], [0.31, -0.29, 1.09], [5.0, 5.0, 5.0]], np.float32)
    keys = pack_keys(points, VOXEL)
    assert len(keys) == 2  # the first two share a voxel
    np.testing.assert_allclose(
        unpack_centres(keys, VOXEL), [[0.35, -0.25, 1.05], [5.05, 5.05, 5.05]]
    )


def test_sensor_scan_moves_a_world_scan_back_to_the_sensor() -> None:
    world_from_sensor = pose_matrix((10.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    scan = sensor_scan(np.array([[12.0, 1.0, 0.5]], np.float32), world_from_sensor, in_world=True)
    np.testing.assert_allclose(scan.points, [[2.0, 1.0, 0.5]])
    assert scan.position == (10.0, 0.0, 0.0)
    kept = sensor_scan(np.array([[2.0, 1.0, 0.5]], np.float32), world_from_sensor, in_world=False)
    np.testing.assert_allclose(kept.points, [[2.0, 1.0, 0.5]])


def test_diffs_replay_to_the_same_set() -> None:
    """Applying every (added, removed) pair from empty reproduces the grid."""
    grid = RayTracedGrid(VOXEL, max_range=10.0)
    state: set[int] = set()
    for scan in _scans(2, 10):
        added, removed = grid.add_scan(SensorScan(scan, *AT_ORIGIN))
        assert not (set(added.tolist()) & set(removed.tolist()))
        state -= set(removed.tolist())
        state |= set(added.tolist())
        assert state == set(grid.keys.tolist())
    assert state


def test_a_wall_seen_through_is_cleared() -> None:
    """Column carving would keep a wall that later scans see straight past;
    ray tracing removes it, and never touches voxels beyond the ray range."""
    grid = RayTracedGrid(VOXEL, max_range=10.0)
    near = _wall(3.0)
    for _ in range(8):
        grid.add_scan(SensorScan(near, *AT_ORIGIN))
    near_keys = set(pack_keys(near, VOXEL).tolist())
    held = set(grid.keys.tolist())
    assert len(near_keys & held) >= 0.9 * len(near_keys)  # the head-on wall is in the map

    far = _wall(6.0)
    removed_total: set[int] = set()
    for _ in range(12):
        _, removed = grid.add_scan(SensorScan(far, *AT_ORIGIN))
        removed_total |= set(removed.tolist())
    assert near_keys & removed_total  # the old wall came out
    assert not (near_keys & set(grid.keys.tolist()))


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
        to_scan=lambda obs: SensorScan(obs.data.points_f32(), *AT_ORIGIN),
        voxel_size=VOXEL,
        max_range=10.0,
        keyframe_interval_s=1.0,
    )
    assert stats.scans == 30
    assert stats.keyframes == 4  # t=100.0, 101.0, 102.0 and the last scan
    assert VoxelReplay.available(store, voxel_size=VOXEL, lidar_stream_name="lidar")
    assert not VoxelReplay.available(store, voxel_size=VOXEL * 2, lidar_stream_name="lidar")

    diffs = list(store.streams["voxel_diff"])
    assert len(diffs) == 30
    tags = diffs[5].data.tags_u8()
    assert set(tags.tolist()) <= {TAG_ADDED, TAG_REMOVED}

    replay = VoxelReplay(store)
    index = replay.index
    assert index.scan_at(100.55) == 5
    assert index.segment_of(5) == 0 and index.segment_of(10) == 1 and index.segment_of(29) == 3
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
