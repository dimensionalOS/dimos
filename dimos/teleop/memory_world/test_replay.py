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
from pathlib import Path

import numpy as np
import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.teleop.memory_world.replay import (
    TAG_ADDED,
    TAG_REMOVED,
    ReplayRecorder,
    final_map,
    timeline_end,
    timeline_matches,
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


def test_final_map_is_the_last_snapshot(store: SqliteStore) -> None:
    maps = _maps(4, 12)
    _record(store, maps, interval=0.5)

    assert timeline_matches(store, voxel_size=VOXEL, keyframe_interval_s=0.5)
    assert not timeline_matches(store, voxel_size=VOXEL * 2, keyframe_interval_s=0.5)
    assert not timeline_matches(store, voxel_size=VOXEL, keyframe_interval_s=5.0)
    assert timeline_end(store) == pytest.approx(101.1)
    assert store.streams["voxel_keyframe"].count() == 3  # t=100.0, 100.5, 101.0

    centers, map_ts = final_map(store, VOXEL)

    assert map_ts == pytest.approx(101.1)
    assert _voxels(centers) == _voxels(maps[-1])
