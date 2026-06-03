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

import time
from typing import TYPE_CHECKING

import numpy as np
import pytest

from dimos.mapping.voxels import VoxelMapTransformer
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.type.observation import Observation
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data

if TYPE_CHECKING:
    from collections.abc import Iterator


@pytest.fixture(scope="module")
def store() -> Iterator[SqliteStore]:
    db = SqliteStore(path=get_data("go2_bigoffice.db"))
    with db:
        yield db


def _make_obs(obs_id: int, points: np.ndarray, ts: float = 0.0) -> Observation[PointCloud2]:
    return Observation(id=obs_id, ts=ts, _data=PointCloud2.from_numpy(points))


def _unit_cube_points(n: int = 100) -> np.ndarray:
    rng = np.random.default_rng(42)
    return rng.random((n, 3)).astype(np.float32)


def test_accumulate_two_frames() -> None:
    """Two non-overlapping frames produce a larger global map."""
    pts = _unit_cube_points(50)
    obs1 = _make_obs(0, pts, ts=1.0)
    obs2 = _make_obs(1, pts + 10.0, ts=2.0)  # offset by 10m, no overlap

    xf = VoxelMapTransformer(voxel_size=0.5, carve_columns=False)
    results = list(xf(iter([obs1, obs2])))

    assert len(results) == 2  # emit_every=1 default
    global_map = results[-1].data  # last result has the full accumulated map

    single_results = list(VoxelMapTransformer(voxel_size=0.5)(iter([obs1])))
    assert len(global_map) > len(single_results[0].data)


def test_empty_stream() -> None:
    xf = VoxelMapTransformer(voxel_size=0.5)
    assert list(xf(iter([]))) == []


def test_frame_count_tag() -> None:
    pts = _unit_cube_points(30)
    obs = [_make_obs(i, pts, ts=float(i)) for i in range(5)]

    xf = VoxelMapTransformer(voxel_size=0.5, device="CPU:0")
    results = list(xf(iter(obs)))

    assert len(results) == 5  # emit_every=1 (default), one result per frame
    assert results[-1].tags["frame_count"] == 5


def test_emit_every_batch_mode() -> None:
    """emit_every=0 yields only on exhaustion (batch mode)."""
    pts = _unit_cube_points(30)
    obs = [_make_obs(i, pts, ts=float(i)) for i in range(5)]

    xf = VoxelMapTransformer(voxel_size=0.5, device="CPU:0", emit_every=0)
    results = list(xf(iter(obs)))

    assert len(results) == 1
    assert results[0].tags["frame_count"] == 5


def test_emit_every_n() -> None:
    """emit_every=3 yields after every 3rd frame, plus remainder on exhaustion."""
    pts = _unit_cube_points(30)
    obs = [_make_obs(i, pts, ts=float(i)) for i in range(7)]

    xf = VoxelMapTransformer(voxel_size=0.5, device="CPU:0", emit_every=3)
    results = list(xf(iter(obs)))

    # 7 frames / emit_every=3 → yields at frame 3, 6, then remainder (7) on exhaustion
    assert len(results) == 3
    assert results[0].tags["frame_count"] == 3
    assert results[1].tags["frame_count"] == 6
    assert results[2].tags["frame_count"] == 7


# -- Integration tests against real replay data --


@pytest.mark.tool
def test_build_global_map(store: SqliteStore) -> None:
    t_total = time.perf_counter()

    lidar = store.stream("lidar", PointCloud2)
    n_frames = lidar.count()

    t0 = time.perf_counter()
    result = lidar.transform(VoxelMapTransformer(voxel_size=0.05)).last()
    t_transform = time.perf_counter() - t0

    t_total = time.perf_counter() - t_total

    global_map = result.data
    frame_count = result.tags["frame_count"]

    assert frame_count == n_frames
    assert len(global_map) > 0

    print(
        lidar.summary(),
        f"\n{frame_count} frames -> {len(global_map)} voxels"
        f"\n  transform: {t_transform:.2f}s ({t_transform / frame_count * 1000:.1f}ms/frame)"
        f"\n  total wall: {t_total:.2f}s",
    )
