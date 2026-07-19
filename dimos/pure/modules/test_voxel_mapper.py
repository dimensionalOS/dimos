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

"""VoxelMapper: fold units (no engine), over() synthetics, go2_hongkong_office."""

from __future__ import annotations

import itertools
from typing import TYPE_CHECKING, Any

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.pure.modules import voxel_mapper
from dimos.pure.modules.voxel_mapper import VoxelMapper

if TYPE_CHECKING:
    from collections.abc import Iterator

VOXEL = 0.5


def _cloud(pts: list[list[float]], ts: float) -> PointCloud2:
    return PointCloud2.from_numpy(np.array(pts, dtype=np.float32), timestamp=ts)


def _rows(*clouds: PointCloud2) -> Iterator[VoxelMapper.In]:
    return iter(VoxelMapper.In(ts=c.ts, scan=c) for c in clouds)


def _grid_rows(n: int) -> list[PointCloud2]:
    """n single-point clouds at distinct voxels, ts = 1.0, 2.0, ..."""
    return [_cloud([[i * 2.0, 0.0, 0.0]], ts=float(i + 1)) for i in range(n)]


# ── unit: fold behavior, no engine ───────────────────────────────────────────


def test_voxel_centers_math() -> None:
    # two points in one 0.5 m voxel + one in the next → two centers, on the grid
    m = VoxelMapper(voxel_size=VOXEL, emit_every=0, carve_columns=False)
    clouds = _cloud([[0.1, 0.1, 0.1], [0.4, 0.2, 0.3], [0.7, 0.1, 0.1]], ts=1.0)
    (out,) = list(m.fold(_rows(clouds)))
    assert out.n_voxels == 2
    centers = np.sort(out.global_map.points_f32(), axis=0)
    np.testing.assert_allclose(centers, [[0.25, 0.25, 0.25], [0.75, 0.25, 0.25]], atol=1e-6)


def test_emit_cadence_and_final_partial() -> None:
    m = VoxelMapper(voxel_size=VOXEL, emit_every=2)
    outs = list(m.fold(_rows(*_grid_rows(5))))
    assert [o.n_scans for o in outs] == [2, 4, 5]
    assert [o.ts for o in outs] == [2.0, 4.0, 5.0]  # stamped from the folded row
    assert [o.n_voxels for o in outs] == [2, 4, 5]


def test_no_duplicate_final_emission() -> None:
    m = VoxelMapper(voxel_size=VOXEL, emit_every=2)
    outs = list(m.fold(_rows(*_grid_rows(4))))
    assert [o.n_scans for o in outs] == [2, 4]


def test_batch_mode_emits_once_at_exhaustion() -> None:
    m = VoxelMapper(voxel_size=VOXEL, emit_every=0)
    outs = list(m.fold(_rows(*_grid_rows(5))))
    assert [(o.ts, o.n_scans) for o in outs] == [(5.0, 5)]


def test_empty_input_emits_nothing() -> None:
    assert list(VoxelMapper().fold(iter([]))) == []


def test_fold_rerun_is_fresh() -> None:
    # module instance carries config only — each run gets a fresh grid
    m = VoxelMapper(voxel_size=VOXEL, emit_every=0)
    a = list(m.fold(_rows(*_grid_rows(3))))
    b = list(m.fold(_rows(*_grid_rows(3))))
    assert a[-1].n_voxels == b[-1].n_voxels == 3


def test_carve_columns_config() -> None:
    # same (x, y) column, different z: carving keeps only the latest observation
    top = _cloud([[0.1, 0.1, 5.0]], ts=2.0)
    bottom = _cloud([[0.1, 0.1, 0.1]], ts=1.0)
    carved = list(VoxelMapper(voxel_size=VOXEL, emit_every=0).fold(_rows(bottom, top)))
    kept = list(
        VoxelMapper(voxel_size=VOXEL, emit_every=0, carve_columns=False).fold(_rows(bottom, top))
    )
    assert carved[-1].n_voxels == 1
    assert kept[-1].n_voxels == 2


class _SpyGrid:
    """Stand-in for VoxelGrid recording dispose calls."""

    last: _SpyGrid | None = None

    def __init__(self, **kwargs: Any) -> None:
        self.frames = 0
        self.disposed = False
        _SpyGrid.last = self

    def add_frame(self, frame: PointCloud2) -> None:
        self.frames += 1

    def get_global_pointcloud2(self) -> PointCloud2:
        return _cloud([[0.0, 0.0, 0.0]], ts=1.0)

    def __len__(self) -> int:
        return self.frames

    def dispose(self) -> None:
        self.disposed = True


def test_fold_early_close_disposes(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(voxel_mapper, "VoxelGrid", _SpyGrid)
    gen = VoxelMapper(emit_every=1).fold(_rows(*_grid_rows(3)))
    next(gen)
    gen.close()
    assert _SpyGrid.last is not None and _SpyGrid.last.disposed


# ── engine: synthetic streams through over() ─────────────────────────────────


def test_over_synthetic_sparse_emission() -> None:
    m = VoxelMapper(voxel_size=VOXEL, emit_every=2)
    rows = list(m.over(scan=_grid_rows(4)))
    assert [r.ts for r in rows] == [2.0, 4.0]  # engine tick ts = payload ts
    assert [r.n_scans for r in rows] == [2, 4]
    assert rows[-1].n_voxels == 4
    assert rows[-1].global_map.frame_id == "world"


def test_over_early_exit_disposes(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(voxel_mapper, "VoxelGrid", _SpyGrid)
    it = VoxelMapper(emit_every=1).over(scan=_grid_rows(3))
    next(it)
    it.close()  # consumer breaks out early — teardown must still dispose
    assert _SpyGrid.last is not None and _SpyGrid.last.disposed


# ── real data: go2_hongkong_office (skip-guarded on missing dataset) ─────────

N_SCANS = 300
REAL_VOXEL = 0.1
EXPECTED_TICKS = 298  # 300 stored scans - 2 payload-ts regressions dropped by align


def _get_data(name: str) -> Any:
    from dimos.utils.data import get_data

    try:
        return get_data(name)
    except Exception as exc:  # LFS unavailable, offline, ... — skip, don't fail
        pytest.skip(f"dataset {name} unavailable: {exc}")


@pytest.fixture(scope="module")
def hk_rows() -> list[VoxelMapper.Out]:
    """First N_SCANS lidar scans through over(); one shared run per module."""
    from dimos.memory2.store.sqlite import SqliteStore

    path = _get_data("go2_hongkong_office.db")
    with SqliteStore(path=path) as store:
        lidar = store.stream("lidar", PointCloud2)
        m = VoxelMapper(voxel_size=REAL_VOXEL, emit_every=50)
        return list(m.over(scan=lidar.range_seek(0, N_SCANS)))


@pytest.fixture(scope="module")
def reference_map() -> PointCloud2:
    path = _get_data("go2_hongkong_office_twopass_map.pc2.lcm")
    return PointCloud2.lcm_decode(path.read_bytes())


def test_real_slice_structure(hk_rows: list[VoxelMapper.Out]) -> None:
    assert [r.n_scans for r in hk_rows] == [50, 100, 150, 200, 250, EXPECTED_TICKS]
    ts = [r.ts for r in hk_rows]
    assert ts == sorted(ts) and len(set(ts)) == len(ts)  # strictly monotonic
    counts = [r.n_voxels for r in hk_rows]
    assert counts[-1] > 10_000  # non-trivial office geometry
    assert all(a <= b for a, b in itertools.pairwise(counts))  # map only grows here
    dims = hk_rows[-1].global_map.bounding_box_dimensions
    assert all(2.0 < d < 40.0 for d in dims[:2])  # plausible office extent
    assert dims[2] < 5.0


def test_real_slice_matches_reference(
    hk_rows: list[VoxelMapper.Out], reference_map: PointCloud2
) -> None:
    final = hk_rows[-1].global_map
    dists = np.asarray(final.pointcloud.compute_point_cloud_distance(reference_map.pointcloud))
    # loose agreement vs the PGO twopass map; measured 0.89 / 0.97
    assert (dists <= REAL_VOXEL).mean() >= 0.6
    assert (dists <= 2 * REAL_VOXEL).mean() >= 0.8


@pytest.mark.self_hosted
def test_full_dataset_matches_reference(reference_map: PointCloud2) -> None:
    """Entire 558 s session (~30 s CPU); deselected by default addopts."""
    from dimos.memory2.store.sqlite import SqliteStore

    path = _get_data("go2_hongkong_office.db")
    with SqliteStore(path=path) as store:
        lidar = store.stream("lidar", PointCloud2)
        m = VoxelMapper(voxel_size=REAL_VOXEL, emit_every=1000)
        rows = list(m.over(scan=lidar))
    assert rows[-1].n_scans > 4000
    dists = np.asarray(
        rows[-1].global_map.pointcloud.compute_point_cloud_distance(reference_map.pointcloud)
    )
    assert (dists <= 2 * REAL_VOXEL).mean() >= 0.5  # PGO corrections drift late in the run
