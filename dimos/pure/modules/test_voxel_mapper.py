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

"""VoxelMapper(2): fold + Mealy units, over() synthetics, go2_hongkong_office parity."""

from __future__ import annotations

import itertools
from typing import TYPE_CHECKING, Any

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.pure.modules import voxel_mapper
from dimos.pure.modules.voxel_mapper import VoxelMapper, VoxelMapper2

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


# ── tf: sensor-frame input transformed into the map frame ────────────────────


def _shift_tf(dx: float, *ts: float) -> list[Any]:
    """world <- sensor edge = a pure +dx translation, sampled at each ts."""
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

    out = []
    for t in ts:
        tf = Transform(
            translation=Vector3(dx, 0.0, 0.0), frame_id="world", child_frame_id="sensor", ts=t
        )
        tf.ts = t  # ctor swaps an explicit ts=0.0 for wall clock; force the sample ts
        out.append(tf)
    return out


def test_over_tf_maps_sensor_into_world_frame() -> None:
    # scans in "sensor"; a +10 m x tf carries them to "world" — identical to
    # mapping the pre-shifted points with no tf wired.
    dx = 10.0
    sensor = [_cloud([[0.1, 0.2, 0.3], [2.1, 0.0, 0.0]], ts=1.0)]
    tf = _shift_tf(dx, 0.0, 2.0)  # brackets the tick at ts=1.0
    m = VoxelMapper(voxel_size=VOXEL, emit_every=0, sensor_frame="sensor")
    (tf_out,) = list(m.over(scan=sensor, tf=tf))

    world = [_cloud([[0.1 + dx, 0.2, 0.3], [2.1 + dx, 0.0, 0.0]], ts=1.0)]
    (world_out,) = list(VoxelMapper(voxel_size=VOXEL, emit_every=0).over(scan=world))

    np.testing.assert_array_equal(_centers(tf_out), _centers(world_out))
    assert tf_out.global_map.frame_id == "world"


def test_over_no_tf_is_passthrough() -> None:
    # the historical shape: unwired tf → pose=None → scans insert as-is
    m = VoxelMapper(voxel_size=VOXEL, emit_every=0, sensor_frame="sensor")
    (out,) = list(m.over(scan=[_cloud([[0.1, 0.1, 0.1]], ts=1.0)]))
    np.testing.assert_allclose(out.global_map.points_f32(), [[0.25, 0.25, 0.25]], atol=1e-6)


def test_mapper2_over_tf_maps_sensor_into_world_frame() -> None:
    dx = 10.0
    sensor = [_cloud([[0.1, 0.2, 0.3]], ts=1.0)]
    m = VoxelMapper2(voxel_size=VOXEL, emit_every=1, sensor_frame="sensor")
    (out,) = list(m.over(scan=sensor, tf=_shift_tf(dx, 0.0, 2.0)))
    np.testing.assert_allclose(out.global_map.points_f32(), [[10.25, 0.25, 0.25]], atol=1e-6)


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


# ══════════════════════════════════════════════════════════════════════════════
# VoxelMapper2 — the Mealy + @resource respelling (same core, compared side by side)
# ══════════════════════════════════════════════════════════════════════════════


def _rows2(*clouds: PointCloud2) -> Iterator[VoxelMapper2.In]:
    return iter(VoxelMapper2.In(ts=c.ts, scan=c) for c in clouds)


def _centers(out: Any) -> np.ndarray:
    """Voxel-center set of an Out row, sorted lexicographically for comparison."""
    pts = out.global_map.points_f32()
    return pts[np.lexsort(pts.T[::-1])]


# ── unit: step behavior, lazy resource, no engine ────────────────────────────


def test_mapper2_step_lazy_resource_and_count() -> None:
    # lazy-resource test-mode: m.grid is touchable without a run (T7 §5.1)
    m = VoxelMapper2(voxel_size=VOXEL, emit_every=2, carve_columns=False)
    s = VoxelMapper2.State()
    s, out = m.step(s, next(_rows2(_cloud([[0.1, 0.1, 0.1], [0.7, 0.1, 0.1]], ts=1.0))))
    assert s.n_scans == 1 and out is None  # below cadence → skip
    s, out = m.step(s, next(_rows2(_cloud([[5.0, 0.0, 0.0]], ts=2.0))))
    assert s.n_scans == 2 and out is not None  # cadence hit → emit
    assert out.n_scans == 2 and out.n_voxels == 3
    centers = np.sort(out.global_map.points_f32(), axis=0)
    np.testing.assert_allclose(
        centers, [[0.25, 0.25, 0.25], [0.75, 0.25, 0.25], [5.25, 0.25, 0.25]], atol=1e-6
    )


def test_mapper2_step_below_cadence_returns_none() -> None:
    m = VoxelMapper2(voxel_size=VOXEL, emit_every=50)
    s = VoxelMapper2.State()
    for c in _grid_rows(49):
        s, out = m.step(s, VoxelMapper2.In(ts=c.ts, scan=c))
        assert out is None
    assert s.n_scans == 49


# ── engine: synthetic streams through over() ─────────────────────────────────


def test_mapper2_over_cadence_no_tail_flush() -> None:
    # 5 scans, emit_every=2 → emits at 2 and 4 ONLY; the 5th (tail partial) is lost
    m = VoxelMapper2(voxel_size=VOXEL, emit_every=2)
    rows = list(m.over(scan=_grid_rows(5)))
    assert [r.n_scans for r in rows] == [2, 4]  # NOT [2, 4, 5] — no exhaustion flush
    assert [r.ts for r in rows] == [2.0, 4.0]  # engine-stamped from the tick row
    assert [r.n_voxels for r in rows] == [2, 4]
    assert rows[-1].global_map.frame_id == "world"


def test_mapper2_over_emits_nothing_below_first_cadence() -> None:
    m = VoxelMapper2(voxel_size=VOXEL, emit_every=50)
    assert list(m.over(scan=_grid_rows(49))) == []  # never reaches a cadence point


def test_mapper2_over_fresh_grid_per_run() -> None:
    # engine builds a fresh resource per run — two runs do not share accumulation
    m = VoxelMapper2(voxel_size=VOXEL, emit_every=3)
    a = list(m.over(scan=_grid_rows(3)))
    b = list(m.over(scan=_grid_rows(3)))
    assert a[-1].n_voxels == b[-1].n_voxels == 3  # not 3 then 6


def test_mapper2_over_disposes_on_exhaustion(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(voxel_mapper, "VoxelGrid", _SpyGrid)
    list(VoxelMapper2(emit_every=1).over(scan=_grid_rows(3)))  # run to exhaustion
    assert _SpyGrid.last is not None and _SpyGrid.last.disposed  # engine teardown disposed


def test_mapper2_over_disposes_on_early_break(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(voxel_mapper, "VoxelGrid", _SpyGrid)
    it = VoxelMapper2(emit_every=1).over(scan=_grid_rows(3))
    next(it)
    it.close()  # consumer breaks early — resource teardown must still dispose
    assert _SpyGrid.last is not None and _SpyGrid.last.disposed


# ── the comparison: fold and Mealy agree at shared cadence points ────────────


def test_mapper2_matches_fold_at_shared_cadence() -> None:
    """Both mappers over the SAME stream → identical maps at 2 & 4; fold alone emits 5."""
    clouds = _grid_rows(5)
    fold = list(VoxelMapper(voxel_size=VOXEL, emit_every=2).over(scan=clouds))
    mealy = list(VoxelMapper2(voxel_size=VOXEL, emit_every=2).over(scan=clouds))
    # cadence points 2 & 4 are shared; fold adds a tail 5, Mealy cannot
    assert [r.n_scans for r in fold] == [2, 4, 5]
    assert [r.n_scans for r in mealy] == [2, 4]
    for f, m in zip(fold, mealy, strict=False):  # the common prefix must be voxel-identical
        assert f.n_scans == m.n_scans and f.n_voxels == m.n_voxels
        assert f.ts == m.ts
        np.testing.assert_array_equal(_centers(f), _centers(m))


# ── real data: VoxelMapper2 over the go2_hongkong_office slice (skip-guarded) ──


@pytest.fixture(scope="module")
def hk_rows2() -> list[VoxelMapper2.Out]:
    """First N_SCANS scans through VoxelMapper2.over(); one shared run per module."""
    from dimos.memory2.store.sqlite import SqliteStore

    path = _get_data("go2_hongkong_office.db")
    with SqliteStore(path=path) as store:
        lidar = store.stream("lidar", PointCloud2)
        m = VoxelMapper2(voxel_size=REAL_VOXEL, emit_every=50)
        return list(m.over(scan=lidar.range_seek(0, N_SCANS)))


def test_mapper2_real_slice_no_tail_flush(hk_rows2: list[VoxelMapper2.Out]) -> None:
    # fold emits 50..250 plus the 298 tail; Mealy emits 50..250 and stops
    assert [r.n_scans for r in hk_rows2] == [50, 100, 150, 200, 250]
    ts = [r.ts for r in hk_rows2]
    assert ts == sorted(ts) and len(set(ts)) == len(ts)
    assert hk_rows2[-1].n_voxels > 10_000


def test_mapper2_matches_fold_on_real_slice(
    hk_rows: list[VoxelMapper.Out], hk_rows2: list[VoxelMapper2.Out]
) -> None:
    """Shared VoxelGrid core → the 250-scan snapshots are voxel-identical across shapes."""
    fold_250 = next(r for r in hk_rows if r.n_scans == 250)
    mealy_250 = next(r for r in hk_rows2 if r.n_scans == 250)
    assert fold_250.n_voxels == mealy_250.n_voxels
    # symmetric nearest-neighbour distance is 0 iff the voxel-center sets coincide
    fwd = np.asarray(
        mealy_250.global_map.pointcloud.compute_point_cloud_distance(fold_250.global_map.pointcloud)
    )
    rev = np.asarray(
        fold_250.global_map.pointcloud.compute_point_cloud_distance(mealy_250.global_map.pointcloud)
    )
    assert float(fwd.max()) == 0.0 and float(rev.max()) == 0.0  # identical maps
