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

"""CostMapper pure module: stateless step, merged-preference alignment, legacy parity."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.mapping.costmapper import CostMapper as LegacyCostMapper
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.pure.modules.costmapper import PureCostMapper as CostMapper


def _cloud(shift: float, ts: float) -> PointCloud2:
    """A 20x20 flat ground patch at z=0, shifted +shift in x, stamped ts."""
    pts = np.array(
        [[x * 0.05 + shift, y * 0.05, 0.0] for x in range(-10, 10) for y in range(-10, 10)],
        dtype=np.float32,
    )
    return PointCloud2.from_numpy(pts, timestamp=ts)


def _legacy_grid(cloud: PointCloud2, **cfg: object) -> object:
    """Legacy CostMapper._calculate_costmap on one cloud, no engine/streams."""
    m = LegacyCostMapper(**cfg)  # type: ignore[arg-type]  # flat config kwargs
    try:
        return m._calculate_costmap(cloud)
    finally:
        m.dispose()  # the ctor spins up transport threads — don't leak them


# ── config ───────────────────────────────────────────────────────────────────


def test_config_fields() -> None:
    m = CostMapper(initial_safe_radius_meters=0.5)
    assert m.algo == "height_cost"
    assert m.initial_safe_radius_meters == 0.5
    # T9: health_hz is a PureModuleConfig base field, so it leads the dump order.
    assert list(m.config.model_dump()) == [
        "health_hz",
        "algo",
        "occupancy",
        "initial_safe_radius_meters",
    ]


# ── stateless step, no engine ────────────────────────────────────────────────


def test_step_uses_global_when_no_merged() -> None:
    g = _cloud(0.0, 1.0)
    out = CostMapper().step(CostMapper.In(ts=1.0, global_map=g, merged_map=None))
    assert np.array_equal(out.global_costmap.grid, _legacy_grid(g).grid)


def test_step_prefers_merged_when_present() -> None:
    g, mrg = _cloud(0.0, 1.0), _cloud(5.0, 1.0)
    out = CostMapper().step(CostMapper.In(ts=1.0, global_map=g, merged_map=mrg))
    # merged is the +5m-shifted cloud → the grid tracks it, not global
    assert np.array_equal(out.global_costmap.grid, _legacy_grid(mrg).grid)
    assert out.global_costmap.origin.position.x == _legacy_grid(mrg).origin.position.x


def test_grid_ts_is_source_cloud_ts() -> None:
    g = _cloud(0.0, 7.0)
    out = CostMapper().step(CostMapper.In(ts=7.0, global_map=g, merged_map=None))
    assert out.global_costmap.ts == 7.0  # replay-deterministic, no wall clock


def test_initial_safe_radius_zeros_origin_disc() -> None:
    g = _cloud(0.0, 1.0)
    bare = CostMapper().step(CostMapper.In(ts=1.0, global_map=g, merged_map=None)).global_costmap
    safe = (
        CostMapper(initial_safe_radius_meters=0.3)
        .step(CostMapper.In(ts=1.0, global_map=g, merged_map=None))
        .global_costmap
    )
    # the safe disc only clears cells (sets them to 0); it never adds cost
    assert (safe.grid == 0).sum() >= (bare.grid == 0).sum()
    assert np.array_equal(safe.grid, _legacy_grid(g, initial_safe_radius_meters=0.3).grid)


# ── engine: over() alignment ─────────────────────────────────────────────────


def test_over_ticks_on_global_and_stamps_ts() -> None:
    rows = list(CostMapper().over(global_map=[_cloud(0.0, 1.0), _cloud(0.0, 2.0)]))
    assert [r.ts for r in rows] == [1.0, 2.0]  # engine tick ts = payload ts
    assert all(r.global_costmap.ts == r.ts for r in rows)


def test_over_merged_preferred_at_tick() -> None:
    g = _cloud(0.0, 2.0)
    mrg = _cloud(5.0, 1.0)  # arrives before the tick → available as latest()
    (row,) = list(CostMapper().over(global_map=[g], merged_map=[mrg]))
    assert row.global_costmap.origin.position.x == _legacy_grid(mrg).origin.position.x


def test_over_merged_after_tick_falls_back_to_global() -> None:
    g = _cloud(0.0, 1.0)
    mrg = _cloud(5.0, 2.0)  # after the tick → no sample at-or-before tick 1.0
    (row,) = list(CostMapper().over(global_map=[g], merged_map=[mrg]))
    assert row.global_costmap.origin.position.x == _legacy_grid(g).origin.position.x


def test_over_empty_input_emits_nothing() -> None:
    assert list(CostMapper().over(global_map=[])) == []


# ── legacy parity ────────────────────────────────────────────────────────────


@pytest.mark.parametrize("radius", [0.0, 0.3, 1.0])
def test_parity_with_legacy_calculate(radius: float) -> None:
    g = _cloud(0.0, 3.0)
    pure = CostMapper(initial_safe_radius_meters=radius).step(
        CostMapper.In(ts=3.0, global_map=g, merged_map=None)
    )
    legacy = _legacy_grid(g, initial_safe_radius_meters=radius)
    assert np.array_equal(pure.global_costmap.grid, legacy.grid)
    assert pure.global_costmap.ts == legacy.ts == 3.0
    assert pure.global_costmap.origin.position.x == legacy.origin.position.x
