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

"""The compact first look at a cloud that ``PointCloud2.agent_encode()`` returns."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

import numpy as np
from pydantic import JsonValue
from pydantic_core import to_jsonable_python

from dimos.experimental.agent_encode.pointcloud.grid.base import Grid
from dimos.experimental.agent_encode.pointcloud.grid.count import Count
from dimos.experimental.agent_encode.pointcloud.grid.lib.cells import cover
from dimos.experimental.agent_encode.pointcloud.grid.regions import Region
from dimos.experimental.agent_encode.pointcloud.grid.z_percentile import ZPercentile
from dimos.experimental.agent_encode.pointcloud.queries.base import Query
from dimos.experimental.agent_encode.pointcloud.queries.bounds import Bounds
from dimos.experimental.agent_encode.pointcloud.queries.lib.points import finite_points
from dimos.experimental.agent_encode.pointcloud.queries.select import Select
from dimos.experimental.agent_encode.pointcloud.queries.spacing import Spacing

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class OverviewGrid:
    """The cells a part of the overview was measured on."""

    origin: tuple[float, float]
    """World x, y of the south-west corner."""
    shape: tuple[int, int]
    """(columns, rows)."""
    cell_m: float
    limited: bool
    """The cell grew to keep the grid within the encoder's cell limit."""


@dataclass(frozen=True)
class Coverage:
    """Where the cloud has returns, on cells of two return spacings."""

    grid: OverviewGrid
    occupied_cells: int
    observed_xy_area_m2: float


@dataclass(frozen=True)
class Quantiles:
    """Linear 10th, 50th and 90th percentiles; None without values."""

    p10: float | None
    p50: float | None
    p90: float | None


@dataclass(frozen=True)
class LowerSurface:
    """The per-cell low percentile of z, on cells of four return spacings."""

    grid: OverviewGrid
    percentile: float
    min_returns_per_cell: int
    supported_cells: int
    """Cells with enough returns for a percentile."""
    observed_cells: int
    """Cells with any return."""
    supported_fraction: float
    z_quantiles_m: Quantiles
    """Of the supported cells' percentiles."""
    reference_z_m: float | None
    """Median of the supported cells' percentiles: a reference height, not a verified floor."""
    status: Literal["measured", "insufficient_support"]


@dataclass(frozen=True)
class StructureRegion:
    """Connected occupied cells in the structure band; they enclose returns and are not
    rooms or solid objects."""

    bounds: tuple[tuple[float, float], tuple[float, float]]
    """South-west and north-east corners of the cells."""
    area_m2: float


@dataclass(frozen=True)
class Structure:
    """Returns in the band ``z_range_m`` above the reference height, as regions."""

    z_range_m: tuple[float, float]
    region_count: int
    observed_area_m2: float
    """Of every region."""
    regions: tuple[StructureRegion, ...]
    """The largest regions."""
    omitted_regions: int
    omitted_area_m2: float


@dataclass(frozen=True)
class ReliefRegion:
    """Lower-surface cells further than the threshold from the reference height."""

    centroid: tuple[float, float]
    area_m2: float
    offset_quantiles_m: Quantiles
    """Of the cells' offsets from the reference height."""


@dataclass(frozen=True)
class Relief:
    """Rises and drops of the lower surface; no region does not prove level ground."""

    offset_threshold_m: float
    region_count: int
    regions: tuple[ReliefRegion, ...]
    """The largest rises and drops."""
    omitted_regions: int
    omitted_area_m2: float


@dataclass(frozen=True)
class OverviewResult:
    """Everything is None for a cloud with no returns; structure and relief are None
    without a measured lower surface."""

    span_m: tuple[float, float, float] | None
    """Extent of the returns along x, y, z."""
    spacing_m: float | None
    """The typical gap between neighbouring returns, to three significant digits."""
    coverage: Coverage | None
    lower_surface: LowerSurface | None
    structure: Structure | None
    relief: Relief | None


@dataclass(frozen=True)
class Overview(Query[OverviewResult]):
    """Summarize observed geometry without emitting grids or inferred objects. Cells are
    multiples of the cloud's own return spacing."""

    def run(self, cloud: PointCloud2) -> OverviewResult:
        points = finite_points(cloud).astype(np.float64)
        if not len(points):
            return OverviewResult(None, None, None, None, None, None)
        low, high = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
        area: tuple[tuple[float, float], tuple[float, float]] = (
            (float(low[0]), float(low[1])),
            (float(high[0]), float(high[1])),
        )
        # Three significant digits drop float32 noise, so equal clouds get equal cells.
        spacing = float(f"{Spacing().run(cloud):.3g}")
        fine_origin, fine_shape, fine_m, fine_limited = cover(points, 2 * spacing)
        pooled_origin, pooled_shape, pooled_m, pooled_limited = cover(points, 4 * spacing)
        fine = OverviewGrid(fine_origin, fine_shape, fine_m, fine_limited)
        pooled = OverviewGrid(pooled_origin, pooled_shape, pooled_m, pooled_limited)

        occupied = int(np.count_nonzero(Count(fine_m, area).run(cloud).values))
        ptp = np.round(np.ptp(points, axis=0), 3)
        span = (float(ptp[0]), float(ptp[1]), float(ptp[2]))
        coverage = Coverage(fine, occupied, round(occupied * fine_m**2, 3))

        lower = ZPercentile(10.0, pooled_m, min_count=4, area=area).run(cloud)
        values = lower.values[np.isfinite(lower.values)]
        observed = int(np.count_nonzero(Count(pooled_m, area).run(cloud).values))
        fraction = len(values) / observed if observed else 0.0
        measured = len(values) >= 8 and fraction >= 0.5
        reference = float(np.median(values)) if len(values) else None
        quantiles: list[float | None] = (
            np.quantile(values, (0.1, 0.5, 0.9)).tolist() if len(values) else [None] * 3
        )
        lower_surface = LowerSurface(
            pooled,
            10.0,
            4,
            len(values),
            observed,
            round(fraction, 3),
            _quantiles(*quantiles),
            None if reference is None else round(reference, 3),
            "measured" if measured else "insufficient_support",
        )
        if not measured or reference is None:
            return OverviewResult(span, spacing, coverage, lower_surface, None, None)
        return OverviewResult(
            span,
            spacing,
            coverage,
            lower_surface,
            _structure(cloud, area, fine_m, reference, (0.15, 1.0), 8),
            _relief(lower - reference, pooled_m, 0.15, 8),
        )


def _structure(
    cloud: PointCloud2,
    area: tuple[tuple[float, float], tuple[float, float]],
    cell_m: float,
    reference: float,
    band: tuple[float, float],
    max_regions: int,
) -> Structure:
    # Each region is a set of observed cells, never a solid box.
    low, high = reference + band[0], reference + band[1]
    regions = (Count(cell_m, area).run(Select(z=(low, high)).run(cloud)) > 0).regions()
    shown, rest = regions[:max_regions], regions[max_regions:]
    cell_area = cell_m**2
    return Structure(
        (round(low, 3), round(high, 3)),
        len(regions),
        round(sum(region.cell_count for region in regions) * cell_area, 3),
        tuple(
            StructureRegion(_rounded_bounds(region), round(region.cell_count * cell_area, 3))
            for region in shown
        ),
        len(rest),
        round(sum(region.cell_count for region in rest) * cell_area, 3),
    )


def _relief(offset: Grid, cell_m: float, relief_m: float, max_regions: int) -> Relief:
    # Rises and drops are labelled apart so adjacent ones cannot merge.
    signs = [
        mask.regions(gap=1, measure=offset) for mask in (offset > relief_m, offset < -relief_m)
    ]
    patches = sorted(
        (region for regions in signs for region in regions[:max_regions]),
        key=lambda region: (-region.cell_count, region.centroid),
    )
    kept = patches[:max_regions]
    total = sum(region.cell_count for regions in signs for region in regions)
    cell_area = cell_m**2
    return Relief(
        relief_m,
        sum(len(regions) for regions in signs),
        tuple(
            ReliefRegion(
                _rounded(region.centroid),
                round(region.cell_count * cell_area, 3),
                _quantiles(*_offsets(region)),
            )
            for region in kept
        ),
        sum(len(regions) for regions in signs) - len(kept),
        round((total - sum(region.cell_count for region in kept)) * cell_area, 3),
    )


def _offsets(region: Region) -> tuple[float | None, float | None, float | None]:
    stats = region.stats
    if stats is None:
        return None, None, None
    return stats.p10, stats.p50, stats.p90


def _rounded(xy: tuple[float, float]) -> tuple[float, float]:
    x, y = np.round(xy, 3).tolist()
    return x, y


def _rounded_bounds(region: Region) -> tuple[tuple[float, float], tuple[float, float]]:
    return _rounded(region.bounds[0]), _rounded(region.bounds[1])


def _quantiles(p10: float | None, p50: float | None, p90: float | None) -> Quantiles:
    p10, p50, p90 = (None if value is None else round(value, 3) for value in (p10, p50, p90))
    return Quantiles(p10, p50, p90)


def encode(cloud: PointCloud2) -> dict[str, JsonValue]:
    """The cloud's frame, timestamp, size, bounds and centroid, and its overview, as JSON."""
    points = finite_points(cloud)
    centroid: JsonValue = [round(float(v), 3) for v in points.mean(axis=0)] if len(points) else None
    return {
        "frame_id": cloud.frame_id,
        "ts": to_jsonable_python(cloud.ts),
        "num_points": len(points),
        "bounds_m": to_jsonable_python(Bounds().run(cloud)),
        "centroid_m": centroid,
        "overview": to_jsonable_python(Overview().run(cloud)),
    }
