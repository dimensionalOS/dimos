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

"""The no-argument default: a compact recipe composed from the fields available to callers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from dimos.experimental.agent_encode.pointcloud import constants
from dimos.experimental.agent_encode.pointcloud.fields import (
    Band,
    Components,
    Grid,
    HeightField,
    Select,
    Threshold,
)
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext


def cover(points: np.ndarray, cell: float) -> tuple[Grid, bool]:
    """Square XY cells covering every return, aligned to multiples of ``cell``.

    Alignment and cell size do not depend on the cloud's extent, so the same
    scene measures the same in any frame that holds it. The cell grows only when
    the grid would exceed ``constants.MAX_GRID_CELLS``, the encoder's memory guard;
    the flag reports that it did.
    """
    low, high = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
    cell, limited = round(cell, 6), False
    while True:
        index = np.floor(low / cell)
        origin = np.round(index * cell, 6)
        origin = np.where(origin > low, np.round((index - 1) * cell, 6), origin)
        shape = np.floor((high - origin) / cell).astype(np.int64) + 1
        if shape[0] * shape[1] <= constants.MAX_GRID_CELLS:
            break
        excess = shape[0] * shape[1] / constants.MAX_GRID_CELLS
        cell = float(np.ceil(cell * np.sqrt(excess) * 1000) / 1000)
        limited = True
    return Grid((float(origin[0]), float(origin[1])), (int(shape[0]), int(shape[1])), cell), limited


def _describe(grid: Grid, limited: bool) -> dict[str, Any]:
    out: dict[str, Any] = {
        "origin": list(grid.origin),
        "shape": list(grid.shape),
        "cell_m": grid.cell_m,
    }
    if limited:
        out["limited_by"] = "grid_cell_limit"
    return out


@dataclass(frozen=True)
class Overview:
    """Summarize observed geometry without emitting grids or inferred objects.

    XY/Z are cloud axes, and cells are multiples of the cloud's own return spacing.
    Explicit fields remain available for other orientations, bands and resolutions.
    """

    max_regions: int = 8
    """Regions listed for structure and for relief, largest first; the rest are counted."""
    percentile: float = 10.0
    """Percentile of Z taken in each cell as the lower surface."""
    min_count: int = 4
    """Returns a cell needs for its percentile to count as supported."""
    band: tuple[float, float] = (0.15, 1.0)
    """The structure band, relative to the median supported lower-return height, not a
    robot pose or a verified floor."""
    relief_m: float = 0.15
    """Lower-surface cells further than this from the reference height are relief."""
    cell_spacings: int = 2
    """Return spacings per cell for coverage and structure."""
    relief_cell_spacings: int = 4
    """Return spacings per cell for the lower surface, pooled so each cell's percentile
    has returns behind it."""
    relief_gap_cells: int = 1
    """Relief cells this far apart still form one patch, as ``Components.gap_cells``."""
    min_supported_cells: int = 8
    """Supported cells the lower surface needs to count as measured."""
    min_supported_fraction: float = 0.5
    """Share of the observed cells that must be supported for the same. Without a
    measured lower surface there is no reference height, so structure and relief are
    left out."""

    def __post_init__(self) -> None:
        if type(self.max_regions) is not int or not 1 <= self.max_regions <= 64:
            raise ValueError("max_regions must be an integer between 1 and 64")
        if not np.isfinite(self.percentile) or not 0 <= self.percentile <= 100:
            raise ValueError("percentile must be between 0 and 100")
        if type(self.min_count) is not int or self.min_count < 1:
            raise ValueError("min_count must be a positive integer")
        if len(self.band) != 2 or not np.isfinite(self.band).all() or self.band[0] >= self.band[1]:
            raise ValueError("band must have finite, increasing endpoints")
        if not np.isfinite(self.relief_m) or self.relief_m <= 0:
            raise ValueError("relief_m must be finite and positive")
        for name in ("cell_spacings", "relief_cell_spacings", "min_supported_cells"):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f"{name} must be a positive integer")
        if type(self.relief_gap_cells) is not int or self.relief_gap_cells < 0:
            raise ValueError("relief_gap_cells must be a non-negative integer")
        if not np.isfinite(self.min_supported_fraction) or not (
            0 <= self.min_supported_fraction <= 1
        ):
            raise ValueError("min_supported_fraction must be between 0 and 1")

    def run(self, ctx: EncodeContext) -> dict[str, Any]:
        if not len(ctx.points):
            return {"handler": "Overview", "status": "no_returns"}
        points = ctx.points.astype(np.float64)
        # Three significant digits drop float32 noise, so equal clouds get equal cells.
        spacing = float(f"{ctx.spacing_m:.3g}")
        fine, fine_limited = cover(points, self.cell_spacings * spacing)
        pooled, pooled_limited = cover(points, self.relief_cell_spacings * spacing)

        occupied = int(np.count_nonzero(ctx.evaluate(HeightField(fine)).values["count"]))
        out: dict[str, Any] = {
            "handler": "Overview",
            "span_m": np.round(np.ptp(points, axis=0), 3).tolist(),
            "spacing_m": spacing,
            "coverage": {
                "grid": _describe(fine, fine_limited),
                "occupied_cells": occupied,
                "observed_xy_area_m2": round(occupied * fine.cell_m**2, 3),
            },
        }

        height = HeightField(pooled)
        lower = height.percentile(self.percentile, min_count=self.min_count)
        values = ctx.evaluate(lower).scalar()
        values = values[np.isfinite(values)]
        observed = int(np.count_nonzero(ctx.evaluate(height).values["count"]))
        fraction = len(values) / observed if observed else 0.0
        measured = (
            len(values) >= self.min_supported_cells and fraction >= self.min_supported_fraction
        )
        reference = float(np.median(values)) if len(values) else None
        quantiles = np.quantile(values, (0.1, 0.5, 0.9)) if len(values) else (None,) * 3
        out["lower_surface"] = {
            "grid": _describe(pooled, pooled_limited),
            "percentile": self.percentile,
            "min_returns_per_cell": self.min_count,
            "supported_cells": len(values),
            "observed_cells": observed,
            "supported_fraction": round(fraction, 3),
            "z_quantiles_m": _quantiles(quantiles),
            "reference_z_m": None if reference is None else round(reference, 3),
            "status": "measured" if measured else "insufficient_support",
        }
        if not measured or reference is None:
            return out

        # Each region is a set of observed cells, never a solid box.
        low, high = reference + self.band[0], reference + self.band[1]
        structure = HeightField(fine, Select(Band("z", low, high, closed=(True, True))))
        table = ctx.evaluate(
            Components(Threshold(structure.count, ">", 0), max_regions=self.max_regions)
        ).metadata
        area = fine.cell_m**2
        shown = sum(region["cells"] for region in table["regions"])
        out["structure"] = {
            "z_range_m": [round(low, 3), round(high, 3)],
            "region_count": table["region_count"],
            "observed_area_m2": round((shown + table["omitted_cells"]) * area, 3),
            "regions": [
                {
                    "bounds": np.round(region["bounds"], 3).tolist(),
                    "area_m2": round(region["cells"] * area, 3),
                }
                for region in table["regions"]
            ],
            "omitted_regions": table["omitted_regions"],
            "omitted_area_m2": round(table["omitted_cells"] * area, 3),
        }

        # Rises and drops are labelled apart so adjacent ones cannot merge.
        offset = lower - reference
        tables = [
            ctx.evaluate(
                Components(
                    Threshold(offset, comparison, value),
                    gap_cells=self.relief_gap_cells,
                    values=offset,
                    max_regions=self.max_regions,
                )
            ).metadata
            for comparison, value in ((">", self.relief_m), ("<", -self.relief_m))
        ]
        patches = [region for table in tables for region in table["regions"]]
        patches.sort(key=lambda region: (-region["cells"], region["centroid"]))
        kept, dropped = patches[: self.max_regions], patches[self.max_regions :]
        area = pooled.cell_m**2
        out["relief"] = {
            "offset_threshold_m": self.relief_m,
            "region_count": sum(table["region_count"] for table in tables),
            "regions": [
                {
                    "centroid": np.round(region["centroid"], 3).tolist(),
                    "area_m2": round(region["cells"] * area, 3),
                    "offset_quantiles_m": _quantiles((region["p10"], region["p50"], region["p90"])),
                }
                for region in kept
            ],
            "omitted_regions": len(dropped) + sum(table["omitted_regions"] for table in tables),
            "omitted_area_m2": round(
                (
                    sum(region["cells"] for region in dropped)
                    + sum(table["omitted_cells"] for table in tables)
                )
                * area,
                3,
            ),
        }
        return out


def _quantiles(values: Any) -> dict[str, float | None]:
    return {
        name: None if value is None else round(float(value), 3)
        for name, value in zip(("p10", "p50", "p90"), values, strict=True)
    }
