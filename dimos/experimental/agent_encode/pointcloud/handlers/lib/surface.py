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

"""What a render offers ``Pick``: its image size and what lies under chosen pixels."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass
from typing import Literal

import numpy as np
from pydantic import JsonValue

from dimos.experimental.agent_encode.pointcloud.fields import Grid
from dimos.experimental.agent_encode.pointcloud.render.overlays import Canvas
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext, Result


@dataclass(frozen=True)
class PickResult(Result):
    outcome: Literal["hit", "ambiguous", "no_return", "outside_image", "field_value"]
    """``field_value`` when the render shows a derived field no single returns make up."""
    image_size: tuple[int, int]
    selected_pixels: int
    selected_return_count: int
    bounds_m: tuple[list[float], list[float]] | None
    """Lowest and highest corner of the selected returns."""
    selection_ref: dict[str, JsonValue]

    def draw(self, canvas: Canvas) -> list[str]:
        """Mark what was found on another render; the kinds of geometry drawn."""
        return []


class PickSurface(ABC):
    """A measured render, ready to answer which returns lie under its pixels."""

    @property
    @abstractmethod
    def size(self) -> tuple[int, int]:
        """Image width, height in native pixels."""

    @abstractmethod
    def measure(
        self, pixels: np.ndarray, max_items: int, selection_ref: dict[str, JsonValue]
    ) -> tuple[PickResult, np.ndarray]:
        """The pick result for (u, v) ``pixels`` and the returns it selects."""


class Pickable(ABC):
    """A render whose pixels ``Pick`` can measure."""

    @abstractmethod
    def surface(self, ctx: EncodeContext) -> PickSurface:
        """Measure the render once against the root context."""


def bounds(points: np.ndarray) -> tuple[list[float], list[float]] | None:
    return (points.min(0).tolist(), points.max(0).tolist()) if len(points) else None


@dataclass(frozen=True)
class Contributors:
    """The returns that make up a grid's cells."""

    points: np.ndarray
    cells: np.ndarray
    """Each return's flat cell id (row * columns + column), -1 outside the grid."""

    @classmethod
    def on(cls, grid: Grid, points: np.ndarray, indices: np.ndarray) -> Contributors:
        """Contributors from each return's (column, row) ``indices`` on ``grid``."""
        inside = ((indices >= 0) & (indices < grid.shape)).all(axis=1)
        return cls(points, np.where(inside, indices[:, 1] * grid.shape[0] + indices[:, 0], -1))


@dataclass(frozen=True)
class PickedCell:
    cell: tuple[int, int]
    """(column, row)."""
    bounds_m: tuple[list[float], list[float]]
    centre_m: list[float]
    value: float | str | None
    """What the render shows there: the field's value, or the occupancy class."""
    count: int | None
    """Returns in the cell; None for a derived field."""
    min_m: float | None
    max_m: float | None
    """The cell's lowest and highest return along the grid's normal."""


@dataclass(frozen=True)
class CellPick(PickResult):
    grid: Grid
    cell_count: int
    cells: list[PickedCell]
    cells_omitted: int

    def draw(self, canvas: Canvas) -> list[str]:
        axes, normal = list(self.grid.axes), self.grid.normal
        drawn = []
        for cell in self.cells:
            if cell.min_m is None:
                continue
            (a, b), (c, d) = cell.bounds_m
            corners = np.zeros((5, 3))
            corners[:, axes] = [[a, b], [c, b], [c, d], [a, d], [a, b]]
            corners[:, normal] = cell.min_m
            canvas.path(corners)
            drawn.append("picked_cell_at_min")
        return drawn


@dataclass(frozen=True)
class GridSurface(PickSurface):
    """A top-down render: each image pixel falls in one grid cell."""

    grid: Grid
    scale: int
    """Image pixels per cell."""
    contributors: Contributors | None
    """None when the rendered field is derived and no returns belong to one cell."""
    cell_value: Callable[[int, int], float | str | None]
    """The rendered value of cell (column, row)."""

    @property
    def size(self) -> tuple[int, int]:
        return self.grid.shape[0] * self.scale, self.grid.shape[1] * self.scale

    def _picked(self, col: int, row: int) -> PickedCell:
        grid = self.grid
        lower = np.array(grid.origin) + np.array([col, row]) * grid.cell_m
        count, low, high = None, None, None
        if self.contributors is not None:
            local = self.contributors.points[
                self.contributors.cells == row * grid.shape[0] + col, grid.normal
            ]
            count = len(local)
            low, high = (float(local.min()), float(local.max())) if count else (None, None)
        return PickedCell(
            (col, row),
            (lower.tolist(), (lower + grid.cell_m).tolist()),
            (lower + grid.cell_m / 2).tolist(),
            self.cell_value(col, row),
            count,
            low,
            high,
        )

    def measure(
        self, pixels: np.ndarray, max_items: int, selection_ref: dict[str, JsonValue]
    ) -> tuple[CellPick, np.ndarray]:
        grid = self.grid
        cells = np.unique(
            np.column_stack(
                (pixels[:, 0] // self.scale, grid.shape[1] - 1 - pixels[:, 1] // self.scale)
            ),
            axis=0,
        )
        if self.contributors is None:
            selected = np.empty((0, 3), dtype=np.float32)
            outcome: Literal["hit", "no_return", "field_value"] = "field_value"
        else:
            ids = cells[:, 1] * grid.shape[0] + cells[:, 0]
            selected = self.contributors.points[np.isin(self.contributors.cells, ids)]
            outcome = "hit" if len(selected) else "no_return"
        picked = [self._picked(int(col), int(row)) for col, row in cells[:max_items]]
        result = CellPick(
            outcome,
            self.size,
            len(pixels),
            len(selected),
            bounds(selected),
            selection_ref,
            grid,
            len(cells),
            picked,
            max(0, len(cells) - max_items),
        )
        return result, selected
