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

"""Agent-facing occupancy-grid encoding."""

from __future__ import annotations

import base64
import math
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


_OCCUPIED_THRESHOLD = 50
_MIN_COMPONENT_AREA_M2 = 0.04


def _structural_cells(cells: np.ndarray, resolution: float) -> np.ndarray:
    """Remove physically small occupied specks without changing map topology."""
    import cv2

    if resolution <= 0:
        raise ValueError("Occupancy grid resolution must be positive")

    known = cells >= 0
    occupied = (cells >= _OCCUPIED_THRESHOLD).astype(np.uint8)
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        occupied, connectivity=8
    )
    retained = np.zeros_like(occupied)
    min_component_cells = max(
        1, math.ceil(_MIN_COMPONENT_AREA_M2 / resolution**2 - 1e-12)
    )
    for label in range(1, component_count):
        if stats[label, cv2.CC_STAT_AREA] >= min_component_cells:
            retained[labels == label] = 1

    structural = np.full(cells.shape, -1, dtype=np.int8)
    structural[known] = 0
    structural[retained.astype(bool)] = 100
    return structural


def occupancy_grid_agent_encode(grid: OccupancyGrid) -> list[dict[str, Any]]:
    """Encode metadata and a cleaned, grid-oriented structural map."""
    origin = grid.origin.position
    orientation = grid.origin.orientation
    metadata = (
        f"OccupancyGrid frame={grid.frame_id!r}, size={grid.width}x{grid.height} cells, "
        f"resolution={grid.resolution:g} m/cell, origin=({origin.x:g}, {origin.y:g}) m, "
        f"origin_orientation=({orientation.x:g}, {orientation.y:g}, {orientation.z:g}, "
        f"{orientation.w:g}), timestamp={grid.ts:g} s. "
        f"Source cells: occupied={grid.occupied_percent:.1f}%, free={grid.free_percent:.1f}%, "
        f"unknown={grid.unknown_percent:.1f}%. The agent image is a cleaned structural "
        f"view: costs >= {_OCCUPIED_THRESHOLD} are occupied, occupied components smaller "
        f"than {_MIN_COMPONENT_AREA_M2:g} m^2 are removed. Map image legend: white=free, "
        "black=occupied, gray=unknown. This is a ground-plane navigation occupancy grid. "
        "White cells are observed traversable free space. "
        "Black cells are occupied obstacles that the robot cannot enter or cross. Gray "
        "cells are unobserved or unknown, not free; treat them as impassable when reasoning "
        "about navigation. Spatial answers must respect these cell semantics. "
        "Image left/right is -X/+X and bottom/top is -Y/+Y in the grid frame."
    )
    blocks: list[dict[str, Any]] = [{"type": "text", "text": metadata}]
    if grid.grid.size == 0:
        return blocks

    cells = _structural_cells(grid.grid, grid.resolution)
    image = np.full((*grid.grid.shape, 3), 255, dtype=np.uint8)
    image[cells < 0] = 127
    image[cells >= 100] = 0
    image = np.flipud(image)

    import cv2

    success, encoded = cv2.imencode(".png", image)
    if not success:
        raise ValueError("Failed to encode occupancy grid as PNG")
    payload = base64.b64encode(encoded.tobytes()).decode("ascii")
    blocks.append(
        {
            "type": "image_url",
            "image_url": {"url": f"data:image/png;base64,{payload}"},
        }
    )
    return blocks
