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

"""A route through the recorded map to an answer.

The ray-traced voxel map is squashed into a 2D costmap along the robot's own
path: the cells it drove through, and a corridor around them, are known
free floor; voxels between a little below and the body height above the
nearest point of that path are obstacles; anything the robot never came near
is off limits. Obstacles are inflated by the robot's radius, and dimos's own
A* (:func:`min_cost_astar`) plans over the result, so the route that appears
in the memory world is the one the navigation stack would drive. Using the
path instead of a floor estimate makes this hold on a tilted world frame, a
split-level floor and a lidar that sees its own robot.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise
import math
from typing import TYPE_CHECKING

import numpy as np
from scipy import ndimage

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar

if TYPE_CHECKING:
    from numpy.typing import NDArray

LETHAL = 100
# Relative to the height of the nearest point of the robot's path: voxels from
# this far below it ...
BODY_BELOW_M = 0.1
# ... to this far above it are things the robot would hit.
BODY_ABOVE_M = 1.2
# Floor this far from the driven path counts as known; further out is not planned over.
CORRIDOR_M = 1.5
ROBOT_RADIUS_M = 0.35
# Cost falls from just under lethal at the robot's radius to nothing here.
INFLATION_M = 0.6
# How far a start or goal may be moved to reach a passable cell (the goal is
# usually inside the object that was asked about).
SNAP_RADIUS_M = 4.0
# The costmap is dense: on a city-scale map the cells grow so the grid stays
# about this many cells across (a 4 km ride plans on ~1 m cells).
MAX_GRID_CELLS = 4000
ROUTE_HEIGHT_BELOW_PATH_M = 0.2


def densify(path: NDArray[np.float64], step: float) -> NDArray[np.float64]:
    """*path* with points added along each leg so none are further than *step* apart."""
    if len(path) < 2:
        return path
    pieces = []
    for a, b in pairwise(path):
        n = max(math.ceil(math.dist(a[:2], b[:2]) / step), 1)
        pieces.append(a + (b - a) * np.linspace(0, 1, n, endpoint=False)[:, None])
    pieces.append(path[-1:])
    return np.concatenate(pieces)


@dataclass
class Route:
    points: list[tuple[float, float, float]]
    length_m: float
    cells: int


class RoutePlanner:
    """Plans over a 2D costmap made from the voxel map once; ``plan`` is then quick."""

    def __init__(
        self,
        costs: NDArray[np.int8],
        floor: NDArray[np.float64],
        origin_xy: tuple[float, float],
        resolution: float,
        frame_id: str = "world",
    ) -> None:
        self.costs = costs
        self.floor = floor  # height to draw a route at, per cell
        self.origin_xy = origin_xy
        self.resolution = resolution
        self._components: NDArray[np.int32] | None = None
        self.grid = OccupancyGrid(
            grid=costs,
            resolution=resolution,
            origin=Pose(position=Vector3(origin_xy[0], origin_xy[1], 0.0)),
            frame_id=frame_id,
        )

    @classmethod
    def from_voxels(
        cls,
        voxels: NDArray[np.floating],
        path: NDArray[np.floating],
        *,
        voxel_size: float,
        robot_radius_m: float = ROBOT_RADIUS_M,
        inflation_m: float = INFLATION_M,
        corridor_m: float = CORRIDOR_M,
        frame_id: str = "world",
    ) -> RoutePlanner:
        """*path* is where the robot's base was, (M, 3) in the map frame."""
        voxels = np.asarray(voxels, dtype=np.float64).reshape(-1, 3)
        path = np.asarray(path, dtype=np.float64).reshape(-1, 3)
        if len(voxels) == 0 or len(path) == 0:
            raise ValueError("no voxels or no path to plan over")
        lo = np.minimum(voxels[:, :2].min(axis=0), path[:, :2].min(axis=0))
        hi = np.maximum(voxels[:, :2].max(axis=0), path[:, :2].max(axis=0))
        resolution = max(float(voxel_size), float((hi - lo).max()) / MAX_GRID_CELLS)
        margin = corridor_m + inflation_m + resolution
        lo = lo - margin
        hi = hi + margin
        width = math.ceil((hi[0] - lo[0]) / resolution) + 1
        height = math.ceil((hi[1] - lo[1]) / resolution) + 1

        def cells(points: NDArray[np.float64]) -> tuple[NDArray[np.int64], NDArray[np.int64]]:
            c = np.clip(((points[:, 0] - lo[0]) / resolution).astype(int), 0, width - 1)
            r = np.clip(((points[:, 1] - lo[1]) / resolution).astype(int), 0, height - 1)
            return r, c

        # The path's height everywhere: each cell takes the nearest driven cell's z.
        # Consecutive samples can be several cells apart, so the legs between
        # them are filled in; otherwise the corridor would be a chain of islands.
        dense = densify(path, resolution / 2)
        driven = np.zeros((height, width), dtype=bool)
        path_z = np.full((height, width), np.nan)
        pr, pc = cells(dense)
        driven[pr, pc] = True
        path_z[pr, pc] = dense[:, 2]
        _, nearest = ndimage.distance_transform_edt(~driven, return_indices=True)
        floor = path_z[nearest[0], nearest[1]]
        known = ndimage.distance_transform_edt(~driven) * resolution <= corridor_m

        row, col = cells(voxels)
        z = voxels[:, 2] - floor[row, col]
        body = (z > -BODY_BELOW_M) & (z <= BODY_ABOVE_M)
        obstacle = np.zeros((height, width), dtype=bool)
        obstacle[row[body], col[body]] = True
        # The robot was where it drove, so voxels there are its own body or
        # people walking beside it, not walls: nothing within its radius of the
        # path blocks.
        obstacle &= ndimage.distance_transform_edt(~driven) * resolution > robot_radius_m
        # Cells the robot's footprint would overlap are lethal; a cost ramp
        # beyond that keeps the route off the walls when there is room.
        distance = ndimage.distance_transform_edt(~obstacle) * resolution
        costs = np.full((height, width), LETHAL, dtype=np.int8)
        costs[known] = 0
        ramp = (distance > robot_radius_m) & (distance < robot_radius_m + inflation_m)
        costs[ramp & known] = np.round(
            (LETHAL - 2) * (1 - (distance[ramp & known] - robot_radius_m) / inflation_m)
        ).astype(np.int8)
        costs[distance <= robot_radius_m] = LETHAL
        return cls(
            costs,
            floor - ROUTE_HEIGHT_BELOW_PATH_M,
            (float(lo[0]), float(lo[1])),
            resolution,
            frame_id,
        )

    # ---- planning ----------------------------------------------------------

    def cell_of(self, xy: tuple[float, float]) -> tuple[int, int]:
        col = int((xy[0] - self.origin_xy[0]) / self.resolution)
        row = int((xy[1] - self.origin_xy[1]) / self.resolution)
        return row, col

    def world_of(self, row: int, col: int) -> tuple[float, float]:
        return (
            self.origin_xy[0] + (col + 0.5) * self.resolution,
            self.origin_xy[1] + (row + 0.5) * self.resolution,
        )

    def floor_at(self, xy: tuple[float, float]) -> float:
        """Height to draw a route at over *xy*."""
        row, col = self.cell_of(xy)
        h, w = self.floor.shape
        value = self.floor[min(max(row, 0), h - 1), min(max(col, 0), w - 1)]
        return float(value) if np.isfinite(value) else float(np.nanmedian(self.floor))

    def passable(self, row: int, col: int) -> bool:
        h, w = self.costs.shape
        return 0 <= row < h and 0 <= col < w and 0 <= self.costs[row, col] < LETHAL

    def snap(
        self,
        xy: tuple[float, float],
        radius_m: float = SNAP_RADIUS_M,
        within: NDArray[np.bool_] | None = None,
    ) -> tuple[float, float] | None:
        """*xy* itself when passable (and in *within*), else the nearest such cell within *radius_m*."""
        row, col = self.cell_of(xy)
        h, w = self.costs.shape
        if self.passable(row, col) and (within is None or within[row, col]):
            return xy
        reach = math.ceil(radius_m / self.resolution)
        r0, r1 = max(row - reach, 0), min(row + reach + 1, h)
        c0, c1 = max(col - reach, 0), min(col + reach + 1, w)
        window = self.costs[r0:r1, c0:c1]
        ok = (window >= 0) & (window < LETHAL)
        if within is not None:
            ok &= within[r0:r1, c0:c1]
        if not ok.any():
            return None
        rows, cols = np.nonzero(ok)
        d2 = (rows + r0 - row) ** 2 + (cols + c0 - col) ** 2
        best = int(np.argmin(d2))
        if math.sqrt(float(d2[best])) * self.resolution > radius_m:
            return None
        return self.world_of(int(rows[best] + r0), int(cols[best] + c0))

    def reachable_from(self, xy: tuple[float, float]) -> NDArray[np.bool_] | None:
        """Passable cells connected to the one under *xy* (8-connected)."""
        row, col = self.cell_of(xy)
        if not self.passable(row, col):
            return None
        if self._components is None:
            passable = (self.costs >= 0) & (self.costs < LETHAL)
            self._components = ndimage.label(passable, structure=np.ones((3, 3), bool))[0]
        return self._components == self._components[row, col]

    def plan(self, start_xy: tuple[float, float], goal_xy: tuple[float, float]) -> Route | None:
        # Coarse cells (a city map) need a proportionally wider snap.
        snap_m = max(SNAP_RADIUS_M, 4 * self.resolution)
        start = self.snap(start_xy, snap_m)
        if start is None:
            return None
        # The goal is snapped to free space the start can actually reach: the
        # nearest free cell to an object may sit on a floor island or another level.
        reachable = self.reachable_from(start)
        goal = self.snap(goal_xy, snap_m, within=reachable)
        if goal is None:
            return None
        path = min_cost_astar(self.grid, goal=goal, start=start, unknown_penalty=0.8)
        if path is None or len(path.poses) < 2:
            return None
        points = [(float(p.x), float(p.y), self.floor_at((p.x, p.y))) for p in path.poses]
        length = float(sum(math.dist(a[:2], b[:2]) for a, b in pairwise(points)))
        return Route(points=points, length_m=length, cells=len(points))
