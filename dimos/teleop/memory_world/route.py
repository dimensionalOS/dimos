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

Two planners. :class:`MlsRoutePlanner` is the one to use: dimos's multi-level
surface planner (``dimos.navigation.nav_3d.mls_planner``, Rust) builds
standable surfaces and a node graph from the ray-traced voxels and plans in
3D with terrain traversability. :class:`RoutePlanner` is the fallback when
that binding is not installed: a 2D costmap along the robot's driven path
with dimos's ``min_cost_astar``.

The 2D fallback squashes the ray-traced voxel map into a costmap along the
robot's own path: the cells it drove through, and a corridor around them, are
known free floor; voxels between a little below and the body height above the
nearest point of that path are obstacles; anything the robot never came near
is off limits. Obstacles are inflated by the robot's radius and A* plans over
the result.
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
# The most that may ever be treated as "the robot drove between these two poses", however
# sparsely the recording was sampled. See `bridgeable_gap`.
MAX_BRIDGE_M = 1.5
# Cost falls from just under lethal at the robot's radius to nothing here.
INFLATION_M = 0.6
# How far a start or goal may be moved to reach a passable cell (the goal is
# usually inside the object that was asked about).
SNAP_RADIUS_M = 4.0
# The costmap is dense: on a city-scale map the cells grow so the grid stays
# about this many cells across (a 4 km ride plans on ~1 m cells).
MAX_GRID_CELLS = 4000
ROUTE_HEIGHT_BELOW_PATH_M = 0.2
ROUTE_HEIGHT_ABOVE_SURFACE_M = 0.1


def densify(
    path: NDArray[np.float64], step: float, max_gap: float | None = None
) -> NDArray[np.float64]:
    """*path* with points added along each leg so none are further than *step* apart.

    With *max_gap*, a leg longer than that is left alone: its two ends stay and nothing
    is drawn between them. A pose series contains both drives and jumps, and only the
    drive is somewhere the robot was.
    """
    if len(path) < 2:
        return path
    pieces = []
    for a, b in pairwise(path):
        span = math.dist(a[:2], b[:2])
        n = 1 if (max_gap is not None and span > max_gap) else max(math.ceil(span / step), 1)
        pieces.append(a + (b - a) * np.linspace(0, 1, n, endpoint=False)[:, None])
    pieces.append(path[-1:])
    return np.concatenate(pieces)


def bridgeable_gap(path: NDArray[np.float64]) -> float:
    """How far apart two poses may be and still have the robot between them.

    From the recording's own sampling. The statistic is the 90th percentile of the leg
    lengths, doubled and capped -- NOT the median, and not the median over legs above
    some floor. Both of those were tried and both were wrong, in opposite directions, on
    real recording shapes:

      median, all legs            a five-second pause fills the list with millimetre
                                  legs (real odometry never repeats a pose exactly) and
                                  drags it to 0.003 m, so nothing is bridged and the
                                  robot's own body between the drive samples reads as
                                  walls -- a straight corridor plans no route at all.
      median, legs over one cell  on a drive sampled every 5 cm with a 10 cm cell, that
                                  filter removes EVERY drive leg and leaves only the
                                  jump, so the jump becomes the median and gets bridged:
                                  the wall it crosses drops from cost 100 to 90 and the
                                  route goes straight through it.

    The 90th percentile needs no filter, because it is already above the small tail a
    pause makes and below the large one a relocalisation makes. Measured on all four
    known shapes -- coarse drive, coarse drive with a pause, fine drive with a 1.2 m
    jump, fine drive with a 2.8 m jump -- it is the only one of the three that bridges
    every drive leg and no jump.
    """
    if len(path) < 2:
        return 0.0
    gaps = np.linalg.norm(np.diff(np.asarray(path)[:, :2], axis=0), axis=1)
    if not len(gaps):
        return 0.0
    return float(min(MAX_BRIDGE_M, 2.0 * float(np.percentile(gaps, 90))))


@dataclass
class Route:
    points: list[tuple[float, float, float]]
    length_m: float
    cells: int
    planner: str = "costmap"


# How far a start or goal may be moved onto a graph node, and how many nearby
# nodes are tried as the goal (the four nearest as the start) before giving up.
MLS_SNAP_RADIUS_M = 4.0
MLS_SNAP_CANDIDATES = 24
# Above this many map voxels the MLS graph build is skipped (a city ride) and
# the costmap fallback plans instead.
MLS_MAX_VOXELS = 4_000_000


def mls_available() -> bool:
    try:
        import dimos_mls_planner  # noqa: F401
    except ImportError:
        return False
    return True


class MlsRoutePlanner:
    """dimos's 3D multi-level-surface planner over the ray-traced voxel map.

    ``update_global_map`` (seconds on a building) runs once at construction;
    ``plan`` tries the graph nodes near the start and the goal, nearest first by
    planar distance plus twice the height gap, within :data:`MLS_SNAP_RADIUS_M`
    (an answer is usually inside the object that was asked about), and returns
    the 3D waypoints of the first pair that connects.
    """

    def __init__(
        self,
        voxels: NDArray[np.floating],
        *,
        voxel_size: float,
        robot_height_m: float = 1.2,
        node_spacing_m: float = 0.5,
    ) -> None:
        from dimos_mls_planner import (
            MLSPlanner,
        )  # the Rust binding dimos.navigation.nav_3d.mls_planner re-exports

        points = np.ascontiguousarray(np.asarray(voxels, dtype=np.float32).reshape(-1, 3))
        if len(points) == 0:
            raise ValueError("no voxels to plan over")
        self.voxel_size = float(voxel_size)
        self.planner = MLSPlanner(
            voxel_size=max(self.voxel_size, 0.1),
            robot_height=robot_height_m,
            node_spacing_m=node_spacing_m,
        )
        self.planner.update_global_map(points)
        self.surface_cells = len(self.planner.surface_map())  # (M, 3) centres; for the log
        # Plans start and end on graph nodes; the planner snaps a point to its nearest
        # node in 3D, which under a shelf is the shelf top. So candidates are nodes.
        self.nodes = np.asarray(self.planner.nodes(), dtype=np.float64).reshape(-1, 3)

    def candidates(self, xyz: tuple[float, float, float]) -> list[tuple[float, float, float]]:
        """Standable cells near *xyz*, nearest first by horizontal distance, cells at or
        below the point's height first: an answer's z is the object's, and the object
        stands on a floor, not on the shelf top the planner also calls a surface."""
        if len(self.nodes) == 0:
            return []
        d = np.linalg.norm(self.nodes[:, :2] - np.asarray(xyz[:2], dtype=np.float64), axis=1)
        near = np.flatnonzero(d <= MLS_SNAP_RADIUS_M)
        if len(near) == 0:
            return []
        above = self.nodes[near, 2] > xyz[2] + 0.3
        # Ranked by planar distance plus twice the height gap, so the surface the point
        # sits on beats one under it; nodes above the point come last.
        closeness = d[near] + 2.0 * np.abs(self.nodes[near, 2] - xyz[2])
        order = near[np.lexsort((closeness, above))][:MLS_SNAP_CANDIDATES]
        return [tuple(float(v) for v in self.nodes[i]) for i in order]  # type: ignore[misc]

    def plan(
        self, start: tuple[float, float, float], goal: tuple[float, float, float]
    ) -> Route | None:
        """The first reachable pairing of nearby start and goal cells (a cell on another
        level, or an island the map never connected, is skipped)."""
        starts = self.candidates(start)[:4]
        goals = self.candidates(goal)
        path = None
        for a in starts:
            for b in goals:
                path = self.planner.plan(a, b)
                if path is not None and len(path) >= 2:
                    break
            if path is not None and len(path) >= 2:
                break
        if path is None or len(path) < 2:
            return None
        points = [
            (float(x), float(y), float(z) + ROUTE_HEIGHT_ABOVE_SURFACE_M)
            for x, y, z in np.asarray(path)
        ]
        length = float(sum(math.dist(p, q) for p, q in pairwise(points)))
        return Route(points=points, length_m=length, cells=len(points), planner="mls")


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
        off_path, nearest = ndimage.distance_transform_edt(~driven, return_indices=True)
        off_path *= resolution  # metres from the driven path
        floor = path_z[nearest[0], nearest[1]]
        known = off_path <= corridor_m

        row, col = cells(voxels)
        z = voxels[:, 2] - floor[row, col]
        body = (z > -BODY_BELOW_M) & (z <= BODY_ABOVE_M)
        obstacle = np.zeros((height, width), dtype=bool)
        obstacle[row[body], col[body]] = True
        # The robot was where it drove, so voxels there are its own body or people
        # walking beside it, not walls: nothing within its radius of the path blocks.
        #
        # From a line that bridges the DRIVES and not the jumps -- not from `dense`,
        # which bridges a gap of any length, and not from the bare samples either.
        #
        # `dense` was wrong because one SLAM jump across a room made this erase the real
        # wall voxels the straight line passed through, and the planner then routed
        # through the hole it had just made: a 2 m jump took the wall from cost 100 to
        # 88-90 and gave a 7.90 m plan through it. The bare samples were wrong for the
        # opposite reason: on a recording sampled every metre, the robot's own body
        # sits between the samples, and refusing to erase it walled off a straight 10 m
        # corridor completely -- `plan` returned None where a 9.20 m route existed.
        #
        # `bridgeable_gap` tells the two apart from the recording's own sampling. The
        # corridor and the floor height still come from `dense`, which is what they are
        # for and what needs ~3 m of bridging on a real recording.
        driven_line = densify(path, resolution / 2, max_gap=bridgeable_gap(path))
        sampled = np.zeros((height, width), dtype=bool)
        sampled_r, sampled_c = cells(driven_line)
        sampled[sampled_r, sampled_c] = True
        off_sampled = ndimage.distance_transform_edt(~sampled) * resolution
        obstacle &= off_sampled > robot_radius_m
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
        # floor, not int(): int() truncates toward zero, so a point up to one cell BELOW
        # the origin landed on cell 0 instead of -1 and passed the bounds check that
        # every caller makes on the result. world_of's `+ 0.5` is floor's inverse.
        col = math.floor((xy[0] - self.origin_xy[0]) / self.resolution)
        row = math.floor((xy[1] - self.origin_xy[1]) / self.resolution)
        return int(row), int(col)

    def world_of(self, row: int, col: int) -> tuple[float, float]:
        return (
            self.origin_xy[0] + (col + 0.5) * self.resolution,
            self.origin_xy[1] + (row + 0.5) * self.resolution,
        )

    def floor_at(self, xy: tuple[float, float]) -> float:
        """Height to draw a route at over *xy*."""
        row, col = self.cell_of(xy)
        h, w = self.floor.shape
        return float(self.floor[min(max(row, 0), h - 1), min(max(col, 0), w - 1)])

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
        # Distance from the POINT that was asked about, not from the cell it fell in.
        # Ranking by cell index makes every cell in a ring equidistant, so argmin took
        # whichever came first in row-major order: asked for (0.1, 0.9) on a 1 m grid,
        # this returned the centre 1.46 m away while one 0.72 m away sat in the ring too.
        # It also made `radius_m` a bound on cell hops rather than on metres, so the cell
        # handed back could be further than the caller allowed.
        centres_x = self.origin_xy[0] + (cols + c0 + 0.5) * self.resolution
        centres_y = self.origin_xy[1] + (rows + r0 + 0.5) * self.resolution
        d2 = (centres_x - xy[0]) ** 2 + (centres_y - xy[1]) ** 2
        best = int(np.argmin(d2))
        if math.sqrt(float(d2[best])) > radius_m:
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
        path = min_cost_astar(self.grid, goal=goal, start=start)  # no unknown cells here
        if path is None or len(path.poses) < 2:
            return None
        # OccupancyGrid.grid_to_world is `origin + cell * resolution`, which is the cell's
        # CORNER, while snap() and world_of() work in centres. Taken as-is the drawn route
        # sits half a cell down and left of the cells it was planned through: 4 cm at
        # voxel_size 0.08, but half a metre on a city map, where the cells reach a metre
        # and half of one is more than ROBOT_RADIUS_M.
        half = self.resolution / 2
        centres = [(float(p.x) + half, float(p.y) + half) for p in path.poses]
        points = [(x, y, self.floor_at((x, y))) for x, y in centres]
        length = float(sum(math.dist(a[:2], b[:2]) for a, b in pairwise(points)))
        return Route(points=points, length_m=length, cells=len(points))
