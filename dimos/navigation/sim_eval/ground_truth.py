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

"""Where a body can stand in a scene and how it can get from one place to another.

Ground is found by casting rays down every column of a grid against the scene's MuJoCo
model and confirming each candidate surface with a thin probe that must touch nothing
in the body's height above it. Clearance is the distance from a ground cell to the
nearest column with no ground. Routes run over the cells a body of the given radius can
stand on, between neighbors within its step height.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components, dijkstra

from dimos.robot.unitree.go2.constants import ROBOT_HEIGHT, ROBOT_WIDTH
from dimos.simulation.scenes.mjcf import add_boxes
from dimos.simulation.scenes.procedural import WALL_THICKNESS, Scene
from dimos.simulation.sensors.mujoco_raycaster import MujocoRaycaster

CELL = 0.05
STEP_LIMIT = 0.2
CENTERING_CLEARANCE = 0.6
CLUTTER_NEAR = 0.5
EPS = 1e-4
DOWN = np.array([[0.0, 0.0, -1.0]])
NEIGHBORS = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)]
NO_PREDECESSOR = -9999


@dataclass(frozen=True)
class Body:
    radius: float
    height: float
    step: float


GO2 = Body(radius=ROBOT_WIDTH / 2, height=ROBOT_HEIGHT, step=STEP_LIMIT)


@dataclass(frozen=True)
class Route:
    """Ground points from start to goal, with the clearance at each one."""

    points: NDArray[np.float64]
    clearance: NDArray[np.float64]

    @property
    def length(self) -> float:
        return float(np.linalg.norm(np.diff(self.points, axis=0), axis=1).sum())


@dataclass(frozen=True)
class Difficulty:
    """How hard a route is, from the geometry alone. min_clearance is the narrowest passage no route can avoid."""

    min_clearance: float
    doors: int
    detour: float
    clutter_near: int


class GroundTruth:
    """One ground height per column, the cells a body can stand on, and routes between them."""

    def __init__(self, scene: Scene, body: Body = GO2, cell: float = CELL) -> None:
        self.scene = scene
        self.body = body
        self.cell = cell
        lo, hi = scene.bounds()
        self.origin = (float(lo[0]), float(lo[1]))
        self.shape = (math.ceil((hi[0] - lo[0]) / cell), math.ceil((hi[1] - lo[1]) / cell))
        self.height = self._ground(float(hi[2]), float(hi[2] - lo[2]))
        ground = self._reachable(np.isfinite(self.height), self.index(scene.start))
        self.clearance = ndimage.distance_transform_edt(ground) * cell
        self.walkable = ground & (self.clearance >= body.radius)

    def index(self, point: tuple[float, ...] | NDArray[np.float64]) -> tuple[int, int]:
        ix = int((point[0] - self.origin[0]) / self.cell)
        iy = int((point[1] - self.origin[1]) / self.cell)
        if not (0 <= ix < self.shape[0] and 0 <= iy < self.shape[1]):
            raise ValueError(f"{point[:2]} is outside the scene")
        return ix, iy

    def center(self, ix: int, iy: int) -> NDArray[np.float64]:
        return np.array(
            [
                self.origin[0] + (ix + 0.5) * self.cell,
                self.origin[1] + (iy + 0.5) * self.cell,
                self.height[ix, iy],
            ]
        )

    def stands(self, point: tuple[float, ...] | NDArray[np.float64]) -> bool:
        return bool(self.walkable[self.index(point)])

    def route(
        self,
        start: tuple[float, ...] | NDArray[np.float64],
        goal: tuple[float, ...] | NDArray[np.float64],
        centered: bool = False,
    ) -> Route | None:
        """The shortest route over walkable cells, or the one that keeps away from obstacles."""
        if not (self.stands(start) and self.stands(goal)):
            return None
        i, j, w = self._edges(self.walkable)
        if centered:
            w = w * CENTERING_CLEARANCE / np.clip(self.clearance.flat[j], None, CENTERING_CLEARANCE)
        n = self.shape[0] * self.shape[1]
        graph = coo_matrix((w, (i, j)), shape=(n, n)).tocsr()
        source = np.ravel_multi_index(self.index(start), self.shape)
        target = np.ravel_multi_index(self.index(goal), self.shape)
        _, predecessors = dijkstra(graph, indices=source, return_predecessors=True)
        if target != source and predecessors[target] == NO_PREDECESSOR:
            return None
        nodes = [target]
        while nodes[-1] != source:
            nodes.append(predecessors[nodes[-1]])
        cells = np.unravel_index(nodes[::-1], self.shape)
        points = np.array([self.center(ix, iy) for ix, iy in zip(*cells, strict=True)])
        return Route(points, self.clearance[cells])

    def bottleneck(
        self,
        start: tuple[float, ...] | NDArray[np.float64],
        goal: tuple[float, ...] | NDArray[np.float64],
    ) -> float:
        """The largest clearance that every cell of some route from start to goal has."""
        source = int(np.ravel_multi_index(self.index(start), self.shape))
        target = int(np.ravel_multi_index(self.index(goal), self.shape))
        levels = np.unique(self.clearance[self.walkable])
        lo, hi = 0, len(levels) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self._connected(self.walkable & (self.clearance >= levels[mid]), source, target):
                lo = mid
            else:
                hi = mid - 1
        return float(levels[lo])

    def difficulty(self, route: Route) -> Difficulty:
        return Difficulty(
            min_clearance=self.bottleneck(route.points[0], route.points[-1]),
            doors=_doors_crossed(self.scene, route),
            detour=_detour(route),
            clutter_near=_clutter_near(self.scene, route),
        )

    def _ground(self, z_top: float, depth: float) -> NDArray[np.float64]:
        """Per column, the lowest surface with the body's headroom above it. The outside is not ground."""
        probe = Probe(self.scene, self.body.height, self.cell / 4)
        caster = MujocoRaycaster(probe.model, probe.data)
        height = np.full(self.shape, np.nan)
        origin = np.array([0.0, 0.0, z_top])
        for ix in range(self.shape[0]):
            origin[0] = self.origin[0] + (ix + 0.5) * self.cell
            for iy in range(self.shape[1]):
                origin[1] = self.origin[1] + (iy + 0.5) * self.cell
                origin[2] = z_top
                height[ix, iy] = _lowest_surface(caster, probe, origin, depth)
        return height

    def _reachable(self, ground: NDArray[np.bool_], start: tuple[int, int]) -> NDArray[np.bool_]:
        """The ground connected to the start by steps within the body's limit."""
        labels = self._components(ground)
        reachable: NDArray[np.bool_] = labels == labels[start]
        return reachable & ground

    def _connected(self, mask: NDArray[np.bool_], source: int, target: int) -> bool:
        labels = self._components(mask)
        return bool(labels.flat[source] == labels.flat[target])

    def _components(self, mask: NDArray[np.bool_]) -> NDArray[np.int32]:
        """A component label per cell, over steps between cells in the mask."""
        i, j, _ = self._edges(mask)
        n = self.shape[0] * self.shape[1]
        graph = coo_matrix((np.ones(len(i)), (i, j)), shape=(n, n)).tocsr()
        _, labels = connected_components(graph, directed=False)
        return np.asarray(labels, dtype=np.int32).reshape(self.shape)

    def _edges(
        self, mask: NDArray[np.bool_]
    ) -> tuple[NDArray[np.intp], NDArray[np.intp], NDArray[np.float64]]:
        """Directed edges between 8-neighbors that are both in the mask and within a step of each other."""
        nx, ny = self.shape
        ids = np.arange(nx * ny).reshape(self.shape)
        sources, targets, weights = [], [], []
        for dx, dy in NEIGHBORS:
            a = (slice(max(0, -dx), nx - max(0, dx)), slice(max(0, -dy), ny - max(0, dy)))
            b = (slice(max(0, dx), nx - max(0, -dx)), slice(max(0, dy), ny - max(0, -dy)))
            dz = self.height[b] - self.height[a]
            ok = mask[a] & mask[b] & (np.abs(dz) <= self.body.step)
            sources.append(ids[a][ok])
            targets.append(ids[b][ok])
            weights.append(np.hypot(self.cell * math.hypot(dx, dy), dz[ok]))
        return np.concatenate(sources), np.concatenate(targets), np.concatenate(weights)


class Probe:
    """A thin column standing on a point, which touches nothing when that point is ground."""

    def __init__(self, scene: Scene, height: float, radius: float) -> None:
        spec = mujoco.MjSpec()
        add_boxes(spec, scene)
        body = spec.worldbody.add_body(name="probe")
        body.add_freejoint()
        geom = body.add_geom(name="probe")
        geom.type = mujoco.mjtGeom.mjGEOM_BOX
        geom.size = (radius, radius, height / 2 - EPS)
        geom.pos = (0.0, 0.0, height / 2)
        self.height = height
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        mujoco.mj_forward(self.model, self.data)

    def free(self, point: NDArray[np.float64]) -> bool:
        self.data.qpos[:3] = point
        mujoco.mj_forward(self.model, self.data)
        return int(self.data.ncon) == 0


def _lowest_surface(
    caster: MujocoRaycaster, probe: Probe, origin: NDArray[np.float64], depth: float
) -> float:
    """Walk down one column. Upward faces are surface candidates, downward faces end the solid above.

    Faces that coincide, such as a wall standing on the floor, can hide a solid from the walk,
    so every candidate is confirmed by the probe.
    """
    found = math.nan
    gap_top = origin[2]
    while True:
        dist, normals = caster.cast(origin, DOWN, max_range=depth)
        if dist[0] < 0:
            return found
        z = origin[2] - dist[0]
        if normals[0, 2] > 0:
            if gap_top - z >= probe.height and probe.free(np.array([origin[0], origin[1], z])):
                found = z
        else:
            gap_top = z
        origin[2] = z - EPS


def _doors_crossed(scene: Scene, route: Route) -> int:
    """How many distinct doorways the route passes through."""
    crossed = 0
    for door in scene.doors:
        along = route.points[:, door.axis] - (door.at + WALL_THICKNESS / 2)
        across = route.points[:, 1 - door.axis]
        in_opening = (across >= door.start) & (across <= door.start + door.width)
        sign_change = np.sign(along[:-1]) != np.sign(along[1:])
        if np.any(sign_change & in_opening[:-1] & in_opening[1:]):
            crossed += 1
    return crossed


def _detour(route: Route) -> float:
    straight = float(np.linalg.norm(route.points[-1, :2] - route.points[0, :2]))
    return route.length / straight if straight > 0 else 1.0


def _clutter_near(scene: Scene, route: Route) -> int:
    """How many clutter boxes come within CLUTTER_NEAR of the route."""
    near = 0
    xy = route.points[:, :2]
    for box in scene.boxes:
        if box.kind != "clutter":
            continue
        center, half = np.array(box.center[:2]), np.array(box.half[:2])
        outside = np.maximum(np.abs(xy - center) - half, 0.0)
        if np.any(np.hypot(outside[:, 0], outside[:, 1]) <= CLUTTER_NEAR):
            near += 1
    return near
