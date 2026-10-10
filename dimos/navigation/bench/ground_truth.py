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
from scipy.sparse import coo_matrix, csr_matrix
from scipy.sparse.csgraph import (
    breadth_first_order,
    connected_components,
    dijkstra,
    minimum_spanning_tree,
)

from dimos.robot.unitree.go2.constants import ROBOT_WIDTH
from dimos.simulation.go2_sim.world import MOUNT_R, MOUNT_XYZ
from dimos.simulation.scenes.mjcf import SCENE_GROUPS, add_boxes
from dimos.simulation.scenes.procedural import WALL_THICKNESS, Box, Scene
from dimos.simulation.sensors.mid360.lidar import SimMid360
from dimos.simulation.sensors.mid360.pattern import POINT_RATE
from dimos.simulation.sensors.mujoco_raycaster import MujocoRaycaster

CELL = 0.05
STEP_LIMIT = 0.2
GO2_STAND_HEIGHT = 0.3
PREMAP_STEP_M = 0.1
PREMAP_SPEED = 0.5
PREMAP_SEED = 0
SEARCH_CACHE = 256
PROBE_GROUP = 5
# the Mid-360 on its mount is the highest point of the standing sim Go2
GO2_CLEARANCE_HEIGHT = 0.6
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
    stand: float


GO2 = Body(
    radius=ROBOT_WIDTH / 2, height=GO2_CLEARANCE_HEIGHT, step=STEP_LIMIT, stand=GO2_STAND_HEIGHT
)


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

    def __init__(self, scene: Scene) -> None:
        self.scene = scene
        self.body = GO2
        self.cell = CELL
        lo, hi = scene.bounds()
        self.origin = (float(lo[0]), float(lo[1]))
        self.shape = (math.ceil((hi[0] - lo[0]) / CELL), math.ceil((hi[1] - lo[1]) / CELL))
        self._probe = Probe(scene, GO2.height, CELL / 4)
        self.height = self._ground(float(hi[2]), float(hi[2] - lo[2]))
        ground = self._reachable(np.isfinite(self.height), self.index(scene.start))
        self.clearance = ndimage.distance_transform_edt(ground) * CELL
        self.walkable = ground & (self.clearance >= GO2.radius)
        self._walk = self._edges(self.walkable)
        self._graphs: dict[bool, csr_matrix] = {}
        self._searches: dict[tuple[int, bool], NDArray[np.int32]] = {}
        self._tree: csr_matrix | None = None
        self._tree_walks: dict[int, NDArray[np.int32]] = {}

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
        source = int(np.ravel_multi_index(self.index(start), self.shape))
        target = int(np.ravel_multi_index(self.index(goal), self.shape))
        predecessors = self._search(source, centered)
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
        predecessors = self._tree_walk(source)
        if target != source and predecessors[target] == NO_PREDECESSOR:
            return float(self.clearance.flat[[source, target]].min())
        narrowest = float(self.clearance.flat[source])
        node = target
        while node != source:
            narrowest = min(narrowest, float(self.clearance.flat[node]))
            node = int(predecessors[node])
        return narrowest

    def premap_cloud(self, route: NDArray[np.float64]) -> NDArray[np.float32]:
        """What the body's Mid-360 returns walking the route, in the world frame.

        One return per CELL voxel, the way a prior mapping run leaves them.
        """
        caster = MujocoRaycaster(self._probe.model, self._probe.data, SCENE_GROUPS)
        lidar = SimMid360.go2(caster, PREMAP_SEED)
        per_pose = int(POINT_RATE * PREMAP_STEP_M / PREMAP_SPEED)
        lo, hi = self.scene.bounds()
        dims = np.ceil((hi - lo) / self.cell).astype(np.int64) + 3
        seen = np.zeros(dims, dtype=bool)
        kept = []
        for position, rotation in sensor_poses(route, self.body.stand, PREMAP_STEP_M):
            points = (
                position + lidar.cast(position, rotation, per_pose).astype(np.float64) @ rotation.T
            )
            keys = np.clip(np.floor((points - lo) / self.cell).astype(np.int64) + 1, 0, dims - 1)
            flat = np.ravel_multi_index((keys[:, 0], keys[:, 1], keys[:, 2]), tuple(dims))
            _, first = np.unique(flat, return_index=True)
            first = first[~seen.flat[flat[first]]]
            seen.flat[flat[first]] = True
            kept.append(points[np.sort(first)].astype(np.float32))
        return np.concatenate(kept)

    def difficulty(self, route: Route) -> Difficulty:
        clutter = [box for box in self.scene.boxes if box.kind == "clutter"]
        return Difficulty(
            min_clearance=self.bottleneck(route.points[0], route.points[-1]),
            doors=doors_crossed(self.scene, route),
            detour=detour(route),
            clutter_near=len(boxes_near(route, clutter, CLUTTER_NEAR)),
        )

    def _ground(self, z_top: float, depth: float) -> NDArray[np.float64]:
        """Per column, the lowest surface with the body's headroom above it. The outside is not ground."""
        probe = self._probe
        caster = MujocoRaycaster(probe.model, probe.data, SCENE_GROUPS)
        height = np.full(self.shape, np.nan)
        origin = np.array([0.0, 0.0, z_top])
        for ix in range(self.shape[0]):
            origin[0] = self.origin[0] + (ix + 0.5) * self.cell
            for iy in range(self.shape[1]):
                origin[1] = self.origin[1] + (iy + 0.5) * self.cell
                origin[2] = z_top
                height[ix, iy] = _lowest_surface(caster, probe, origin, depth)
        return height

    def _search(self, source: int, centered: bool) -> NDArray[np.int32]:
        """Shortest-route predecessors from the source over the walkable graph, kept for reuse."""
        key = (source, centered)
        if key not in self._searches:
            if centered not in self._graphs:
                i, j, w = self._walk
                if centered:
                    w = (
                        w
                        * CENTERING_CLEARANCE
                        / np.clip(self.clearance.flat[j], None, CENTERING_CLEARANCE)
                    )
                n = self.shape[0] * self.shape[1]
                self._graphs[centered] = coo_matrix((w, (i, j)), shape=(n, n)).tocsr()
            _, predecessors = dijkstra(
                self._graphs[centered], indices=source, return_predecessors=True
            )
            if len(self._searches) >= SEARCH_CACHE:
                self._searches.pop(next(iter(self._searches)))
            self._searches[key] = np.asarray(predecessors, dtype=np.int32)
        return self._searches[key]

    def _widest_tree(self) -> csr_matrix:
        """A spanning tree over the walkable cells whose paths keep the most clearance."""
        if self._tree is None:
            i, j, _ = self._walk
            level = np.minimum(self.clearance.flat[i], self.clearance.flat[j])
            n = self.shape[0] * self.shape[1]
            graph = coo_matrix((level.max() + 1.0 - level, (i, j)), shape=(n, n)).tocsr()
            tree = minimum_spanning_tree(graph)
            self._tree = (tree + tree.T).tocsr()
        return self._tree

    def _tree_walk(self, source: int) -> NDArray[np.int32]:
        """Predecessors from the source along the widest tree, kept for reuse."""
        if source not in self._tree_walks:
            _, predecessors = breadth_first_order(
                self._widest_tree(), source, directed=False, return_predecessors=True
            )
            if len(self._tree_walks) >= SEARCH_CACHE:
                self._tree_walks.pop(next(iter(self._tree_walks)))
            self._tree_walks[source] = np.asarray(predecessors, dtype=np.int32)
        return self._tree_walks[source]

    def _reachable(self, ground: NDArray[np.bool_], start: tuple[int, int]) -> NDArray[np.bool_]:
        """The ground connected to the start by steps within the body's limit."""
        i, j, _ = self._edges(ground)
        labels = self._components(i, j)
        reachable: NDArray[np.bool_] = labels == labels[start]
        return reachable & ground

    def _components(self, i: NDArray[np.intp], j: NDArray[np.intp]) -> NDArray[np.int32]:
        """A component label per cell, over the given edges."""
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


def sensor_poses(
    route: NDArray[np.float64], stand: float, step: float
) -> list[tuple[NDArray[np.float64], NDArray[np.float64]]]:
    """Mid-360 poses of a body walking the route, facing along it, one every step of arc length."""
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))])
    stops = np.minimum(np.arange(0.0, arc[-1] + step / 2, step), arc[-1])
    bases = np.column_stack([np.interp(stops, arc, route[:, i]) for i in range(3)])
    poses = []
    for s, base in zip(stops, bases, strict=True):
        k = int(np.clip(np.searchsorted(arc, s, side="right"), 1, len(route) - 1))
        heading = route[k, :2] - route[k - 1, :2] if len(route) > 1 else np.array([1.0, 0.0])
        yaw = math.atan2(heading[1], heading[0])
        c, w = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, -w, 0.0], [w, c, 0.0], [0.0, 0.0, 1.0]])
        position = base + [0.0, 0.0, stand] + rotation @ MOUNT_XYZ
        poses.append((position, rotation @ MOUNT_R))
    return poses


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
        geom.group = PROBE_GROUP
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


def doors_crossed(scene: Scene, route: Route) -> int:
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


def detour(route: Route) -> float:
    """Route length over the straight-line distance between its ends."""
    straight = float(np.linalg.norm(route.points[-1, :2] - route.points[0, :2]))
    return route.length / straight if straight > 0 else 1.0


def boxes_near(route: Route, boxes: list[Box], within: float) -> list[Box]:
    """The boxes whose footprint comes within the distance of the route."""
    xy = route.points[:, :2]
    near = []
    for box in boxes:
        center, half = np.array(box.center[:2]), np.array(box.half[:2])
        outside = np.maximum(np.abs(xy - center) - half, 0.0)
        if np.any(np.hypot(outside[:, 0], outside[:, 1]) <= within):
            near.append(box)
    return near
