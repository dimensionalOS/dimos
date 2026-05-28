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

"""PACK MIND — fog-of-war exploration engine (S1).

A team of dogs explores an UNKNOWN maze. Each dog reveals cells via a raycast
sensor (walls block line of sight, RTS-style). Dogs navigate to frontiers (the
boundary between known-free and unknown).

The A/B variable is the discovered map:
  - SHARED   : one known-map; every dog reveals into and plans against it.
  - INDEPENDENT: each dog has its own known-map; the *team* fog is the union of
    ONLINE dogs' maps — so taking a dog offline erases its territory from the
    team's knowledge. In shared mode the map persists regardless.

Pure numpy/scipy + DimOS min_cost_astar. No CUDA/ROS/sim deps.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from math import cos, exp, hypot, sin

import numpy as np
from numpy.typing import NDArray
from scipy.ndimage import binary_dilation, label

from dimos.experimental.pack_mind.world import (
    free_cells_world,
    load_building_world,
    make_maze_world,
    spread_starts,
)
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar

UNKNOWN = int(CostValues.UNKNOWN)
FREE = int(CostValues.FREE)
OCC = int(CostValues.OCCUPIED)

SENSOR_R_CELLS = 11  # vision radius in cells
N_RAYS = 180
STEP_M = 0.12
_CROSS = np.ones((3, 3), dtype=bool)
_MIN_CLUSTER = 4
_DOG_COLORS = ["#ff6b35", "#4dabf7", "#b197fc", "#69db7c", "#ffd43b"]


def _raycast_reveal(
    known: NDArray[np.int8],
    truth: NDArray[np.int8],
    cx: int,
    cy: int,
    visible: NDArray[np.bool_] | None = None,
) -> None:
    """Reveal cells within SENSOR_R_CELLS of (cx, cy); walls stop each ray.

    Writes discovered cells into ``known`` (persistent memory) and, if given,
    marks currently-seen cells in ``visible`` (this-tick line of sight)."""
    h, w = truth.shape

    def _see(gx: int, gy: int, val: int) -> None:
        known[gy, gx] = val
        if visible is not None:
            visible[gy, gx] = True

    if 0 <= cy < h and 0 <= cx < w:
        _see(cx, cy, OCC if truth[cy, cx] == OCC else FREE)
    for k in range(N_RAYS):
        a = 2.0 * np.pi * k / N_RAYS
        dx, dy = cos(a), sin(a)
        for r in range(1, SENSOR_R_CELLS + 1):
            gx = int(round(cx + dx * r))
            gy = int(round(cy + dy * r))
            if not (0 <= gx < w and 0 <= gy < h):
                break
            if truth[gy, gx] == OCC:
                _see(gx, gy, OCC)  # see the wall, then vision stops
                break
            _see(gx, gy, FREE)


@dataclass
class Dog:
    name: str
    color: str
    x: float
    y: float
    known: NDArray[np.int8]  # shared array (pack) or private (independent)
    online: bool = True
    idle: bool = False
    path: list[tuple[float, float]] | None = None
    idx: int = 0
    goal: tuple[float, float] | None = None  # current frontier target (for pack de-confliction)
    trail: list[tuple[float, float]] = field(default_factory=list)


class ExploreSim:
    def __init__(
        self,
        world: OccupancyGrid,
        n_dogs: int,
        shared: bool,
        starts: list[tuple[float, float]],
        seed: int = 0,
        target: tuple[float, float] | None = None,
        target_label: str = "target",
        converge_on_found: bool = False,
    ) -> None:
        np.random.seed(seed)
        self.world = world
        self.truth = world.grid
        self.shared = shared
        self.res = world.info.resolution
        self.tick_n = 0
        self._total_free = int(np.count_nonzero(self.truth == FREE))

        # Search target ("find me a red object"). Detected when its cell enters any
        # online dog's line of sight; the find is written to shared memory (found/
        # found_by/found_tick) so the whole pack knows at once.
        self.target_xy = target
        self.target_label = target_label
        self.converge_on_found = converge_on_found
        self.found = False
        self.found_by: str | None = None
        self.found_tick: int | None = None

        shape = self.truth.shape
        self._visible = np.zeros(shape, dtype=bool)
        if shared:
            self._shared_known = np.full(shape, UNKNOWN, dtype=np.int8)
        self.dogs: list[Dog] = []
        for i in range(n_dogs):
            known = self._shared_known if shared else np.full(shape, UNKNOWN, dtype=np.int8)
            self.dogs.append(
                Dog(f"dog{i}", _DOG_COLORS[i % len(_DOG_COLORS)], starts[i][0], starts[i][1], known)
            )

    # ---- geometry helpers ----
    def _grid(self, x: float, y: float) -> tuple[int, int]:
        g = self.world.world_to_grid((x, y))
        return int(g.x), int(g.y)

    # ---- fog / metrics ----
    def team_known(self) -> NDArray[np.int8]:
        """What the operator sees. Shared: the one map. Independent: union over
        ONLINE dogs (offline dogs' knowledge is lost to the team)."""
        if self.shared:
            return self._shared_known
        merged = np.full(self.truth.shape, UNKNOWN, dtype=np.int8)
        for d in self.dogs:
            if not d.online:
                continue
            seen = d.known != UNKNOWN
            merged[seen] = d.known[seen]
        return merged

    def revealed_frac(self) -> float:
        k = self.team_known()
        return float(np.count_nonzero((k == FREE) & (self.truth == FREE))) / max(1, self._total_free)

    def state(self) -> dict:
        """JSON-serializable snapshot for the web frontend. Per-cell code:
        0=unknown, 1=remembered-free, 2=visible-free, 3=remembered-wall, 4=visible-wall."""
        k = self.team_known()
        vis = self._visible
        codes = np.zeros(k.shape, dtype=np.uint8)
        free, wall = k == FREE, k == OCC
        codes[free] = 1
        codes[free & vis] = 2
        codes[wall] = 3
        codes[wall & vis] = 4
        h, w = k.shape
        return {
            "mode": "shared" if self.shared else "independent",
            "tick": self.tick_n,
            "revealed": round(self.revealed_frac(), 3),
            "w": w,
            "h": h,
            "res": self.res,
            "cells": codes.flatten().tolist(),
            "dogs": [
                {"x": round(d.x, 3), "y": round(d.y, 3), "color": d.color, "online": d.online}
                for d in self.dogs
            ],
            "target": (
                {
                    "x": round(self.target_xy[0], 3),
                    "y": round(self.target_xy[1], 3),
                    "label": self.target_label,
                    "found": self.found,
                    "found_by": self.found_by,
                    "found_tick": self.found_tick,
                }
                if self.target_xy is not None
                else None
            ),
        }

    # ---- exploration ----
    def _pick_goal(self, dog: Dog) -> tuple[float, float] | None:
        # Object found: the pack already knows (shared memory) — converge on it
        # instead of continuing to clear frontiers. "One sees it, all know."
        if self.found and self.converge_on_found and self.target_xy is not None:
            return self.target_xy
        known = dog.known
        free = known == FREE
        unknown = known == UNKNOWN
        frontier = free & binary_dilation(unknown, structure=_CROSS)
        if not frontier.any():
            return None
        lab, n = label(frontier)
        if n == 0:
            return None
        gx, gy = self._grid(dog.x, dog.y)
        # Frontiers other online dogs are already heading for. The shared map lets
        # the pack de-conflict: a cluster near a teammate's target is down-weighted
        # so the dogs fan out instead of all chasing the same boundary.
        claimed = [
            self._grid(*o.goal)
            for o in self.dogs
            if o is not dog and o.online and o.goal is not None
        ]
        rows, cols = np.where(frontier)
        ids = lab[rows, cols]
        best_id, best_score = -1, -1.0
        for cid in range(1, n + 1):
            m = ids == cid
            size = int(np.count_nonzero(m))
            if size < _MIN_CLUSTER:
                continue
            cr, cc = rows[m].mean(), cols[m].mean()
            dist = max(1.0, hypot(cc - gx, cr - gy))
            score = size / dist
            if claimed:
                d_other = min(hypot(cc - ox, cr - oy) for ox, oy in claimed)
                score *= 1.0 - 0.85 * exp(-(d_other**2) / (2.0 * SENSOR_R_CELLS**2))
            if score > best_score:
                best_score, best_id = score, cid
        if best_id < 0:
            return None
        m = ids == best_id
        cr, cc = rows[m], cols[m]
        d2 = (cc - gx) ** 2 + (cr - gy) ** 2
        j = int(np.argmin(d2))  # nearest frontier cell in best cluster
        wp = self.world.grid_to_world((int(cc[j]), int(cr[j]), 0))
        return (wp.x, wp.y)

    def _plan(self, dog: Dog, goal: tuple[float, float]) -> bool:
        costmap = OccupancyGrid(grid=dog.known, resolution=self.res, frame_id="world")
        path = min_cost_astar(costmap, goal, (dog.x, dog.y), unknown_penalty=1.0, use_cpp=False)
        if path is None or len(path.poses) == 0:
            return False
        dog.path = [(p.position.x, p.position.y) for p in path.poses]
        dog.idx = 0
        return True

    def _advance(self, dog: Dog) -> None:
        if dog.path is None or dog.idx >= len(dog.path):
            goal = self._pick_goal(dog)
            if goal is None or not self._plan(dog, goal):
                dog.idle = True
                dog.goal = None
                return
            dog.goal = goal
            dog.idle = False
        assert dog.path is not None
        tx, ty = dog.path[dog.idx]
        dx, dy = tx - dog.x, ty - dog.y
        dist = hypot(dx, dy)
        if dist <= STEP_M or dist == 0:
            dog.x, dog.y = tx, ty
            dog.idx += 1
        else:
            dog.x += STEP_M * dx / dist
            dog.y += STEP_M * dy / dist

    def set_online(self, dog_index: int, online: bool) -> None:
        self.dogs[dog_index].online = online

    def step(self) -> None:
        self.tick_n += 1
        self._visible = np.zeros(self.truth.shape, dtype=bool)
        for d in self.dogs:
            if d.online:
                gx, gy = self._grid(d.x, d.y)
                _raycast_reveal(d.known, self.truth, gx, gy, self._visible)
        self._check_target()
        for d in self.dogs:
            if d.online:
                self._advance(d)
                d.trail.append((d.x, d.y))

    def _check_target(self) -> None:
        """Mark the target found the tick its cell first enters any dog's sight."""
        if self.target_xy is None or self.found:
            return
        tgx, tgy = self._grid(*self.target_xy)
        h, w = self._visible.shape
        if not (0 <= tgy < h and 0 <= tgx < w) or not self._visible[tgy, tgx]:
            return
        self.found = True
        self.found_tick = self.tick_n
        online = [d for d in self.dogs if d.online]
        if online:
            tx, ty = self.target_xy
            self.found_by = min(online, key=lambda d: (d.x - tx) ** 2 + (d.y - ty) ** 2).name

    def all_done(self) -> bool:
        return all((not d.online) or d.idle for d in self.dogs)

    def run(self, max_ticks: int = 4000, target: float = 0.95, kill: tuple[int, int] | None = None):
        t_target: int | None = None
        for _ in range(max_ticks):
            if kill is not None and self.tick_n == kill[1]:
                self.set_online(kill[0], False)
            self.step()
            if t_target is None and self.revealed_frac() >= target:
                t_target = self.tick_n
            if self.all_done():
                break
        return {
            "mode": "shared" if self.shared else "independent",
            "ticks": self.tick_n,
            "revealed": self.revealed_frac(),
            "ticks_to_target": t_target,
        }


_START_POOL = [(0.6, 0.5), (3.4, 0.5), (5.4, 0.5), (0.6, 5.4), (5.4, 5.4)]


def _far_target(world: OccupancyGrid, starts: list[tuple[float, float]]) -> tuple[float, float]:
    """Plant the object at the free cell farthest from every start, so it's only
    found after real searching — not next to where a dog spawns."""
    cells = free_cells_world(world)
    return max(cells, key=lambda c: min((c[0] - s[0]) ** 2 + (c[1] - s[1]) ** 2 for s in starts))


def build_explore(
    shared: bool,
    seed: int = 0,
    n_dogs: int = 3,
    target_label: str | None = None,
    converge_on_found: bool = False,
) -> ExploreSim:
    world = make_maze_world()
    # Spread deployment (different entry points, realistic multi-dog SAR). Both
    # modes use the SAME starts — only shared-vs-private memory differs. Spreading
    # gives each dog distinct territory, so in independent mode losing a dog loses
    # its discoveries (beat 4), while shared retains them.
    starts = [_START_POOL[i % len(_START_POOL)] for i in range(n_dogs)]
    target = _far_target(world, starts) if target_label else None
    return ExploreSim(
        world, n_dogs, shared, starts, seed,
        target=target, target_label=target_label or "target",
        converge_on_found=converge_on_found,
    )


def build_explore_building(
    shared: bool,
    seed: int = 0,
    n_dogs: int = 3,
    downsample: int = 4,
    target_label: str | None = None,
    converge_on_found: bool = False,
) -> ExploreSim:
    """ExploreSim on a real DimOS SLAM floor plan instead of the synthetic maze.

    A 27x37m office footprint (big_office_simple_occupancy.png) with long wings
    and corridors. Starts are spread deterministically from the seed, so the
    shared and independent runs share identical deployment — only the memory
    model differs. The bigger, concave floor is where the shared-vs-independent
    gap blows open (shared clears 100%, independent stalls redundantly)."""
    world = load_building_world(downsample=downsample)
    starts = spread_starts(world, n_dogs, seed=seed, min_dist_m=5.0)
    target = _far_target(world, starts) if target_label else None
    return ExploreSim(
        world, n_dogs, shared, starts, seed,
        target=target, target_label=target_label or "target",
        converge_on_found=converge_on_found,
    )


if __name__ == "__main__":
    import time

    builder = build_explore_building if os.environ.get("PACK_MIND_MAP") == "building" else build_explore
    for sh in (True, False):
        t0 = time.perf_counter()
        r = builder(shared=sh).run()
        print(r, "wall=%.1fs" % (time.perf_counter() - t0))
