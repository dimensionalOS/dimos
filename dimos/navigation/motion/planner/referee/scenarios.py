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

"""Ground-truth worlds for the sim2d battery: curated + seeded generation.

Generated worlds are deterministic per seed, obey placement rules (no sliver
gaps, free spawn/goal disks), and auto-label their expectation from truth:
a BFS on the exact occupancy with a conservative body disk (any-orientation)
proves "clear"; infeasible even for an optimistic disk proves "refuse";
between the two it is "safe" (never a non-vetoed interpenetration).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import hashlib
import inspect
import math
import os
from pathlib import Path as FilePath
import pickle
import sys
from typing import Any

import numpy as np

from .types import Path, PoseStamped


@dataclass(frozen=True)
class Embodiment:
    """The robot under test — conditions the gold oracle, the generator's
    difficulty rules, and the judge. A net trained on varied embodiments
    deploys on a new robot by being handed a new one of these.

    comfort = obstacles-we-care-about radius (preference, tunable);
    precision = local control tracking accuracy (hard floor — clearance
    below it is fiction, planning it is planning a contact).
    """

    # Moving-body envelope measured in the fitted MuJoCo sim (union of all
    # robot geometry over stand/fwd/reverse/strafe/spin/arc/crab commands,
    # yaw-aligned base frame): the swinging legs, not the 0.31 m trunk, set
    # the width. Measured 0.852 x 0.495, centre +x offset -0.009.
    tag: str = "go2"
    length: float = 0.85
    width: float = 0.50
    center_off: float = -0.01  # body center relative to the pose point
    comfort: float = 0.4
    precision: float = 0.05
    # gait cost multipliers for the SE(2) reference (and any rollout later):
    # forward = 1; strafe/reverse scale it; yaw_w prices rotation per rad.
    strafe: float = 1.8
    reverse: float = 1.5
    yaw_w: float = 0.25

    @property
    def half_diag(self) -> float:
        return math.hypot(self.length, self.width) / 2.0

    def offsets(self, step: float = 0.05) -> np.ndarray:
        """Footprint sample points, dense enough that thin slats can't slip."""
        hl, hw = self.length / 2.0, self.width / 2.0
        return np.array(
            [
                (x + self.center_off, y)
                for x in np.arange(-hl, hl + step / 2.0, step)
                for y in np.arange(-hw, hw + step / 2.0, step)
            ]
        )


GO2 = Embodiment()
EMBODIMENTS = {
    "go2": GO2,
    # payload adds 8 cm in front: longer body, centre 4 cm further forward
    "go2-payload": Embodiment(tag="go2-payload", length=0.93, center_off=0.03, comfort=0.5),
    "slim": Embodiment(tag="slim", length=2.0, width=0.24, comfort=0.3),
    "diffdrive": Embodiment(tag="diffdrive", strafe=50.0, reverse=3.0),  # cannot crab
}


@dataclass
class Box:
    """Ground-truth obstacle: axis of yaw, footprint sx*sy, floor to height."""

    cx: float
    cy: float
    sx: float
    sy: float
    yaw: float = 0.0
    height: float = 0.6

    def surface(self, step: float) -> np.ndarray:
        """Sample the 4 side faces + top on a `step` grid, world frame."""
        hx, hy = self.sx / 2.0, self.sy / 2.0
        zs = np.arange(step / 2.0, self.height, step)
        pts: list[Any] = []
        for sign in (-1.0, 1.0):
            ys = np.arange(-hy, hy + step / 2.0, step)
            pts.append([(sign * hx, y, z) for y in ys for z in zs])
            xs = np.arange(-hx, hx + step / 2.0, step)
            pts.append([(x, sign * hy, z) for x in xs for z in zs])
        xs = np.arange(-hx, hx + step / 2.0, step)
        ys = np.arange(-hy, hy + step / 2.0, step)
        pts.append([(x, y, self.height) for x in xs for y in ys])
        local = np.concatenate([np.asarray(p) for p in pts if p])
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        world = local.copy()
        world[:, 0] = self.cx + c * local[:, 0] - s * local[:, 1]
        world[:, 1] = self.cy + s * local[:, 0] + c * local[:, 1]
        return np.asarray(world)

    def outline(self, z: float) -> np.ndarray:
        hx, hy = self.sx / 2.0, self.sy / 2.0
        corners = np.array([(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy), (-hx, -hy)])
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        out = np.zeros((5, 3))
        out[:, 0] = self.cx + c * corners[:, 0] - s * corners[:, 1]
        out[:, 1] = self.cy + s * corners[:, 0] + c * corners[:, 1]
        out[:, 2] = z
        return out

    def sdf2d(self, pts: np.ndarray) -> np.ndarray:
        """Exact signed distance of Nx2 points to the footprint rectangle."""
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        lx = c * (pts[:, 0] - self.cx) - s * (pts[:, 1] - self.cy)
        ly = s * (pts[:, 0] - self.cx) + c * (pts[:, 1] - self.cy)
        dx = np.abs(lx) - self.sx / 2.0
        dy = np.abs(ly) - self.sy / 2.0
        outside = np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0))
        return np.asarray(outside + np.minimum(np.maximum(dx, dy), 0.0))


@dataclass
class Scenario:
    name: str
    boxes: list[Box]
    goal: tuple[float, float]
    start: tuple[float, float, float] = (0.0, 0.0, 0.0)  # x, y, yaw
    # "clear": a route exists — find it, no veto. "refuse": the task is
    # impossible — veto (stop) is the only correct output. "safe": either is
    # fine, but never a non-vetoed interpenetrating path.
    expect: str = "clear"
    emb: Embodiment = GO2
    note: str = ""


SCENARIOS = [
    Scenario("empty", [], goal=(4.0, 0.0), note="baseline: straight, no deform"),
    Scenario(
        "box_on_path",
        [Box(2.0, 0.0, 0.5, 0.5)],
        goal=(4.0, 0.0),
        note="side choice, decisive",
    ),
    Scenario(
        "box_offset",
        [Box(2.0, 0.2, 0.5, 0.5)],
        goal=(4.0, 0.0),
        note="easy side is -y; take it",
    ),
    Scenario(
        "corridor",
        [Box(2.0, 0.60, 2.0, 0.3), Box(2.0, -0.60, 2.0, 0.3)],
        goal=(4.0, 0.0),
        note="0.9 m gap: thread center",
    ),
    Scenario(
        "door_gap",
        [
            Box(2.0, 1.28, 0.15, 1.6),
            Box(2.0, -1.28, 0.15, 1.6),
        ],
        goal=(4.0, 0.0),
        note="0.8 m opening in a wall",
    ),
    Scenario(
        "door_side",
        [Box(2.0, 1.28, 0.15, 1.6), Box(2.0, -1.28, 0.15, 1.6)],
        goal=(4.0, 0.0),
        start=(1.4, 1.15, 2.0),
        note="starts beside the door, 0.5 m off the wall, facing away",
    ),
    Scenario(
        "goal_by_wall",
        [Box(4.0, 0.55, 2.0, 0.3)],
        goal=(4.0, 0.0),
        note="terminal approach along a wall",
    ),
    Scenario(
        "slalom",
        [Box(1.5, 0.3, 0.5, 0.5), Box(3.0, -0.3, 0.5, 0.5)],
        goal=(4.5, 0.0),
        note="S-path, clean side handoff",
    ),
    Scenario(
        "corridor_side_goal",
        [Box(2.0, 0.60, 4.0, 0.3), Box(2.0, -0.60, 4.0, 0.3)],
        goal=(1.5, 1.5),
        start=(1.5, 0.0, 0.0),
        note="goal through the side wall: around the end or stop, never grind it",
    ),
    Scenario(
        "corridor_reverse",
        [Box(2.0, 0.60, 2.0, 0.3), Box(2.0, -0.60, 2.0, 0.3)],
        goal=(-0.5, 0.0),
        start=(1.5, 0.0, 0.0),
        note="goal behind, corridor too tight to turn: back out",
    ),
    Scenario(
        "u_trap",
        [
            Box(2.5, 0.0, 0.3, 1.9),  # dead-end wall
            Box(1.5, 0.95, 2.3, 0.3),  # arm
            Box(1.5, -0.95, 2.3, 0.3),  # arm
        ],
        goal=(4.0, 0.0),
        start=(1.5, 0.0, 0.0),
        note="in a U pocket facing the dead end: out and around, or stop",
    ),
    Scenario(
        "pickets",
        [
            Box(2.0, -0.5, 0.06, 0.06, height=0.5),
            Box(2.0, 0.0, 0.06, 0.06, height=0.5),
            Box(2.0, 0.5, 0.06, 0.06, height=0.5),
        ],
        goal=(4.0, 0.0),
        note="thin posts across the path: between or around, never through",
    ),
    Scenario(
        "offset_wall",
        [Box(2.0, 1.13, 0.15, 2.0), Box(2.0, -0.13, 0.15, 2.0)],
        goal=(4.0, 0.0),
        expect="safe",
        note="overlapping walls seal the middle: around the end, not through",
    ),
    Scenario(
        "narrow_gap",
        [Box(2.0, 1.13, 0.15, 2.0), Box(2.0, -1.13, 0.15, 2.0)],
        goal=(4.0, 0.0),
        expect="safe",
        note="0.26 m opening, body is 0.50: around or stop, never squeeze",
    ),
    Scenario(
        "zigzag_room",
        [
            # the room: 5x5, sealed
            Box(2.5, -0.075, 5.3, 0.15, height=0.8),
            Box(2.5, 5.075, 5.3, 0.15, height=0.8),
            Box(-0.075, 2.5, 0.15, 5.3, height=0.8),
            Box(5.075, 2.5, 0.15, 5.3, height=0.8),
            # two zigzag walls: an S through the room
            Box(1.75, 1.8, 3.5, 0.15, height=0.8),
            Box(3.25, 3.2, 3.5, 0.15, height=0.8),
        ],
        goal=(4.4, 4.4),
        start=(0.5, 0.6, -math.pi / 2),
        expect="safe",
        note="corner to corner through the zigzag, spawned facing the wrong way",
    ),
    Scenario(
        "boxed_in",
        [
            Box(0.875, 0.0, 0.15, 1.9, height=0.8),
            Box(-0.875, 0.0, 0.15, 1.9, height=0.8),
            Box(0.0, 0.875, 1.9, 0.15, height=0.8),
            Box(0.0, -0.875, 1.9, 0.15, height=0.8),
        ],
        goal=(3.0, 0.0),
        expect="refuse",
        note="fully enclosed: refusal is the only correct answer",
    ),
]


def recorded(path: str | FilePath, name: str | None = None, emb: Embodiment = GO2) -> Scenario:
    """A world recorded on the robot, from a ``recorded_world`` npz.

    The npz already carries the stability-filtered map's body-band footprint as
    merged rectangles, so this is the same oriented-box truth every other world
    speaks — only the expectation is withheld: reality is labeled "safe",
    never "clear" (see simulation/recorded_world.py for the extraction).
    """
    d = np.load(FilePath(path))
    h = float(d["band_height"])
    boxes = [
        Box(float(cx), float(cy), float(sx), float(sy), 0.0, h) for cx, cy, sx, sy in d["rects"]
    ]
    start = (float(d["start"][0]), float(d["start"][1]), float(d["start"][2]))
    return Scenario(
        name=name or str(d["name"]),
        boxes=boxes,
        goal=(float(d["goal"][0]), float(d["goal"][1])),
        start=start,
        expect="safe",
        emb=emb,
        note=f"recorded world, {len(boxes)} band rects",
    )


def straight_plan(start: tuple[float, float, float], goal: tuple[float, float]) -> Path:
    n = max(2, int(math.hypot(goal[0] - start[0], goal[1] - start[1]) / 0.1) + 1)
    xs = np.linspace(start[0], goal[0], n)
    ys = np.linspace(start[1], goal[1], n)
    return Path(
        frame_id="world",
        poses=[
            PoseStamped(frame_id="world", position=[float(x), float(y), 0.0])
            for x, y in zip(xs, ys, strict=False)
        ],
    )


@dataclass
class GenRules:
    """Placement rules for generated worlds."""

    bounds: tuple[float, float, float, float] = (-1.5, -3.0, 6.0, 3.0)
    n_boxes: tuple[int, int] = (8, 16)  # inclusive range
    # this many of them land ON the start->goal line (small lateral jitter):
    # the task must confront the robot, not decorate the margins
    on_path: tuple[int, int] = (2, 4)
    size: tuple[float, float] = (0.3, 1.6)
    # clutter sprinkled on top: small cubes and thin slats (pole / table-leg
    # class), allowed to touch the structural boxes — only spawn/goal
    # clearance applies, tight spots are the point
    sprinkle: tuple[int, int] = (8, 16)
    sprinkle_size: tuple[float, float] = (0.05, 0.35)
    slat_thickness: tuple[float, float] = (0.04, 0.08)
    slat_length: tuple[float, float] = (0.3, 1.2)
    # any two obstacles are either touching (< contact) — one wall — or at
    # least min_gap apart: no accidental slivers the rules did not intend
    min_gap: float = 0.9
    contact: float = 0.05
    spawn_clear: float = 0.6  # free disk around start and goal
    goal_dist: float = 3.5
    tries: int = 40


def bfs_path(
    boxes: list[Box],
    a: tuple[float, float],
    b: tuple[float, float],
    radius: float,
    bounds: tuple[float, float, float, float],
    cell: float = 0.1,
    pad: float = 2.5,
) -> np.ndarray | None:
    """Shortest grid route on the exact truth occupancy (body = disk of
    `radius`), or None. The grid extends `pad` beyond the placement bounds:
    space is open, a route around the whole clutter field counts."""
    x0, y0, x1, y1 = bounds
    x0, y0, x1, y1 = x0 - pad, y0 - pad, x1 + pad, y1 + pad
    gx = np.arange(x0, x1 + cell, cell)
    gy = np.arange(y0, y1 + cell, cell)
    X, Y = np.meshgrid(gx, gy, indexing="ij")
    P = np.column_stack([X.ravel(), Y.ravel()])
    occ = np.zeros(len(P), dtype=bool)
    for bx in boxes:
        occ |= bx.sdf2d(P) < radius
    grid = occ.reshape(len(gx), len(gy))

    def cell_of(p: tuple[float, float]) -> tuple[int, int]:
        return (
            int(np.clip(round((p[0] - x0) / cell), 0, len(gx) - 1)),
            int(np.clip(round((p[1] - y0) / cell), 0, len(gy) - 1)),
        )

    ai, bi = cell_of(a), cell_of(b)
    if grid[ai] or grid[bi]:
        return None
    parent = np.full((len(gx), len(gy), 2), -1, dtype=np.int32)
    parent[ai] = ai
    q = deque([ai])
    while q:
        i, j = q.popleft()
        if (i, j) == bi:
            path = [(i, j)]
            while (i, j) != ai:
                i, j = parent[i, j]
                path.append((i, j))
            cells = np.array(path[::-1], dtype=float)
            return np.column_stack([x0 + cells[:, 0] * cell, y0 + cells[:, 1] * cell])
        for ni, nj in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if (
                0 <= ni < len(gx)
                and 0 <= nj < len(gy)
                and parent[ni, nj, 0] < 0
                and not grid[ni, nj]
            ):
                parent[ni, nj] = (i, j)
                q.append((ni, nj))
    return None


def _route(
    boxes: list[Box],
    a: tuple[float, float],
    b: tuple[float, float],
    radius: float,
    bounds: tuple[float, float, float, float],
) -> bool:
    return bfs_path(boxes, a, b, radius, bounds) is not None


# Cache location and write policy. AUTORESEARCH_CACHE_DIR redirects both
# pickle caches (harnesses point workers/worktrees at one shared dir);
# AUTORESEARCH_CACHE_RO suppresses writes — the caches are whole-file
# read-modify-write pickles, so concurrent writers lose updates.
_CACHE_BASE = FilePath(os.environ.get("AUTORESEARCH_CACHE_DIR") or FilePath(__file__).parent)
_CACHE_RO = bool(os.environ.get("AUTORESEARCH_CACHE_RO"))


def _write_atomic(path: FilePath, payload: bytes) -> None:
    """Atomic replace: a crash mid-write can never leave a torn pickle."""
    tmp = path.with_suffix(path.suffix + f".tmp{os.getpid()}")
    tmp.write_bytes(payload)
    os.replace(tmp, path)


# The true body footprint, sampled densely enough that a 4 cm slat cannot
# slip between sample points (GO2 box 0.85 x 0.50, centered -0.01).
_SE2_CACHE = _CACHE_BASE / ".se2_cache.pkl"


def se2_search(
    fgx: np.ndarray,
    fgy: np.ndarray,
    sdf_grid: np.ndarray,
    bounds: tuple[float, float, float, float],
    start: tuple[float, float, float],
    goal: tuple[float, float],
    emb: Embodiment,
    margin: float,
    cell: float = 0.12,
    yaw_bins: int = 16,
) -> np.ndarray | None:
    """The SE(2) lattice search on a prebuilt fine SDF grid — shared by the
    gold oracle (box-exact grid) and any candidate that builds its grid from
    the cloud. Returns (N, 3) smoothed states or None."""
    fine = float(fgx[1] - fgx[0])
    x0, y0, x1, y1 = bounds
    offsets = emb.offsets()
    gx = np.arange(x0, x1 + cell, cell)
    gy = np.arange(y0, y1 + cell, cell)
    nx, ny = len(gx), len(gy)
    thetas = np.linspace(-math.pi, math.pi, yaw_bins, endpoint=False)

    def lookup(px: np.ndarray, py: np.ndarray) -> np.ndarray:
        i = np.clip(np.round((px - fgx[0]) / fine).astype(int), 0, len(fgx) - 1)
        j = np.clip(np.round((py - fgy[0]) / fine).astype(int), 0, len(fgy) - 1)
        return np.asarray(sdf_grid[i, j])

    X, Y = np.meshgrid(gx, gy, indexing="ij")
    clr = np.zeros((yaw_bins, nx, ny), dtype=np.float32)
    for bi, th in enumerate(thetas):
        c, s = math.cos(th), math.sin(th)
        clear = np.full(nx * ny, np.inf)
        for ox, oy in offsets:
            wx = X.ravel() + c * ox - s * oy
            wy = Y.ravel() + s * ox + c * oy
            clear = np.minimum(clear, lookup(wx, wy))
        clr[bi] = clear.reshape(nx, ny)
    free = clr > margin

    import heapq

    def cell_of(p: tuple[float, float]) -> tuple[int, int]:
        return (
            int(np.clip(round((p[0] - x0) / cell), 0, nx - 1)),
            int(np.clip(round((p[1] - y0) / cell), 0, ny - 1)),
        )

    sb = int(np.argmin(np.abs(np.angle(np.exp(1j * (thetas - start[2]))))))
    si, sj = cell_of(start[:2])
    gi, gj = cell_of(goal)
    result: np.ndarray | None = None
    if free[sb, si, sj]:
        # Gait-real costs: walking forward is cheapest, strafing ~1.8x,
        # backing up ~1.5x — the ideal turns to face long legs instead of
        # crabbing through the whole world, yet still backs out of pockets.
        # 16 directions (8-connected + knight steps, ~26.6 deg resolution).
        # Knight steps span 2 cells, so their midpoint cells are checked too —
        # the oracle must not commit the very thin-wall hop it exists to catch.
        moves = []
        for di in range(-2, 3):
            for dj in range(-2, 3):
                if (di, dj) == (0, 0) or math.gcd(abs(di), abs(dj)) == 2:
                    continue
                mids = []
                if max(abs(di), abs(dj)) == 2:
                    mids = [
                        (math.floor(di / 2.0), math.floor(dj / 2.0)),
                        (math.ceil(di / 2.0), math.ceil(dj / 2.0)),
                    ]
                moves.append((di, dj, math.hypot(di, dj) * cell, mids))

        def move_cost(base: float, di: int, dj: int, th: float) -> float:
            rel = math.atan2(dj, di) - th
            f, l = math.cos(rel), math.sin(rel)
            return base * (
                1.0 + (emb.strafe - 1.0) * abs(l) + ((emb.reverse - 1.0) if f < 0 else 0.0)
            )

        yaw_cost = emb.yaw_w * (2 * math.pi / yaw_bins)

        # Comfort: clearance under CLEAR_PREF is charged progressively (up
        # to ~2.5x at contact) — swing wide when it's free, thread tight
        # gaps only when the world demands it. Beyond CLEAR_PREF: no charge.
        pref = emb.comfort
        tight = 1.0 + 1.5 * np.clip((pref - clr) / pref, 0.0, 1.0)
        dist = np.full((yaw_bins, nx, ny), np.inf)
        prev = np.full((yaw_bins, nx, ny, 3), -1, dtype=np.int16)
        dist[sb, si, sj] = 0.0
        q: list[tuple[float, int, int, int]] = [(0.0, sb, si, sj)]
        goal_state = None
        while q:
            d, b_, i, j = heapq.heappop(q)
            if d > dist[b_, i, j]:
                continue
            if (i, j) == (gi, gj):
                goal_state = (b_, i, j)
                break
            for di, dj, base, mids in moves:
                ni, nj = i + di, j + dj
                if not (0 <= ni < nx and 0 <= nj < ny and free[b_, ni, nj]):
                    continue
                if any(not free[b_, i + mi, j + mj] for mi, mj in mids):
                    continue
                c_ = move_cost(base, di, dj, thetas[b_]) * float(tight[b_, ni, nj])
                if d + c_ < dist[b_, ni, nj]:
                    dist[b_, ni, nj] = d + c_
                    prev[b_, ni, nj] = (b_, i, j)
                    heapq.heappush(q, (d + c_, b_, ni, nj))
            for nb in ((b_ + 1) % yaw_bins, (b_ - 1) % yaw_bins):
                yc = yaw_cost * float(tight[nb, i, j])
                if free[nb, i, j] and d + yc < dist[nb, i, j]:
                    dist[nb, i, j] = d + yc
                    prev[nb, i, j] = (b_, i, j)
                    heapq.heappush(q, (d + yc, nb, i, j))
                # Blend edges: walk and turn in the same step, discounted —
                # without these the lattice can only express turn-THEN-walk,
                # so it rotated in place even in open space.
                for di, dj, base, mids in moves:
                    ni, nj = i + di, j + dj
                    if not (0 <= ni < nx and 0 <= nj < ny and free[nb, ni, nj]):
                        continue
                    if any(not free[nb, i + mi, j + mj] for mi, mj in mids):
                        continue
                    c_ = (move_cost(base, di, dj, thetas[b_]) + 0.5 * yaw_cost) * float(
                        tight[nb, ni, nj]
                    )
                    if d + c_ < dist[nb, ni, nj]:
                        dist[nb, ni, nj] = d + c_
                        prev[nb, ni, nj] = (b_, i, j)
                        heapq.heappush(q, (d + c_, nb, ni, nj))
        if goal_state is not None:
            states = [goal_state]
            while tuple(prev[states[-1]][:3]) != (-1, -1, -1) and states[-1] != (sb, si, sj):
                states.append(tuple(prev[states[-1]]))
            states.reverse()
            raw = np.array([(gx[i], gy[j], thetas[b_]) for b_, i, j in states])

            # Shortcut smoothing, validity-preserving: replace state runs by
            # straight SE(2) interpolations (yaw = shortest arc) whenever every
            # interpolated body pose clears the margin — the staircase is
            # lattice quantization, not the optimum.
            def pose_clear(x: float, y: float, th: float) -> float:
                c_, s_ = math.cos(th), math.sin(th)
                wx = x + c_ * offsets[:, 0] - s_ * offsets[:, 1]
                wy = y + s_ * offsets[:, 0] + c_ * offsets[:, 1]
                return float(np.min(lookup(wx, wy)))

            def seg_free(a: np.ndarray, b: np.ndarray, floor: float) -> bool:
                # A shortcut may never get closer to the world than the raw
                # detour it replaces (capped at the comfort preference) —
                # else smoothing re-cuts the corners the cost paid to avoid.
                dyaw = math.remainder(b[2] - a[2], 2 * math.pi)
                steps = max(
                    2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.06), int(abs(dyaw) / 0.15)
                )
                for t in np.linspace(0.0, 1.0, steps + 1):
                    if (
                        pose_clear(
                            a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * dyaw
                        )
                        <= floor
                    ):
                        return False
                return True

            raw_clear = np.array([pose_clear(x, y, th) for x, y, th in raw])
            keep = [0]
            while keep[-1] < len(raw) - 1:
                j = len(raw) - 1
                while j > keep[-1] + 1:
                    floor = max(
                        margin,
                        min(emb.comfort, float(np.min(raw_clear[keep[-1] : j + 1]))) - 0.02,
                    )
                    if seg_free(raw[keep[-1]], raw[j], floor):
                        break
                    j -= 1
                keep.append(j)
            result = raw[keep]

    return result


def se2_path(
    boxes: list[Box],
    start: tuple[float, float, float],
    goal: tuple[float, float],
    emb: Embodiment = GO2,
    margin: float | None = None,
    cell: float = 0.12,
    yaw_bins: int = 16,
    pad: float = 1.5,
) -> np.ndarray | None:
    """Brute-force maneuver with the REAL body: Dijkstra over (x, y, yaw).

    Box-exact truth SDF (judge privilege — candidates build theirs from the
    cloud and call se2_search). See se2_search for the search itself.
    Cached on (world, query) — a solve is a few seconds per world.
    """
    if margin is None:
        margin = emb.precision  # below control precision, clearance is fiction
    key = hashlib.sha256(
        repr(("v8-shared-search", boxes, start, goal, emb, margin, cell, yaw_bins, pad)).encode()
    ).hexdigest()
    memo: dict[str, np.ndarray | None] = {}
    if _SE2_CACHE.exists():
        try:
            memo = pickle.loads(_SE2_CACHE.read_bytes())
            if key in memo:
                return memo[key]
        except Exception:
            memo = {}

    xs = [start[0], goal[0]]
    ys = [start[1], goal[1]]
    for b in boxes:
        o = b.outline(0.0)
        xs.extend(o[:, 0])
        ys.extend(o[:, 1])
    x0, y0 = min(xs) - pad, min(ys) - pad
    x1, y1 = max(xs) + pad, max(ys) + pad

    fine = 0.05
    fgx = np.arange(x0 - 0.6, x1 + 0.6, fine)
    fgy = np.arange(y0 - 0.6, y1 + 0.6, fine)
    FX, FY = np.meshgrid(fgx, fgy, indexing="ij")
    P = np.column_stack([FX.ravel(), FY.ravel()])
    sdf = np.full(len(P), np.inf)
    for b in boxes:
        sdf = np.minimum(sdf, b.sdf2d(P))
    sdf_grid = sdf.reshape(len(fgx), len(fgy))

    result = se2_search(
        fgx, fgy, sdf_grid, (x0, y0, x1, y1), start, goal, emb, margin, cell, yaw_bins
    )
    memo[key] = result
    if not _CACHE_RO:
        try:
            _write_atomic(_SE2_CACHE, pickle.dumps(memo))
        except Exception:
            pass
    return result


def label(
    boxes: list[Box],
    start: tuple[float, float, float],
    goal: tuple[float, float],
    emb: Embodiment = GO2,
) -> str:
    """Expectation from ground truth, body-exact: a maneuver with margin
    above control precision proves clear; none even at zero proves refuse."""
    if se2_path(boxes, start, goal, emb, margin=emb.precision + 0.05) is not None:
        return "clear"
    if se2_path(boxes, start, goal, emb, margin=0.0) is None:
        return "refuse"
    return "safe"


def generate(seed: int, rules: GenRules | None = None, emb: Embodiment = GO2) -> Scenario:
    """Deterministic world for a seed, rule-checked and truth-labeled.

    Difficulty is embodiment-relative: gap rules, spawn/goal disks and the
    passability guarantee all derive from the body under test, so "tight"
    means the same thing for every robot."""
    rules = rules or GenRules()
    # Derived rules: no ambiguous slivers (gaps are passable-with-margin or
    # sealed), spawn/goal free for the body at any orientation.
    min_gap = emb.width + 2.0 * emb.precision + 0.15
    spawn_clear = emb.half_diag + max(0.15, emb.comfort / 2.0)
    r_con = emb.half_diag + emb.precision
    rng = np.random.default_rng(seed)
    x0, y0, x1, y1 = rules.bounds
    start = (0.0, 0.0, float(rng.uniform(-math.pi, math.pi)))
    while True:
        goal = (float(rng.uniform(x0 + 0.5, x1 - 0.5)), float(rng.uniform(y0 + 0.5, y1 - 0.5)))
        if math.hypot(goal[0] - start[0], goal[1] - start[1]) >= rules.goal_dist:
            break
    keep_out = np.array([start[:2], goal])
    dist = math.hypot(goal[0] - start[0], goal[1] - start[1])
    u = ((goal[0] - start[0]) / dist, (goal[1] - start[1]) / dist)
    boxes: list[Box] = []
    n_total = int(rng.integers(rules.n_boxes[0], rules.n_boxes[1] + 1))
    n_on = int(rng.integers(rules.on_path[0], rules.on_path[1] + 1))
    for i in range(n_total):
        for _ in range(rules.tries):
            if i < n_on:
                t = float(rng.uniform(0.25, 0.85)) * dist
                lat = float(rng.uniform(-0.4, 0.4))
                cx = start[0] + u[0] * t - u[1] * lat
                cy = start[1] + u[1] * t + u[0] * lat
            else:
                cx, cy = float(rng.uniform(x0, x1)), float(rng.uniform(y0, y1))
            b = Box(
                cx=cx,
                cy=cy,
                sx=float(rng.uniform(*rules.size)),
                sy=float(rng.uniform(*rules.size)),
                yaw=float(rng.uniform(0.0, math.pi)),
            )
            if float(np.min(b.sdf2d(keep_out))) < spawn_clear:
                continue
            gaps = [float(np.min(b.sdf2d(o.outline(0.0)[:, :2]))) for o in boxes]
            if any(rules.contact < g < min_gap for g in gaps):
                continue  # sliver between obstacles: not what the rules meant
            boxes.append(b)
            break
    for _ in range(int(rng.integers(rules.sprinkle[0], rules.sprinkle[1] + 1))):
        for _ in range(rules.tries):
            if rng.random() < 0.5:  # thin slat
                sx = float(rng.uniform(*rules.slat_thickness))
                sy = float(rng.uniform(*rules.slat_length))
            else:  # small cube-ish
                sx = float(rng.uniform(*rules.sprinkle_size))
                sy = float(rng.uniform(*rules.sprinkle_size))
            b = Box(
                cx=float(rng.uniform(x0, x1)),
                cy=float(rng.uniform(y0, y1)),
                sx=sx,
                sy=sy,
                yaw=float(rng.uniform(0.0, math.pi)),
                height=float(rng.uniform(0.2, 0.5)),
            )
            if float(np.min(b.sdf2d(keep_out))) < spawn_clear:
                continue
            boxes.append(b)
            break
    # Space is open, so nearly every world is passable by going wide; the
    # rare exception is boxes ringing the start or goal. Peel until the
    # conservative disk provably routes — generated tasks are always solvable.
    while boxes and not _route(boxes, start[:2], goal, r_con, rules.bounds):
        for i in range(len(boxes)):
            if _route(boxes[:i] + boxes[i + 1 :], start[:2], goal, r_con, rules.bounds):
                boxes.pop(i)
                break
        else:
            boxes.pop(int(np.argmax([b.sx * b.sy for b in boxes])))
    exp = label(boxes, start, goal, emb)
    return Scenario(
        name=f"gen{seed:03d}",
        boxes=boxes,
        goal=goal,
        start=start,
        expect=exp,
        emb=emb,
        note=f"seeded world, truth-labeled {exp}" + (f" [{emb.tag}]" if emb.tag != "go2" else ""),
    )


# Generated-set defaults: the suite and the CLI share these.
GEN_COUNT = 40
GEN_SEED = 0


def _gen_cache(count: int, seed: int) -> FilePath:
    """One cache file per battery: a 40-world set and a 20-world OOD set
    must not clobber each other (the key check alone made them alternate)."""
    return _CACHE_BASE / f".gen_cache_{count}_{seed}.pkl"


def _gen_hash(count: int, seed: int, rules: GenRules, emb: Any) -> str:
    """Fingerprint of the generator: source + parameters. Any change stales
    the cache, so cached worlds are always exactly what this code produces."""
    src = "".join(inspect.getsource(o) for o in (Box, GenRules, generate, label, _route))
    return hashlib.sha256((src + repr((count, seed, rules, emb))).encode()).hexdigest()[:16]


def generated(
    count: int = GEN_COUNT,
    seed: int = GEN_SEED,
    rules: GenRules | None = None,
    emb: Embodiment | None = GO2,
) -> list[Scenario]:
    """The generated battery, cached under the generator fingerprint.
    emb=None mixes the roster: each seed gets EMBODIMENTS[seed % len]."""
    rules = rules or GenRules()
    key = _gen_hash(count, seed, rules, emb if emb is not None else tuple(EMBODIMENTS))
    cache = _gen_cache(count, seed)
    if not cache.exists() and (legacy := _CACHE_BASE / ".gen_cache.pkl").exists():
        cache = legacy  # pre-split cache; read-only fallback, new writes split
    if cache.exists():
        try:
            payload = pickle.loads(cache.read_bytes())
            if payload.get("key") == key:
                return list(payload["scenarios"])
        except Exception:
            pass  # stale/corrupt cache: regenerate below
    scenarios = []
    for i, s in enumerate(range(seed, seed + count)):
        print(f"\rgenerating worlds (SE2-labeled): {i + 1}/{count}", end="", file=sys.stderr)
        roster = list(EMBODIMENTS.values())
        scenarios.append(generate(s, rules, emb if emb is not None else roster[s % len(roster)]))
    print(file=sys.stderr)
    if not _CACHE_RO:
        _write_atomic(_gen_cache(count, seed), pickle.dumps({"key": key, "scenarios": scenarios}))
    return scenarios
