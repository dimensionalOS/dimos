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

"""SE(2) lattice search over an `SdfGrid`; the spec the rust planner is checked against."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import cKDTree

from dimos.navigation.embodiment.base import Embodiment, box_offsets

# Edges are priced on the follower's own speed law; one copy of it.
from dimos.navigation.local_planner.profile import governor_speed

# (N, 3) rows of (x, y, yaw), the rust boundary's shape too.
States = NDArray[np.float64]
Pose2 = tuple[float, float, float]  # x, y, yaw


# Lattice pitch: VOXEL is the map's; grid corners snap to PERIOD so a new
# obstacle can add rows at the edge but never move a sample.
VOXEL = 0.08
FINE = 0.04  # VOXEL / 2
CELL = 0.12  # 3 * FINE
PERIOD = 0.24  # 2 * CELL == 3 * VOXEL == 6 * FINE

# How much cheaper a challenger must be to displace the published route, in
# `path_cost` units; handed to the rust planner unchanged.
COMMIT_MARGIN = 3.0


@dataclass(frozen=True)
class SdfGrid:
    """Distance to the nearest obstacle sampled on the fine lattice, indexed [ix, iy]."""

    x0: float
    y0: float
    pitch: float
    d: NDArray[np.float64]

    @classmethod
    def from_obstacles(
        cls,
        bounds: tuple[float, float, float, float],
        obstacles: NDArray[np.float64],
        pitch: float = FINE,
    ) -> SdfGrid:
        """Distance to the nearest of (N, 2) obstacle xy, sampled over `bounds` (x0, y0, x1, y1)."""
        x0, y0, x1, y1 = bounds
        xs, ys = np.arange(x0, x1, pitch), np.arange(y0, y1, pitch)
        if not len(obstacles):
            return cls(x0, y0, pitch, np.full((len(xs), len(ys)), np.inf))
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        d, _ = cKDTree(obstacles).query(np.column_stack([X.ravel(), Y.ravel()]))
        return cls(x0, y0, pitch, d.reshape(len(xs), len(ys)))

    def ix(self, x: NDArray[np.floating[Any]] | float) -> NDArray[np.intp]:
        return np.clip(np.round((x - self.x0) / self.pitch).astype(np.intp), 0, self.d.shape[0] - 1)

    def iy(self, y: NDArray[np.floating[Any]] | float) -> NDArray[np.intp]:
        return np.clip(np.round((y - self.y0) / self.pitch).astype(np.intp), 0, self.d.shape[1] - 1)

    def lookup(self, x: NDArray[np.float64], y: NDArray[np.float64]) -> NDArray[np.float64]:
        """Nearest-sample clearance at each (x, y), metres."""
        return np.asarray(self.d[self.ix(x), self.iy(y)])


def anchor(v: float, period: float = PERIOD) -> float:
    """Snap a grid corner down onto the frame's own absolute lattice."""
    return math.floor(v / period) * period


# Pricing pitch along the route's arc, not its vertices: a densified incumbent
# and a sparse fresh route must price identically.
COST_STEP = FINE


def path_cost(grid: SdfGrid, states: States, emb: Embodiment, step: float = COST_STEP) -> float:
    """Route cost in open-space metres: the search's edge pricing, integrated along the arc.

    Yaw is half price while translating (blend edge), full in place (turn edge).
    """
    s = np.asarray(states, dtype=float).reshape(-1, 3)
    if len(s) < 2:
        return 0.0
    off = box_offsets(emb.box(None))

    def tight(
        px: NDArray[np.float64], py: NDArray[np.float64], th: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        c_, s_ = np.cos(th)[:, None], np.sin(th)[:, None]
        wx = px[:, None] + c_ * off[None, :, 0] - s_ * off[None, :, 1]
        wy = py[:, None] + s_ * off[None, :, 0] + c_ * off[None, :, 1]
        return np.asarray(emb.max_speed / governor_speed(np.min(grid.lookup(wx, wy), axis=1), emb))

    d = s[1:] - s[:-1]
    span = np.hypot(d[:, 0], d[:, 1])
    dyaw = np.remainder(d[:, 2] + math.pi, 2.0 * math.pi) - math.pi
    moving = span > 1e-9
    total = 0.0

    # Rotations in place carry no arc, so they are priced outside the integral.
    spin = np.flatnonzero(~moving)
    if len(spin):
        th = s[spin, 2] + 0.5 * dyaw[spin]
        total += float(np.sum(emb.yaw_w * np.abs(dyaw[spin]) * tight(s[spin, 0], s[spin, 1], th)))

    mv = np.flatnonzero(moving)
    if not len(mv):
        return total
    # Sub-steps split the total length evenly, so adding a vertex moves no sample.
    arcs = np.concatenate([[0.0], np.cumsum(np.where(moving, span, 0.0))])
    length = float(arcs[-1])
    edges = np.linspace(0.0, length, max(1, math.ceil(length / step)) + 1)
    w = np.diff(edges)
    mid = 0.5 * (edges[:-1] + edges[1:])
    k = mv[np.clip(np.searchsorted(arcs[mv], mid, side="right") - 1, 0, len(mv) - 1)]
    t = np.clip((mid - arcs[k]) / span[k], 0.0, 1.0)

    th = s[k, 2] + t * dyaw[k]
    rel = np.arctan2(d[k, 1], d[k, 0]) - th
    gait = 1.0 + (emb.strafe - 1.0) * np.abs(np.sin(rel))
    gait = gait + np.where(np.cos(rel) < 0.0, emb.reverse - 1.0, 0.0)
    turn = 0.5 * emb.yaw_w * np.abs(dyaw[k]) / span[k]
    px, py = s[k, 0] + t * d[k, 0], s[k, 1] + t * d[k, 1]
    return total + float(np.sum((gait + turn) * w * tight(px, py, th)))


def trim_to_pose(states: States, pose: Pose2) -> States:
    """The route from its waypoint nearest `pose` on. The pose is not spliced in: a
    head that turns that hard would fail re-validation; `priced` charges the walk."""
    s = np.asarray(states, dtype=float).reshape(-1, 3)
    if not len(s):
        return np.array([[float(pose[0]), float(pose[1]), float(pose[2])]])
    i = int(np.argmin(np.linalg.norm(s[:, :2] - np.array(pose[:2]), axis=1)))
    return np.asarray(s[i:])


def se2_search(
    grid: SdfGrid,
    bounds: tuple[float, float, float, float],
    start: Pose2,
    goal: tuple[float, float],
    emb: Embodiment,
    margin: float,
    cell: float = CELL,
    yaw_bins: int = 16,
    incumbent: States | None = None,
    commit_margin: float = COMMIT_MARGIN,
) -> States | None:
    """SE(2) lattice search on a fine SDF grid; (N, 3) smoothed states or None.

    Edges are tested against the swept box for their drift angle (union when
    unmeasured or turning in place); `incumbent` is kept unless beaten by `commit_margin`.
    """
    fine = grid.pitch
    x0, y0, x1, y1 = bounds
    gx = np.arange(x0, x1 + cell, cell)
    gy = np.arange(y0, y1 + cell, cell)
    nx, ny = len(gx), len(gy)
    thetas = np.linspace(-math.pi, math.pi, yaw_bins, endpoint=False)

    # Boxes interned by geometry: one clearance stack per distinct box, not per edge.
    fp_ids: dict[tuple[float, ...], int] = {}
    fp_offsets: list[NDArray[np.float64]] = []

    def footprint(box: tuple[float, float, float, float]) -> int:
        key = tuple(round(v, 9) for v in box)
        if key not in fp_ids:
            fp_ids[key] = len(fp_offsets)
            fp_offsets.append(box_offsets(box))
        return fp_ids[key]

    UNION = footprint(emb.box(None))  # id 0: the fallback shape

    # 8-connected plus knight steps; knight midpoints are checked so a thin wall cannot be hopped.
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
            moves.append((di, dj, math.hypot(di, dj) * cell, mids, math.atan2(dj, di)))

    # Per (yaw bin, move): the swept box, and the half-width a blend edge's
    # curvature adds (`arc_inflate` is per rad/m, the edge length converts it).
    yaw_step = 2.0 * math.pi / yaw_bins
    fp_move = [[footprint(emb.box(head - th)) for _, _, _, _, head in moves] for th in thetas]
    arc_pad = [0.5 * emb.arc_inflate * yaw_step / base for _, _, base, _, _ in moves]

    # Both grids share the anchored corner and cell is a whole number of fine
    # samples, so every footprint offset is an integer shift of the field.
    ci, cj = grid.ix(gx), grid.iy(gy)
    nfx, nfy = grid.d.shape
    clr = np.full((yaw_bins, len(fp_offsets), nx, ny), -np.inf, dtype=np.float32)
    for bi, th in enumerate(thetas):
        c, s = math.cos(th), math.sin(th)
        for fp in {UNION, *fp_move[bi]}:
            shifts = {
                (round((c * ox - s * oy) / fine), round((s * ox + c * oy) / fine))
                for ox, oy in fp_offsets[fp].tolist()
            }
            clear = np.full((nx, ny), np.inf)
            for di_, dj_ in sorted(shifts):
                ii = np.clip(ci + di_, 0, nfx - 1)
                jj = np.clip(cj + dj_, 0, nfy - 1)
                np.minimum(clear, grid.d[np.ix_(ii, jj)], out=clear)
            clr[bi, fp] = clear
    free = clr > margin

    def pose_clear(x: float, y: float, th: float, fp: int = UNION) -> float:
        """Body clearance at an exact pose, no lattice snap."""
        off = fp_offsets[fp]
        c_, s_ = math.cos(th), math.sin(th)
        wx = x + c_ * off[:, 0] - s_ * off[:, 1]
        wy = y + s_ * off[:, 0] + c_ * off[:, 1]
        return float(np.min(grid.lookup(wx, wy)))

    def cell_of(p: tuple[float, float]) -> tuple[int, int]:
        return (
            int(np.clip(round((p[0] - x0) / cell), 0, nx - 1)),
            int(np.clip(round((p[1] - y0) / cell), 0, ny - 1)),
        )

    gi, gj = cell_of(goal)

    # One long edge judged as the lattice judges its own: per-drift box plus
    # curvature pad. Shared by the smoother and the incumbent's re-validation.
    def seg_free(a: NDArray[np.float64], b: NDArray[np.float64], floor: float) -> bool:
        dyaw = math.remainder(b[2] - a[2], 2 * math.pi)
        dx, dy = b[0] - a[0], b[1] - a[1]
        span = math.hypot(dx, dy)
        head = math.atan2(dy, dx) if span > 1e-9 else None
        pad = 0.5 * emb.arc_inflate * abs(dyaw) / span if span > 1e-9 else 0.0
        steps = max(2, int(span / 0.06), int(abs(dyaw) / 0.15))
        for t in np.linspace(0.0, 1.0, steps + 1):
            th = a[2] + t * dyaw
            fp = UNION if head is None else footprint(emb.box(head - th))
            if pose_clear(a[0] + t * dx, a[1] + t * dy, th, fp) <= floor + pad:
                return False
        return True

    # The seed is judged at the true pose with the static body, so a replan
    # from this planner's own route never refuses; the cell still names the node.
    STAND = footprint(emb.stand_box())

    def solve(seed: tuple[float, float, float]) -> NDArray[np.float64] | None:
        """The search from one seed pose to the goal; `committed` reuses it from the route's end."""
        if pose_clear(*seed, STAND) <= margin:
            return None
        sb = int(np.argmin(np.abs(np.angle(np.exp(1j * (thetas - seed[2]))))))
        si, sj = cell_of((seed[0], seed[1]))

        def move_cost(base: float, head: float, th: float) -> float:
            rel = head - th
            f, l = math.cos(rel), math.sin(rel)
            return base * (
                1.0 + (emb.strafe - 1.0) * abs(l) + ((emb.reverse - 1.0) if f < 0 else 0.0)
            )

        yaw_cost = emb.yaw_w * yaw_step

        # A metre at clearance c costs max_speed/governor_speed(c) open-space
        # metres (time on the follower's law); union clearance so edges compare.
        tight = emb.max_speed / governor_speed(clr[:, UNION], emb)
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
            fps = fp_move[b_]
            for m, (di, dj, base, mids, head) in enumerate(moves):
                ni, nj = i + di, j + dj
                fp = fps[m]
                if not (0 <= ni < nx and 0 <= nj < ny and free[b_, fp, ni, nj]):
                    continue
                if any(not free[b_, fp, i + mi, j + mj] for mi, mj in mids):
                    continue
                c_ = move_cost(base, head, thetas[b_]) * float(tight[b_, ni, nj])
                if d + c_ < dist[b_, ni, nj]:
                    dist[b_, ni, nj] = d + c_
                    prev[b_, ni, nj] = (b_, i, j)
                    heapq.heappush(q, (d + c_, b_, ni, nj))
            for nb in ((b_ + 1) % yaw_bins, (b_ - 1) % yaw_bins):
                yc = yaw_cost * float(tight[nb, i, j])
                # A turn in place has no drift row: the union is its shape.
                if free[nb, UNION, i, j] and d + yc < dist[nb, i, j]:
                    dist[nb, i, j] = d + yc
                    prev[nb, i, j] = (b_, i, j)
                    heapq.heappush(q, (d + yc, nb, i, j))
                # Blend edges (walk and turn in one step), judged at the arrival yaw
                # plus the curvature pad; without them the lattice only turns-then-walks.
                nfps = fp_move[nb]
                for m, (di, dj, base, mids, head) in enumerate(moves):
                    ni, nj = i + di, j + dj
                    fp, pad = nfps[m], arc_pad[m]
                    if not (0 <= ni < nx and 0 <= nj < ny and clr[nb, fp, ni, nj] > margin + pad):
                        continue
                    if any(clr[nb, fp, i + mi, j + mj] <= margin + pad for mi, mj in mids):
                        continue
                    c_ = (move_cost(base, head, thetas[b_]) + 0.5 * yaw_cost) * float(
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

            # Shortcut smoothing. A shortcut may not come closer to the world than
            # the raw detour it replaces (capped at comfort), or it re-cuts corners.
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
            return np.asarray(raw[keep])
        return None

    result = solve(start)
    if incumbent is None:
        return result

    def committed() -> NDArray[np.float64] | None:
        """The published route trimmed to here and carried to the goal, or None
        when this map no longer lets the body walk it (no debounce: map noise is perception's)."""
        route = trim_to_pose(np.asarray(incumbent, dtype=float).reshape(-1, 3), start)
        if len(route) < 2:
            return None
        end = route[-1]
        # The carrot advances between replans: extend by the chord when clear,
        # else by a search from the far end. A goal that jumped is the caller's to drop.
        if cell_of((float(end[0]), float(end[1]))) != (gi, gj):
            tgt = np.array([goal[0], goal[1], end[2]])
            if seg_free(end, tgt, margin):
                route = np.vstack([route, tgt])
            else:
                tail = solve((float(end[0]), float(end[1]), float(end[2])))
                if tail is None:
                    return None
                route = np.vstack([route, tail])
        # The published route is re-validated collision-free, not at the planning
        # margin: a corridor nibbled a few cm is not a reason for a new route.
        if any(not seg_free(a, b, 0.0) for a, b in itertools.pairwise(route)):
            return None
        return route

    route = committed()
    if route is None:
        return result
    if result is None:
        return route

    def priced(p: NDArray[np.float64]) -> NDArray[np.float64]:
        """Both routes priced from the true pose: the fresh one opens at its snapped cell."""
        here = np.array([[start[0], start[1], start[2]]])
        return p if np.allclose(p[0], here[0]) else np.vstack([here, p])

    fresh = path_cost(grid, priced(result), emb)
    held = path_cost(grid, priced(route), emb)
    return result if fresh < held - commit_margin else route
