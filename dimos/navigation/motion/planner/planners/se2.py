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

"""The SE(2) domain the local planner is written in.

`se2_search` is the lattice search itself, over an `SdfGrid`, with the pricing
(`path_cost`) and commitment (`trim_to_pose`) rules the rust planner is checked
against.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import cKDTree

# The search prices an edge by the time the FOLLOWER's governor will take over
# it, so it reads that law from the one place it is defined rather than keeping
# a second copy. Same direction adapter/planner.py already imports in (the
# stamps it encodes are the same curve); profile.py depends on nothing here, so
# this stays a leaf.
from dimos.navigation.motion.control.profile import governor_speed
from dimos.navigation.motion.embodiment.base import Embodiment, box_offsets

# (N, 3) rows of (x, y, yaw): the state the search speaks, and the rust boundary's.
States = NDArray[np.float64]
Pose2 = tuple[float, float, float]  # x, y, yaw


# Lattice pitch. VOXEL is a config constant of
# the deployment (the map's voxel size, never sniffed from data); FINE is half
# of it, so every voxel centre lands exactly on a fine sample; CELL is 3 fine
# samples. PERIOD is the pitch at which all three are commensurate -- 2 cells,
# 3 voxels, 6 fine samples -- and every grid corner is snapped DOWN onto
# multiples of it in the scenario's own frame. That is what makes sample
# positions absolute: an obstacle appearing or vanishing can add whole rows at
# the edge, but it can never move a sample position, so a lidar return metres
# behind the robot cannot re-sample the question the search is answering.
VOXEL = 0.08
FINE = 0.04  # VOXEL / 2
CELL = 0.12  # 3 * FINE
PERIOD = 0.24  # 2 * CELL == 3 * VOXEL == 6 * FINE

# How much cheaper a challenger route has to be before the planner abandons the
# one it already published, in open-space metres (`path_cost`'s unit). Measured:
# it covers every way a route's price can move while the world stands still
# (the seed snap and the incumbent's re-densification), plus headroom. One copy:
# the rust candidate is handed this value across the extension boundary.
COMMIT_MARGIN = 1.50


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


# Pitch at which a route is PRICED, along the route's own ARC. Not along its
# vertices, and that is the whole point: an incumbent comes back at path
# resolution while a fresh answer is a handful of smoothed vertices, and the two
# have to be weighed on one scale. Densifying a polyline adds points that lie on
# it, so it leaves the curve, its arc length and its yaw-by-arc untouched --
# sample by arc and the two representations price IDENTICALLY, rather than to
# within a quadrature error sitting next to a 0.1 m threshold.
COST_STEP = FINE


def path_cost(grid: SdfGrid, states: States, emb: Embodiment, step: float = COST_STEP) -> float:
    """What this route costs on the follower's own clock, in open-space metres.

    The pricing the search puts on its own edges, read along a continuous
    curve instead of along a lattice: a metre of gait-weighted travel, charged
    `max_speed/governor(clearance)` for the time it will take, plus the yaw the
    route commands -- half price while translating, as a blend edge pays it,
    full price for a rotation in place, as a turn edge does. Clearance is read
    on the UNION, again as the search reads it: a preference has to be
    comparable across routes, so it may not shift with an edge's own drift row.
    """
    s = np.asarray(states, dtype=float).reshape(-1, 3)
    if len(s) < 2:
        return 0.0
    off = box_offsets(emb.box(None))

    def tight(
        px: NDArray[np.float64], py: NDArray[np.float64], th: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """max_speed/governor(clearance): what a metre here costs in metres."""
        c_, s_ = np.cos(th)[:, None], np.sin(th)[:, None]
        wx = px[:, None] + c_ * off[None, :, 0] - s_ * off[None, :, 1]
        wy = py[:, None] + s_ * off[None, :, 0] + c_ * off[None, :, 1]
        return np.asarray(emb.max_speed / governor_speed(np.min(grid.lookup(wx, wy), axis=1), emb))

    d = s[1:] - s[:-1]
    span = np.hypot(d[:, 0], d[:, 1])
    dyaw = np.remainder(d[:, 2] + math.pi, 2.0 * math.pi) - math.pi
    moving = span > 1e-9
    total = 0.0

    # Rotation in place: no direction of travel, no drift row, full yaw price —
    # the turn edge's own charge. It carries no arc, so it is priced here rather
    # than in the integral below, which is parameterised by arc alone.
    spin = np.flatnonzero(~moving)
    if len(spin):
        th = s[spin, 2] + 0.5 * dyaw[spin]
        total += float(np.sum(emb.yaw_w * np.abs(dyaw[spin]) * tight(s[spin, 0], s[spin, 1], th)))

    mv = np.flatnonzero(moving)
    if not len(mv):
        return total
    # Arc at each vertex, rotations contributing nothing to it. Sub-steps split
    # the whole route evenly, so they are a function of its total length and of
    # nothing else — a vertex added anywhere on the curve moves no sample.
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
    # Turning while translating is a blend edge, and pays half the yaw price.
    turn = 0.5 * emb.yaw_w * np.abs(dyaw[k]) / span[k]
    px, py = s[k, 0] + t * d[k, 0], s[k, 1] + t * d[k, 1]
    return total + float(np.sum((gait + turn) * w * tight(px, py, th)))


def trim_to_pose(states: States, pose: Pose2) -> States:
    """Head-trim a published route to where the robot is now: the remainder from
    the nearest published waypoint on, exactly as the global route is trimmed to
    the robot on every republish. That remainder IS the commitment.

    The true pose does NOT replace the head. Splicing it in would hand the whole
    difference between the robot's yaw and the route's to one ~0.1 m segment,
    and a segment that turns that hard over that little asks for `arc_inflate`
    room the corridor does not have — the route would then fail its own
    re-validation for a reason that is about the splice and not about the world.
    The fresh answer does not do this either: it opens at the lattice pose its
    seed snapped to, up to half a cell diagonal from the robot. Both routes are
    PRICED from the true pose, which is where that walk is accounted for.
    """
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
    """The SE(2) lattice search on a prebuilt fine SDF grid. Returns (N, 3)
    smoothed states or None.

    Feasibility is motion-conditioned: an edge is tested against the swept box
    the embodiment needs for THAT edge's drift angle (move direction minus body
    yaw), widened by the gait's turning splay when the edge also rotates. The
    all-gait union stays the fallback -- for embodiments with no measured
    envelope, and for the turn-in-place edges, which are real motion sweeping
    the full shape.

    The seed is judged at the TRUE start pose rather than at the cell it snaps
    to: a pose the robot actually occupies may always be departed. It is judged
    STANDING -- the static body, the intersection of the envelope's rows -- not
    on the union of the swept boxes it is not moving through.

    The route already published, if there is one, is an INPUT. The lattice is
    quantized, so a replan from a pose a few millimetres along is a slightly
    different query, and where two routes near-tie the argmin flips for reasons
    that are about the quantization and not about the world. So: trim the
    incumbent to the current pose, re-validate it on THIS map, extend it to the
    goal if the goal moved, price both on the same clock, and switch only if the
    challenger wins by more than `commit_margin`. `incumbent=None` is
    bit-identical to a planner that never heard of any of this.
    """
    fine = grid.pitch
    x0, y0, x1, y1 = bounds
    gx = np.arange(x0, x1 + cell, cell)
    gy = np.arange(y0, y1 + cell, cell)
    nx, ny = len(gx), len(gy)
    thetas = np.linspace(-math.pi, math.pi, yaw_bins, endpoint=False)

    # Footprint table. The envelope answers every drift angle with one of a
    # handful of distinct boxes, so they are interned by geometry: the
    # clearance stack is built once per box, not once per edge.
    fp_ids: dict[tuple[float, ...], int] = {}
    fp_offsets: list[NDArray[np.float64]] = []

    def footprint(box: tuple[float, float, float, float]) -> int:
        key = tuple(round(v, 9) for v in box)
        if key not in fp_ids:
            fp_ids[key] = len(fp_offsets)
            fp_offsets.append(box_offsets(box))
        return fp_ids[key]

    UNION = footprint(emb.box(None))  # id 0: the veto shape and every fallback

    # 16 directions (8-connected + knight steps, ~26.6 deg resolution).
    # Knight steps span 2 cells, so their midpoint cells are checked too —
    # the search must not commit the very thin-wall hop it exists to catch.
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

    # Per (yaw bin, move): which swept box, and how much extra half-width a
    # blend edge's curvature costs. `arc_inflate` is per rad-per-metre, so the
    # edge's own length converts it; half of the extra width is what a
    # clearance test against the box centre-line has to give up.
    yaw_step = 2.0 * math.pi / yaw_bins
    fp_move = [[footprint(emb.box(head - th)) for _, _, _, _, head in moves] for th in thetas]
    arc_pad = [0.5 * emb.arc_inflate * yaw_step / base for _, _, base, _, _ in moves]

    # Cell centres as fine-grid indices. Both grids hang off the same anchored
    # corner and cell is a whole number of fine samples, so a footprint offset
    # is an INTEGER shift of the field — the whole precompute is gathers on
    # shifted views, and the envelope's extra footprints cost a gather each
    # rather than a fresh round of coordinate arithmetic.
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
        """Clearance of the body at an exact pose — no lattice snap anywhere."""
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

    # One long edge, judged the way the lattice judges its own: against the
    # drift row the body needs for THAT edge, widened by its own curvature.
    # Judging it against the union instead would forbid every shortcut through
    # a gap the lattice just proved the body walks down nose-first. Both the
    # smoother and the incumbent's re-validation ask exactly this question, so
    # there is one copy of it.
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

    # A pose the robot actually occupies may always be departed. The seed's
    # feasibility is therefore read at the TRUE start pose, not at the cell it
    # snaps to (the snap moves the body up to half a cell diagonal, ~85 mm);
    # the cell still NAMES the seed node. Standing has no direction of travel,
    # and the shape it occupies is the STATIC BODY -- the intersection of the
    # envelope's rows, nested in each of them -- so a pose any edge was cleared
    # by clears the witness too, and a replan from this planner's own route can
    # never refuse (on the union it did: a gap only a drift row fits refuses
    # forever once the robot is inside it). A start genuinely inside an
    # obstacle still reads negative and still refuses. Interned here, not with
    # the moving rows: it is read at one exact pose and never wants a plane.
    STAND = footprint(emb.stand_box())

    def solve(seed: tuple[float, float, float]) -> NDArray[np.float64] | None:
        """The search proper, from one seed pose to the fixed goal.

        Named rather than inlined because the incumbent's extension asks the
        very same question from the route's far end, and a sub-query the
        planner answers with a second copy of itself is a second planner.
        """
        if pose_clear(*seed, STAND) <= margin:
            return None
        sb = int(np.argmin(np.abs(np.angle(np.exp(1j * (thetas - seed[2]))))))
        si, sj = cell_of((seed[0], seed[1]))

        # Gait-real costs: walking forward is cheapest, strafing ~1.8x,
        # backing up ~1.5x — the ideal turns to face long legs instead of
        # crabbing through the whole world, yet still backs out of pockets.
        def move_cost(base: float, head: float, th: float) -> float:
            rel = head - th
            f, l = math.cos(rel), math.sin(rel)
            return base * (
                1.0 + (emb.strafe - 1.0) * abs(l) + ((emb.reverse - 1.0) if f < 0 else 0.0)
            )

        yaw_cost = emb.yaw_w * yaw_step

        # Tightness prices TIME, on the follower's own committed speed law: a
        # metre at clearance c takes 1/governor_speed(c) seconds, so it costs
        # max_speed/governor_speed(c) metres of open-space walking. Planner and
        # follower then optimize the same clock instead of a tunable comfort
        # ramp, and the multiplier composes with the gait factors, which are
        # ratios of the same kind. It needs no artificial ceiling: the governor
        # floors at min_speed, so the charge caps itself at
        # max_speed/min_speed = 2.5x. Read on the UNION clearance — a
        # preference has to be comparable across edges, so it may not shift
        # with the edge's own drift row (feasibility stays per-heading).
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
            # Straight edges keep the body yaw, so the drift row is fixed by
            # the bin the edge leaves from and no curvature is added.
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
                # A turn in place has no direction of travel and therefore no
                # drift row: the union is the honest shape for it (and covers
                # the measured turn-in-place box with room to spare).
                if free[nb, UNION, i, j] and d + yc < dist[nb, i, j]:
                    dist[nb, i, j] = d + yc
                    prev[nb, i, j] = (b_, i, j)
                    heapq.heappush(q, (d + yc, nb, i, j))
                # Blend edges: walk and turn in the same step, discounted —
                # without these the lattice can only express turn-THEN-walk,
                # so it rotated in place even in open space. Judged at the yaw
                # they arrive in, as their cells always were, and widened by
                # the splay one bin of turn over their own length costs.
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

            # Shortcut smoothing, validity-preserving: replace state runs by
            # straight SE(2) interpolations (yaw = shortest arc) whenever every
            # interpolated body pose clears the margin — the staircase is
            # lattice quantization, not the optimum. A shortcut may never get
            # closer to the world than the raw detour it replaces (capped at the
            # comfort preference), else smoothing re-cuts the corners the cost
            # paid to avoid.
            #
            # The raw states' own clearance is the reference the shortcut has
            # to hold: a standing pose has no drift, so it is a union reading.
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
        """The published route, trimmed to here and carried to the goal —
        or None when this map no longer lets the body walk it.

        Re-validation is instant and unfiltered: an obstacle the map shows
        today invalidates the route today. Delaying belief in one is a
        robustness layer priced in collisions, and map noise flapping a
        corridor is perception's ledger, not the planner's to absorb.
        """
        route = trim_to_pose(np.asarray(incumbent, dtype=float).reshape(-1, 3), start)
        if len(route) < 2:
            return None
        end = route[-1]
        # The goal moves under the incumbent between replans (the carrot
        # advances ~0.2 m in the field), so the route rarely ends on it any
        # more. Carry it the rest of the way: the straight chord when the
        # chord is clear, and the search's own answer from the far end when it
        # is not. A goal that jumped is the caller's business — it drops the
        # incumbent rather than asking for a route across the world.
        if cell_of((float(end[0]), float(end[1]))) != (gi, gj):
            tgt = np.array([goal[0], goal[1], end[2]])
            if seg_free(end, tgt, margin):
                route = np.vstack([route, tgt])
            else:
                tail = solve((float(end[0]), float(end[1]), float(end[2])))
                if tail is None:
                    return None
                route = np.vstack([route, tail])
        if any(not seg_free(a, b, margin) for a, b in itertools.pairwise(route)):
            return None
        return route

    route = committed()
    if route is None:
        return result
    if result is None:
        # A still-walkable route beats a stub: refuse only when neither the
        # fresh search nor the carried incumbent has anywhere to go.
        return route

    def priced(p: NDArray[np.float64]) -> NDArray[np.float64]:
        """Both routes are priced from where the robot actually IS — the fresh
        answer opens at the cell its seed snapped to, up to half a cell diagonal
        away, and that walk is real."""
        here = np.array([[start[0], start[1], start[2]]])
        return p if np.allclose(p[0], here[0]) else np.vstack([here, p])

    fresh = path_cost(grid, priced(result), emb)
    held = path_cost(grid, priced(route), emb)
    return result if fresh < held - commit_margin else route
