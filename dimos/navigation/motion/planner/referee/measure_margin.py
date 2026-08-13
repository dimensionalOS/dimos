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

"""Bake `scenarios.COMMIT_MARGIN` — the price advantage that earns a switch.

    python -m dimos.navigation.motion.planner.referee.measure_margin
    python -m dimos.navigation.motion.planner.referee.measure_margin --only quanta
    python -m dimos.navigation.motion.planner.referee.measure_margin --only jitter

The margin is not a preference, it is a NOISE FLOOR, so it is measured the way
the envelope is (`simulation/envelope.py --bake`) rather than tuned against a
score. Two things move a route's price while the world stands still, and the
constant has to cover both:

**quanta** — the continuous pose enters the search as a lattice seed (one of 16
yaw bins, one 0.12 m cell), so a replan from a pose a few millimetres along can
be a different query. The floor is measured where that is visible and nowhere
else: put the pose exactly ON the bin and cell boundaries and step a tenth of a
degree and a millimetre to either side. The eight poses are the same pose to
anything that measures the world, and they seed eight different lattice states;
whatever the argmin's price does across them, it did for no reason.

Sweeping a WHOLE bin and half a cell instead — the amendment's first wording —
measures something else and says so loudly: the spread is 0.75 m median, 2.81 m
p99 on gen40 + curated, because at 22.5° of yaw the robot really is facing
somewhere else and the route really is worse. A margin built from that number
(4.2 m with headroom, against 2.4 m routes) would make the first plan permanent,
which is not commitment but deafness. `--wide` reproduces it.

**jitter** — the map is a live raycast, and consecutive local_map frames of a
scene nobody is touching disagree. Hold ONE route fixed and price it under each
frame in turn: the spread is what the world's own noise is worth.

Both are read in `path_cost`'s unit (open-space metres) on one field per
measurement, so nothing but the effect under test can move. The two are
independent — a lattice seed and a lidar return know nothing about each other —
so the constant is their p99s added, plus headroom.
"""

from __future__ import annotations

import argparse
import math
from typing import Any

import numpy as np

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.motion.embodiment import EMBODIMENTS
from dimos.navigation.motion.geometry import AvoidanceConfig
from dimos.navigation.motion.obstacles import hard_points, load as load_model
from dimos.navigation.motion.planner.planners.base import load
from dimos.navigation.motion.scenarios import (
    CELL,
    FINE,
    SCENARIOS,
    Scenario,
    anchor,
    generated,
    path_cost,
    truth_grid,
)

PAD = 1.5
GRID_PAD = 0.72  # 3 PERIODs, se2_path's own
CLOUD_STEP = 0.05  # referee/sim.py's planner-visible sampling
STALL_ARC = 0.3  # shorter than this is a refusal, not a route
# Where along its own answer a replan happens, as a fraction of arc: the
# referee's consistency chain walks the start pose down the published path, and
# these are the poses a margin has to survive.
SPOTS = (0.0, 0.25, 0.5, 0.75)
HEADROOM = 1.5  # what the constant is multiplied by once both floors are in
# How far either side of a quantization boundary a pose is nudged. Small enough
# that nothing about the world or the body has changed -- gen001's pinned repro
# is exactly this, 0.1 deg across the -11.25 deg bin edge.
EPS_YAW = math.radians(0.1)
EPS_XY = 0.001


def _states(path: Any) -> np.ndarray:
    return np.array(
        [[p.position.x, p.position.y, p.orientation.euler[2]] for p in path.poses]
    ).reshape(-1, 3)


def _arc(xy: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))) if len(xy) > 1 else 0.0


def _from(pose: tuple[float, float, float], states: np.ndarray) -> np.ndarray:
    """Price every candidate route from the SAME pose, so the perturbation's own
    displacement cannot masquerade as a difference between the routes."""
    return np.vstack([[[pose[0], pose[1], pose[2]]], states])


def _pct(v: list[float], q: float) -> float:
    return float(np.percentile(v, q)) if v else math.nan


def _report(name: str, v: list[float]) -> float:
    if not v:
        print(f"{name}: nothing measured")
        return math.nan
    p99 = _pct(v, 99)
    print(
        f"{name}: n {len(v)}  median {_pct(v, 50):.4f}  p90 {_pct(v, 90):.4f}  "
        f"p99 {p99:.4f}  max {max(v):.4f}"
    )
    return p99


# ------------------------------------------------------------------ quanta --


def query_poses(states: np.ndarray) -> list[tuple[float, float, float]]:
    """The route's own start plus the spots a replan lands on, tangent-facing."""
    xy = states[:, :2]
    if len(xy) < 3:
        return [(float(states[0][0]), float(states[0][1]), float(states[0][2]))]
    arcs = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
    out = []
    for f in SPOTS:
        i = int(np.clip(np.searchsorted(arcs, f * float(arcs[-1])), 1, len(xy) - 2))
        tang = xy[i + 1] - xy[i - 1]
        yaw = math.atan2(tang[1], tang[0]) if np.hypot(*tang) > 1e-9 else float(states[i][2])
        out.append((float(xy[i][0]), float(xy[i][1]), yaw))
    return out


def seed_family(
    q: tuple[float, float, float], wide: bool
) -> tuple[tuple[float, float, float], list[tuple[float, float, float]]]:
    """The poses whose SEED differs and whose pose does not, and the pose they
    are all priced from.

    The lattice quantizes yaw into 16 bins centred on `-pi + k * step` and
    position into cells centred on multiples of CELL, so a boundary sits half a
    quantum off each centre. Put the pose on the boundaries and step either side:
    eight seeds, one physical pose. `wide` restores the amendment's first
    wording -- a whole bin and half a cell -- which measures a different thing.
    """
    yaw_step = 2.0 * math.pi / 16.0
    if wide:
        return q, [
            (q[0] + dx * CELL / 2.0, q[1] + dy * CELL / 2.0, q[2] + db * yaw_step)
            for db in (-1, 0, 1)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
        ]
    here = (
        (math.floor(q[0] / CELL) + 0.5) * CELL,
        (math.floor(q[1] / CELL) + 0.5) * CELL,
        math.floor((q[2] + math.pi) / yaw_step) * yaw_step - math.pi + yaw_step / 2.0,
    )
    return here, [
        (here[0] + sx * EPS_XY, here[1] + sy * EPS_XY, here[2] + sb * EPS_YAW)
        for sb in (-1, 1)
        for sx in (-1, 1)
        for sy in (-1, 1)
    ]


def quanta(
    scenarios: list[Scenario], planner: str, wide: bool = False
) -> tuple[list[float], list[float]]:
    """Price spread of the argmin across the seeds one physical pose can take.

    Returned twice over: in metres, and as a fraction of the cheapest route in
    the family. Which of the two is the stabler statistic is the question the
    constant's SHAPE turns on, so both are reported rather than one chosen here.
    """
    spreads: list[float] = []
    ratios: list[float] = []
    for sc in scenarios:
        cloud_pts = (
            np.concatenate([b.surface(CLOUD_STEP) for b in sc.boxes])
            if sc.boxes
            else np.empty((0, 3))
        )
        cloud = PointCloud2.from_numpy(cloud_pts.astype(np.float32), frame_id="world")
        obstacles = hard_points(load_model("raw_band", sc.emb), cloud.points_f32(), 0.0)[:, :2]
        ep = load(planner)(sc, AvoidanceConfig())
        ep.reset()
        seed = _states(ep.plan(obstacles, sc.start, sc.goal))
        if _arc(seed[:, :2]) < STALL_ARC:
            continue  # a sealed world has no route whose price could move
        fgx, fgy, grid, _ = truth_grid(sc.boxes, sc.start, sc.goal)
        for q in query_poses(seed):
            here, family = seed_family(q, wide)
            costs = []
            for p in family:
                out = _states(ep.plan(obstacles, p, sc.goal))
                if _arc(out[:, :2]) < STALL_ARC:
                    continue  # refused from this seed: no price to compare
                costs.append(path_cost(fgx, fgy, grid, _from(here, out), sc.emb))
            if len(costs) > 1:
                spreads.append(max(costs) - min(costs))
                ratios.append(spreads[-1] / max(min(costs), 1e-9))
        print(f"  {sc.name}: {len(spreads)} spreads so far", flush=True)
    return spreads, ratios


# ------------------------------------------------------------------ jitter --


def _grid(bounds: tuple[float, float, float, float], band: np.ndarray) -> Any:
    """A fine SDF over fixed bounds — only the point set changes between frames,
    and sample POSITIONS are absolute, so two frames are read at one ruler."""
    from scipy.spatial import cKDTree

    x0, y0, x1, y1 = bounds
    fgx = np.arange(x0 - GRID_PAD, x1 + GRID_PAD, FINE)
    fgy = np.arange(y0 - GRID_PAD, y1 + GRID_PAD, FINE)
    if not len(band):
        return fgx, fgy, np.full((len(fgx), len(fgy)), np.inf)
    FX, FY = np.meshgrid(fgx, fgy, indexing="ij")
    d, _ = cKDTree(band).query(np.column_stack([FX.ravel(), FY.ravel()]))
    return fgx, fgy, d.reshape(len(fgx), len(fgy))


def jitter(recording: str, embodiment: str, model: str, stride: int) -> list[float]:
    """Price change of ONE fixed route between consecutive local_map frames."""
    from dimos.navigation.motion.adapter.diagnose import load_recording

    emb = EMBODIMENTS[embodiment]
    band_model = load_model(model, emb)
    rec = load_recording(recording, "base_link", 5.0)
    ep = load("target")(Scenario("jitter", [], goal=(0.0, 0.0), emb=emb), AvoidanceConfig())
    ep.reset()
    deltas: list[float] = []
    for t in rec.ticks[::stride]:
        if t.imap + 1 >= len(rec.maps):
            continue
        ground_z = (t.pose[3] if len(t.pose) > 3 else 0.0) - emb.base_height
        pose = (float(t.pose[0]), float(t.pose[1]), float(t.pose[2]))
        here = hard_points(band_model, rec.maps[t.imap][1], ground_z)[:, :2]
        route = _states(ep.plan(here, pose, t.goal))
        if _arc(route[:, :2]) < STALL_ARC:
            continue  # a hold has no route to re-price
        xs = np.concatenate([route[:, 0], [pose[0], t.goal[0]]])
        ys = np.concatenate([route[:, 1], [pose[1], t.goal[1]]])
        bounds = (
            anchor(float(xs.min()) - PAD),
            anchor(float(ys.min()) - PAD),
            float(xs.max()) + PAD,
            float(ys.max()) + PAD,
        )
        costs = []
        for k in (t.imap, t.imap + 1):
            band = hard_points(band_model, rec.maps[k][1], ground_z)[:, :2]
            fgx, fgy, grid = _grid(bounds, band)
            costs.append(path_cost(fgx, fgy, grid, route, emb))
        deltas.append(abs(costs[1] - costs[0]))
    return deltas


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", default="quanta,jitter")
    ap.add_argument("--planner", default="target")
    ap.add_argument("--gen", type=int, default=40, help="generated worlds in the sweep")
    ap.add_argument("--curated", action="store_true", help="add the curated 16")
    ap.add_argument(
        "--wide", action="store_true", help="sweep a whole yaw bin and half a cell instead"
    )
    ap.add_argument("--recording", default="ml-trajectory-research/door2.zenoh.mcap")
    ap.add_argument("--embodiment", default="go2")
    ap.add_argument("--model", default="raw_band")
    ap.add_argument("--stride", type=int, default=1, help="tick subsample for --only jitter")
    args = ap.parse_args()
    passes = set(args.only.split(","))

    q99 = j99 = 0.0
    if "quanta" in passes:
        worlds = list(generated(args.gen)) + (list(SCENARIOS) if args.curated else [])
        span = "+-1 yaw bin, +-half cell" if args.wide else "either side of the seed boundaries"
        print(f"== quanta: {span}, {len(worlds)} worlds ==")
        spreads, ratios = quanta(worlds, args.planner, args.wide)
        q99 = _report("quanta (m)", spreads)
        _report("quanta (fraction of the route's own price)", ratios)
    if "jitter" in passes:
        print(f"== jitter: consecutive local_map frames of {args.recording} ==")
        j99 = _report("jitter", jitter(args.recording, args.embodiment, args.model, args.stride))

    total = (0.0 if math.isnan(q99) else q99) + (0.0 if math.isnan(j99) else j99)
    print(
        f"\ncombined p99 {total:.4f} m  x{HEADROOM} headroom -> COMMIT_MARGIN {total * HEADROOM:.3f}"
    )


if __name__ == "__main__":
    main()
