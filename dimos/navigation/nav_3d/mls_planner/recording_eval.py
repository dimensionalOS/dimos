#!/usr/bin/env python
"""False-negative eval for the MLS planner on a recorded multi-floor dataset.

Idea (issue #2996, scoped with Andrew to "safe path produced iff one exists"):
  * The robot physically traversed the whole recording in one continuous run, so
    ANY two points on its odometry trajectory are connected by a real, feasible
    path -- that is free ground truth, no manual labelling.
  * We replay the recording through the production pipeline (RayTraceMap ->
    MLSPlanner.update_region) to build the accumulated multi-floor map, then ask
    plan(A, B) for many trajectory-derived pairs A, B.
  * plan() -> None on a pair that is feasible-by-construction is a FALSE NEGATIVE
    (Andrew: "as soon as there is any disconnect between start and goal it doesn't
    plan"). We break those down by floor stratum and, via a union-find over the
    planner's own node graph, prove whether start/goal fell in different connected
    components (the exact "disconnect" failure).
  * For pairs that DO plan, a light robot-box-vs-voxel slide flags any path that
    clips occupied voxels (the "keep paths safe" guardrail). Collision detection
    proper is a separate downstream module, so this is a guardrail, not a gate.

Deliverable is a scorecard: false-negative rate (overall + a floor x floor
matrix) and unsafe-path count. Objective (Andrew): drive FNs down while keeping
unsafe at 0.

Run:  ~/dimos/.venv/bin/python recording_eval.py [db] [--max-frames N] [--pairs-per-cell K]
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass, field

import numpy as np

from dimos.mapping.ray_tracing.transformer import RayTraceMap
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import FnTransformer
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner

# ---- planner / pipeline params (match plan_rrd + the wiki command) ----
VS = 0.08          # voxel size (m)
ROBOT_H = 0.4      # robot height (m) -- from the wiki's canonical command
STEP_THRESH = 0.16
# Safety guardrail footprint: flag paths that come CLOSER to a voxel than the
# planner's own hard wall_clearance (0.1 m) -- i.e. a genuine clearance violation,
# not a path legitimately hugging a wall at the 0.1 m limit.
ROBOT_RADIUS = 0.08

# Floor strata (m), derived from the athens z-histogram: basement/landing/ground/upper.
STRATA = [
    ("basement", -7.0, -4.0),
    ("landing", -4.0, -1.5),
    ("ground", -1.5, 1.0),
    ("upper", 1.0, 4.0),
]


def stratum_of(z: float) -> str | None:
    for name, lo, hi in STRATA:
        if lo <= z < hi:
            return name
    return None


def attach_pose(pair):
    lidar_obs, odom_obs = pair.data
    o = odom_obs.data
    return lidar_obs.with_pose((
        float(o.position.x), float(o.position.y), float(o.position.z),
        float(o.orientation.x), float(o.orientation.y),
        float(o.orientation.z), float(o.orientation.w),
    ))


@dataclass
class BuildResult:
    planner: MLSPlanner
    feet: np.ndarray          # (F,3) robot foot positions along the trajectory
    voxel_curve: list         # (frame, voxel_count) samples, to confirm growth


def build_map(db: str, max_frames: int | None, progress: bool = True) -> BuildResult:
    """Replay the recording, accumulating the map; collect robot foot positions."""
    store = SqliteStore(path=db, must_exist=True)
    feet = []
    curve = []
    with store:
        assert "pointlio_lidar" in store.list_streams(), store.list_streams()
        lidar = store.stream("pointlio_lidar", PointCloud2).order_by("ts")
        odom = store.stream("pointlio_odometry", Odometry).order_by("ts")
        pose_tagged = lidar.align(odom, tolerance=0.05).transform(FnTransformer(attach_pose))
        ray_pipeline = pose_tagged.transform(RayTraceMap(
            voxel_size=VS, max_range=30.0, ray_subsample=1, shadow_depth=0.1,
            grace_depth=0.2, emit_every=1, min_health=-1, max_health=5, support_min=4))
        planner = MLSPlanner(
            voxel_size=VS, robot_height=ROBOT_H, max_overhead_m=2.0,
            surface_closing_radius=0.3, node_spacing_m=1.0, wall_clearance_m=0.1,
            wall_buffer_m=0.75, wall_buffer_weight=100.0, step_threshold_m=STEP_THRESH,
            step_penalty_weight=4.0)
        n = 0
        t0 = time.time()
        for ray_obs in ray_pipeline:
            if ray_obs.pose_tuple is None:
                continue
            ox, oy, radius, z_min, z_max = ray_obs.tags["region_bounds"]
            px, py, pz, *_ = ray_obs.pose_tuple
            planner.update_region(ray_obs.data.points_f32(), (ox, oy), radius, z_min, z_max, float(pz))
            feet.append((float(px), float(py), float(pz) - ROBOT_H))
            n += 1
            if n % 100 == 0:
                curve.append((n, planner.voxel_count()))
                if progress:
                    print(f"  frame {n:4d}  voxels={planner.voxel_count():7d}  "
                          f"z={pz:6.2f}  {time.time()-t0:5.1f}s", flush=True)
            if max_frames and n >= max_frames:
                break
    return BuildResult(planner=planner, feet=np.array(feet, dtype=np.float64), voxel_curve=curve)


# ---------------- connectivity diagnostic (union-find over node graph) -------------
def _key(p, vs=VS):
    return (int(np.floor(p[0] / vs)), int(np.floor(p[1] / vs)), int(round(p[2] / vs)) - 1)


def node_components(planner: MLSPlanner):
    """Union-find over node_edges() segment endpoints == the components plan() routes over.

    Returns (comp_of_node: list[int], nodes: (K,3)). A node absent from any segment
    is its own singleton component (the isolated-node case).
    """
    nodes = planner.nodes()
    segs = planner.node_edges()  # (E,7): [x0,y0,z0,x1,y1,z1,cost]
    parent: dict = {}

    def find(k):
        parent.setdefault(k, k)
        root = k
        while parent[root] != root:
            root = parent[root]
        while parent[k] != root:
            parent[k], k = root, parent[k]
        return root

    def union(a, b):
        parent[find(a)] = find(b)

    for r in segs:
        union(_key(r[0:3]), _key(r[3:6]))
    node_keys = [_key(n) for n in nodes]
    roots = {}
    comp_of = []
    for k in node_keys:
        root = find(k) if k in parent else k
        comp_of.append(roots.setdefault(root, len(roots)))
    return comp_of, nodes


def same_component(nodes, comp_of, a, b) -> bool | None:
    """Nearest node (3D) to a and to b; are they in the same node component?

    None if there are no nodes. Must use 3D distance: on a multi-floor map the
    floors overlap in xy (the stairwell), so an xy-only nearest node could land on
    the wrong floor. Treat as a strong hint; the plan() result is the truth.
    """
    if len(nodes) == 0:
        return None
    da = np.linalg.norm(nodes - np.asarray(a), axis=1)
    db = np.linalg.norm(nodes - np.asarray(b), axis=1)
    return comp_of[int(np.argmin(da))] == comp_of[int(np.argmin(db))]


# ---------------- safety guardrail (robot-box vs occupied voxels) -------------
def path_is_safe(occ: set, path: np.ndarray, radius=ROBOT_RADIUS, height=ROBOT_H, vs=VS) -> bool:
    """Slide a robot-shaped box along the path; unsafe if it intersects a voxel.

    Skips the floor the robot stands on (band starts 2 voxels above the foot) and
    checks the body column [foot+2vs, foot+height] within the footprint radius.
    Light guardrail, not a certified collision check.
    """
    rc = int(np.ceil(radius / vs))
    z_lo = 2  # cells above foot to clear the standing surface
    z_hi = max(z_lo + 1, int(np.floor(height / vs)))
    step = vs  # sample step < voxel edge so the box can't tunnel a thin wall
    for i in range(len(path) - 1):
        p0, p1 = path[i], path[i + 1]
        seg = np.linalg.norm(p1 - p0)
        for t in np.linspace(0.0, 1.0, max(2, int(seg / step) + 1)):
            c = p0 + t * (p1 - p0)
            cx, cy, cz = int(np.floor(c[0] / vs)), int(np.floor(c[1] / vs)), int(np.floor(c[2] / vs))
            for dz in range(z_lo, z_hi + 1):
                for dx in range(-rc, rc + 1):
                    for dy in range(-rc, rc + 1):
                        if dx * dx + dy * dy > rc * rc:
                            continue
                        if (cx + dx, cy + dy, cz + dz) in occ:
                            return False
    return True


# ---------------- pair sampling: floor x floor matrix ------------------------
@dataclass
class Scorecard:
    total: int = 0
    false_neg: int = 0
    unsafe: int = 0
    # (stratA, stratB) -> [n_pairs, n_false_neg, n_disconnected, n_unsafe]
    cells: dict = field(default_factory=dict)
    # graph fragmentation context
    n_nodes: int = 0
    n_components: int = 0
    largest_frac: float = 0.0


def pick_representatives(feet: np.ndarray, per_stratum: int) -> dict:
    """Deterministically pick per_stratum foot positions within each floor stratum."""
    reps = {}
    for name, lo, hi in STRATA:
        # feet z has robot_height subtracted; compare the pose z (foot z + ROBOT_H) to bands.
        idx = np.where((feet[:, 2] + ROBOT_H >= lo) & (feet[:, 2] + ROBOT_H < hi))[0]
        if len(idx) == 0:
            continue
        take = np.linspace(0, len(idx) - 1, min(per_stratum, len(idx))).astype(int)
        reps[name] = feet[idx[take]]
    return reps


def evaluate(build: BuildResult, per_cell: int = 3) -> Scorecard:
    planner = build.planner
    occ = set(map(tuple, np.floor(planner.voxel_map() / VS).astype(int)))
    comp_of, nodes = node_components(planner)
    reps = pick_representatives(build.feet, per_cell)
    sc = Scorecard()
    # fragmentation context: how many disjoint components did the map shatter into?
    if len(comp_of):
        from collections import Counter
        sizes = Counter(comp_of)
        sc.n_nodes = len(comp_of)
        sc.n_components = len(sizes)
        sc.largest_frac = max(sizes.values()) / len(comp_of)
    strat_names = [s[0] for s in STRATA if s[0] in reps]
    for sa in strat_names:
        for sb in strat_names:
            key = (sa, sb)
            cell = [0, 0, 0, 0]
            for a in reps[sa]:
                for b in reps[sb]:
                    if np.allclose(a, b):
                        continue
                    cell[0] += 1
                    sc.total += 1
                    path = planner.plan(tuple(map(float, a)), tuple(map(float, b)))
                    if path is None:
                        cell[1] += 1
                        sc.false_neg += 1
                        conn = same_component(nodes, comp_of, a, b)
                        if conn is False:
                            cell[2] += 1
                    else:
                        if not path_is_safe(occ, np.asarray(path, dtype=np.float64)):
                            cell[3] += 1
                            sc.unsafe += 1
            sc.cells[key] = cell
    return sc


def print_scorecard(sc: Scorecard):
    print("\n================ MLS planner false-negative scorecard ================")
    print(f"pairs tested (all feasible by construction): {sc.total}")
    fnr = 100.0 * sc.false_neg / sc.total if sc.total else 0.0
    print(f"FALSE NEGATIVES (feasible but plan()->None): {sc.false_neg}  ({fnr:.1f}%)")
    print(f"UNSAFE paths (returned path clips a voxel):  {sc.unsafe}")
    print(f"graph: {sc.n_nodes} nodes shattered into {sc.n_components} components; "
          f"largest holds {100*sc.largest_frac:.1f}% of nodes")
    print("\nfloor x floor  [pairs | false-neg | of-which-disconnected | unsafe]")
    names = sorted({k[0] for k in sc.cells} | {k[1] for k in sc.cells},
                   key=lambda n: [s[1] for s in STRATA if s[0] == n][0])
    hdr = "start\\goal  " + "".join(f"{n:>14}" for n in names)
    print(hdr)
    for sa in names:
        row = f"{sa:>10}  "
        for sb in names:
            c = sc.cells.get((sa, sb))
            row += f"{('%d|%d|%d|%d' % tuple(c)) if c else '-':>14}"
        print(row)
    print("======================================================================")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("db", nargs="?", default="data/mid360_athens_stairs.db")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--pairs-per-cell", type=int, default=3)
    args = ap.parse_args()

    t0 = time.time()
    print(f"building map from {args.db} ...", flush=True)
    build = build_map(args.db, args.max_frames)
    print(f"map built: {build.planner!r}", flush=True)
    print(f"voxel growth: {build.voxel_curve[:3]} ... {build.voxel_curve[-3:]}", flush=True)
    grew = all(build.voxel_curve[i][1] <= build.voxel_curve[i + 1][1]
               for i in range(len(build.voxel_curve) - 1))
    print(f"voxel_count monotonic non-decreasing (no floor evicted): {grew}", flush=True)
    sc = evaluate(build, per_cell=args.pairs_per_cell)
    print_scorecard(sc)
    print(f"\ntotal wall time: {time.time()-t0:.1f}s", flush=True)


if __name__ == "__main__":
    main()
