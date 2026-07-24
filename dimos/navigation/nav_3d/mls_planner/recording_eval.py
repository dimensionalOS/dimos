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

"""Recording-based false-negative eval for the MLS planner (issue #2996).

Ground truth for free
---------------------
The robot traversed the whole recording in one continuous run, so ANY two points
on its odometry trajectory are connected by a real, feasible path -- no manual
labelling. We replay the recording through the ONLINE pipeline
(RayTraceMap -> MLSPlanner.update_region), then ask the planner to connect
trajectory-sampled start/goal pairs. plan() -> None on a walkable pair is a
FALSE NEGATIVE (Andrew: "as soon as there is any disconnect between start and
goal it doesn't plan").

Why this shape
--------------
Planning is only as good as the map, so the point of this eval is to measure
whether changes to the ray-tracing voxel mapper's algorithm/config improve
planning accuracy. The mapper parameters are therefore a first-class input
(`MapperConfig`), and `sweep()` runs the eval across several mapper configs and
tabulates the false-negative rate for each -- an A/B harness for mapper work.

It is deliberately dataset-agnostic: floor levels are detected from the
trajectory itself (`detect_floors`), so it runs on any recording without tuning.

Outputs a scorecard: overall false-negative rate, a floor x floor matrix, a
false-negative-vs-separation breakdown, graph fragmentation stats, and a light
robot-box-vs-voxel safety count (reported, not gated -- true collision checking
is a separate downstream module).

CLI:
    python -m dimos.navigation.nav_3d.mls_planner.recording_eval <dataset.db>
    python -m dimos.navigation.nav_3d.mls_planner.recording_eval <dataset.db> --sweep
"""
from __future__ import annotations

import argparse
import time
from collections import Counter
from dataclasses import asdict, dataclass, field, replace

import numpy as np

from dimos.mapping.ray_tracing.transformer import RayTraceMap
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import FnTransformer
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner


@dataclass(frozen=True)
class MapperConfig:
    """Ray-tracing voxel mapper parameters -- the highest-leverage module, since
    planning is only as good as the map. `sweep()` varies these."""

    voxel_size: float = 0.08
    max_range: float = 30.0
    ray_subsample: int = 1
    shadow_depth: float = 0.1
    grace_depth: float = 0.2
    emit_every: int = 1
    min_health: int = -1
    max_health: int = 5
    support_min: int = 4


@dataclass(frozen=True)
class PlannerConfig:
    """MLS planner parameters. `voxel_size` is kept in sync with the mapper's."""

    voxel_size: float = 0.08
    robot_height: float = 0.4
    max_overhead_m: float = 2.0
    surface_closing_radius: float = 0.3
    node_spacing_m: float = 1.0
    wall_clearance_m: float = 0.1
    wall_buffer_m: float = 0.75
    wall_buffer_weight: float = 100.0
    step_threshold_m: float = 0.16
    step_penalty_weight: float = 4.0


# Convenience constants (also used by the demo/visualization scripts).
VS = MapperConfig().voxel_size
ROBOT_H = PlannerConfig().robot_height
STEP_THRESH = PlannerConfig().step_threshold_m


# --------------------------------------------------------------------------- #
# replay -> map
# --------------------------------------------------------------------------- #
def _attach_pose_from_odom(pair):
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
    feet: np.ndarray          # (F, 3) robot foot positions (pose xyz, z minus robot_height)
    voxel_curve: list         # [(frame, voxel_count)] samples
    mapper: MapperConfig
    planner_cfg: PlannerConfig


def build_map(
    db_path,
    mapper: MapperConfig = MapperConfig(),
    planner_cfg: PlannerConfig = PlannerConfig(),
    *,
    lidar_stream: str = "pointlio_lidar",
    odom_stream: str = "pointlio_odometry",
    align_tol: float = 0.05,
    max_frames: int | None = None,
    progress: bool = False,
) -> BuildResult:
    """Replay a recording through RayTraceMap -> MLSPlanner.update_region (the
    online/production path) and return the accumulated planner + foot trajectory."""
    if abs(mapper.voxel_size - planner_cfg.voxel_size) > 1e-9:
        planner_cfg = replace(planner_cfg, voxel_size=mapper.voxel_size)

    store = SqliteStore(path=str(db_path), must_exist=True)
    feet: list = []
    curve: list = []
    with store:
        streams = store.list_streams()
        if lidar_stream not in streams or odom_stream not in streams:
            raise ValueError(f"streams {lidar_stream!r}/{odom_stream!r} not found in {streams}")
        lidar = store.stream(lidar_stream, PointCloud2).order_by("ts")
        odom = store.stream(odom_stream, Odometry).order_by("ts")
        pose_tagged = lidar.align(odom, tolerance=align_tol).transform(
            FnTransformer(_attach_pose_from_odom))
        ray_pipeline = pose_tagged.transform(RayTraceMap(**asdict(mapper)))
        planner = MLSPlanner(**asdict(planner_cfg))

        n = 0
        t0 = time.time()
        for ray_obs in ray_pipeline:
            if ray_obs.pose_tuple is None:
                continue
            ox, oy, radius, z_min, z_max = ray_obs.tags["region_bounds"]
            px, py, pz, *_ = ray_obs.pose_tuple
            planner.update_region(ray_obs.data.points_f32(), (ox, oy), radius, z_min, z_max, float(pz))
            feet.append((float(px), float(py), float(pz) - planner_cfg.robot_height))
            n += 1
            if n % 100 == 0:
                curve.append((n, planner.voxel_count()))
                if progress:
                    print(f"  frame {n:4d}  voxels={planner.voxel_count():7d}  "
                          f"z={pz:6.2f}  {time.time() - t0:5.1f}s", flush=True)
            if max_frames and n >= max_frames:
                break
    feet_arr = np.array(feet, dtype=float)
    if feet_arr.size == 0:
        raise ValueError(
            f"no aligned lidar/odometry observations (align_tol={align_tol}) in "
            f"{db_path}; check the stream names and that lidar/odom timestamps overlap")
    return BuildResult(planner, feet_arr, curve, mapper, planner_cfg)


# --------------------------------------------------------------------------- #
# floor detection (dataset-agnostic)
# --------------------------------------------------------------------------- #
def detect_floors(feet: np.ndarray, robot_height: float, *,
                  bin_m: float = 0.2, dwell_frac: float = 0.04, merge_gap_m: float = 1.0) -> list:
    """Floor levels = the dwell modes of the trajectory's height histogram.

    The robot lingers on floors (tall histogram bins) and passes through stairs
    quickly (sparse bins), so the well-populated modes are the floors. Returns a
    sorted list of z levels (pose height)."""
    if feet.ndim != 2 or len(feet) == 0:
        raise ValueError("detect_floors: need a non-empty (N, 3) trajectory")
    z = feet[:, 2] + robot_height
    lo, hi = float(z.min()), float(z.max())
    if hi - lo < merge_gap_m:
        return [0.5 * (lo + hi)]
    nbins = int(np.clip(round((hi - lo) / bin_m), 8, 80))
    hist, edges = np.histogram(z, bins=nbins)
    centers = 0.5 * (edges[:-1] + edges[1:])
    thresh = dwell_frac * len(z)
    peaks = [centers[i] for i in range(nbins)
             if hist[i] >= thresh
             and (i == 0 or hist[i] >= hist[i - 1])
             and (i == nbins - 1 or hist[i] >= hist[i + 1])]
    merged: list = []
    for p in sorted(peaks):
        if merged and p - merged[-1] < merge_gap_m:
            merged[-1] = 0.5 * (merged[-1] + p)
        else:
            merged.append(p)
    return merged or [0.5 * (lo + hi)]


def floor_labels(feet: np.ndarray, levels: list, robot_height: float, band_m: float = 0.75) -> np.ndarray:
    """Label each foot position by nearest floor level, or -1 if it is between
    floors (on the stairs). `band_m` shrinks to half the closest floor gap."""
    if len(levels) > 1:
        band_m = min(band_m, 0.49 * float(np.min(np.diff(sorted(levels)))))
    z = feet[:, 2] + robot_height
    lab = np.full(len(feet), -1, dtype=int)
    lv = np.asarray(levels)
    for i, zi in enumerate(z):
        k = int(np.argmin(np.abs(zi - lv)))
        if abs(zi - lv[k]) <= band_m:
            lab[i] = k
    return lab


# --------------------------------------------------------------------------- #
# connectivity diagnostic (union-find over the planner's own node graph)
# --------------------------------------------------------------------------- #
def _key(p, vs=VS):
    return (int(np.floor(p[0] / vs)), int(np.floor(p[1] / vs)), int(round(p[2] / vs)) - 1)


def node_components(planner: MLSPlanner, vs: float = VS):
    """Union-find over node_edges() segment endpoints == the components plan()
    routes over. Returns (comp_of_node: list[int], nodes: (K,3))."""
    nodes = planner.nodes()
    segs = planner.node_edges()  # (E, 7): [x0,y0,z0,x1,y1,z1,cost]
    parent: dict = {}

    def find(k):
        parent.setdefault(k, k)
        root = k
        while parent[root] != root:
            root = parent[root]
        while parent[k] != root:
            parent[k], k = root, parent[k]
        return root

    for r in segs:
        parent[find(_key(r[0:3], vs))] = find(_key(r[3:6], vs))

    roots: dict = {}
    comp_of = []
    for n in nodes:
        k = _key(n, vs)
        root = find(k) if k in parent else k
        comp_of.append(roots.setdefault(root, len(roots)))
    return comp_of, nodes


def same_component(nodes, comp_of, a, b):
    """Nearest node (3D) to a and to b: same component? None if no nodes. 3D
    distance is required -- floors overlap in xy at the stairwell."""
    if len(nodes) == 0:
        return None
    ia = int(np.argmin(np.linalg.norm(nodes - np.asarray(a), axis=1)))
    ib = int(np.argmin(np.linalg.norm(nodes - np.asarray(b), axis=1)))
    return comp_of[ia] == comp_of[ib]


# --------------------------------------------------------------------------- #
# safety guardrail (robot-box vs occupied voxels) -- light, reported not gated
# --------------------------------------------------------------------------- #
def path_is_safe(occ: set, path: np.ndarray, *, radius: float, height: float, vs: float) -> bool:
    """Slide a robot-shaped box along the path; unsafe if it intersects a voxel.
    Skips the standing surface (band starts 2 voxels above the foot). Approximate:
    over-flags where the path legitimately hugs a stair edge."""
    rc = int(np.ceil(radius / vs))
    z_lo, z_hi = 2, max(3, int(np.floor(height / vs)))
    for i in range(len(path) - 1):
        p0, p1 = path[i], path[i + 1]
        seg = float(np.linalg.norm(p1 - p0))
        for t in np.linspace(0.0, 1.0, max(2, int(seg / vs) + 1)):
            c = p0 + t * (p1 - p0)
            cx, cy, cz = int(np.floor(c[0] / vs)), int(np.floor(c[1] / vs)), int(np.floor(c[2] / vs))
            for dz in range(z_lo, z_hi + 1):
                for dx in range(-rc, rc + 1):
                    for dy in range(-rc, rc + 1):
                        if dx * dx + dy * dy <= rc * rc and (cx + dx, cy + dy, cz + dz) in occ:
                            return False
    return True


# --------------------------------------------------------------------------- #
# evaluation
# --------------------------------------------------------------------------- #
@dataclass
class Scorecard:
    total: int = 0
    false_neg: int = 0
    unsafe: int = 0
    floor_levels: list = field(default_factory=list)
    cells: dict = field(default_factory=dict)          # (i, j) -> [pairs, fn, disconnected, unsafe]
    by_distance: dict = field(default_factory=dict)    # upper_bound_m -> [pairs, fn]
    n_nodes: int = 0
    n_components: int = 0
    largest_frac: float = 0.0
    mapper: MapperConfig | None = None
    planner_cfg: PlannerConfig | None = None

    @property
    def fn_rate(self) -> float:
        return self.false_neg / self.total if self.total else 0.0


def _pick(feet, labels, floor_k, per_floor):
    idx = np.where(labels == floor_k)[0]
    if len(idx) == 0:
        return np.empty((0, 3))
    take = np.linspace(0, len(idx) - 1, min(per_floor, len(idx))).astype(int)
    return feet[idx[take]]


def evaluate(build: BuildResult, *, per_floor: int = 3, dist_buckets=(2.0, 5.0, 10.0, 1e9)) -> Scorecard:
    """Plan every start/goal pair sampled from the trajectory; tally false negatives."""
    planner = build.planner
    cfg = build.planner_cfg
    vs = cfg.voxel_size
    occ = set(map(tuple, np.floor(planner.voxel_map() / vs).astype(int)))
    comp_of, nodes = node_components(planner, vs)
    levels = detect_floors(build.feet, cfg.robot_height)
    labels = floor_labels(build.feet, levels, cfg.robot_height)

    sc = Scorecard(floor_levels=[round(float(l), 1) for l in levels],
                   mapper=build.mapper, planner_cfg=cfg)
    if comp_of:
        sizes = Counter(comp_of)
        sc.n_nodes, sc.n_components = len(comp_of), len(sizes)
        sc.largest_frac = max(sizes.values()) / len(comp_of)

    reps = {k: _pick(build.feet, labels, k, per_floor) for k in range(len(levels))}
    radius = min(cfg.wall_clearance_m, vs)
    for i in range(len(levels)):
        for j in range(len(levels)):
            cell = [0, 0, 0, 0]
            for a in reps[i]:
                for b in reps[j]:
                    if np.allclose(a, b):
                        continue
                    cell[0] += 1
                    sc.total += 1
                    d = float(np.linalg.norm(a - b))
                    bucket = next(bk for bk in dist_buckets if d <= bk)
                    db_row = sc.by_distance.setdefault(bucket, [0, 0])
                    db_row[0] += 1
                    path = planner.plan(tuple(map(float, a)), tuple(map(float, b)))
                    if path is None:
                        cell[1] += 1
                        sc.false_neg += 1
                        db_row[1] += 1
                        if same_component(nodes, comp_of, a, b) is False:
                            cell[2] += 1
                    elif not path_is_safe(occ, np.asarray(path, dtype=float),
                                          radius=radius, height=cfg.robot_height, vs=vs):
                        cell[3] += 1
                        sc.unsafe += 1
            sc.cells[(i, j)] = cell
    return sc


def run_eval(db_path, mapper: MapperConfig = MapperConfig(),
             planner_cfg: PlannerConfig = PlannerConfig(), *,
             per_floor: int = 3, max_frames: int | None = None, progress: bool = False) -> Scorecard:
    build = build_map(db_path, mapper, planner_cfg, max_frames=max_frames, progress=progress)
    return evaluate(build, per_floor=per_floor)


def sweep(db_path, mapper_configs: list, planner_cfg: PlannerConfig = PlannerConfig(),
          *, per_floor: int = 3, max_frames: int | None = None, progress: bool = True) -> list:
    """Run the eval for each (label, MapperConfig); return [(label, Scorecard)].

    This is the A/B harness for mapper work: change the mapper config, see if the
    planning false-negative rate moves. Pass `max_frames` to iterate on a capped
    replay before committing to full runs."""
    rows = []
    for label, mc in mapper_configs:
        sc = run_eval(db_path, mc, planner_cfg, per_floor=per_floor, max_frames=max_frames)
        rows.append((label, sc))
        if progress:
            print(f"[sweep] {label:22s} FN={100 * sc.fn_rate:5.1f}%  "
                  f"nodes={sc.n_nodes:5d}  components={sc.n_components:5d}", flush=True)
    return rows


# --------------------------------------------------------------------------- #
# reporting
# --------------------------------------------------------------------------- #
def print_scorecard(sc: Scorecard):
    print("\n================ MLS planner false-negative scorecard ================")
    print(f"floors detected (z, m): {sc.floor_levels}")
    print(f"pairs tested (all feasible by construction): {sc.total}")
    print(f"FALSE NEGATIVES (feasible but plan()->None): {sc.false_neg}  ({100 * sc.fn_rate:.1f}%)")
    print(f"UNSAFE produced paths (approx guardrail):    {sc.unsafe}")
    print(f"graph: {sc.n_nodes} nodes in {sc.n_components} components; "
          f"largest holds {100 * sc.largest_frac:.1f}%")
    print("\nfalse-negatives by start/goal separation:")
    for bk in sorted(sc.by_distance):
        pairs, fn = sc.by_distance[bk]
        label = f"<={bk:g}m" if bk < 1e9 else ">10m"
        print(f"  {label:>6}: {fn}/{pairs}  ({100 * fn / max(1, pairs):.0f}%)")
    print("\nfloor x floor  [pairs | false-neg | of-which-disconnected | unsafe]")
    lv = sc.floor_levels
    header = "start\\goal " + "".join(f"{f'z={v:g}':>15}" for v in lv)
    print(header)
    for i, vi in enumerate(lv):
        row = f"{f'z={vi:g}':>10} "
        for j in range(len(lv)):
            c = sc.cells.get((i, j))
            row += f"{('%d|%d|%d|%d' % tuple(c)) if c else '-':>15}"
        print(row)
    print("======================================================================")


def print_sweep(rows: list):
    print("\n==================== mapper-config sweep ====================")
    print(f"{'config':24s}{'FN rate':>10}{'cross-floor FN':>16}{'nodes':>8}{'comps':>8}")
    for label, sc in rows:
        off = [c for (i, j), c in sc.cells.items() if i != j and c[0] > 0]
        off_pairs = sum(c[0] for c in off) or 1
        off_fn = sum(c[1] for c in off)
        print(f"{label:24s}{100 * sc.fn_rate:9.1f}%{100 * off_fn / off_pairs:15.1f}%"
              f"{sc.n_nodes:8d}{sc.n_components:8d}")
    print("lower FN rate = the mapper config yields a better-connected map for planning")
    print("============================================================")


def main():
    ap = argparse.ArgumentParser(description="Recording-based MLS-planner false-negative eval")
    ap.add_argument("db", help="recording .db (path)")
    ap.add_argument("--sweep", action="store_true", help="A/B several mapper configs")
    ap.add_argument("--per-floor", type=int, default=3, help="representative points sampled per floor")
    ap.add_argument("--max-frames", type=int, default=None, help="cap replayed frames (debugging)")
    args = ap.parse_args()

    planner_cfg = PlannerConfig()
    if args.sweep:
        configs = [
            ("baseline", MapperConfig()),
            ("support_min=2", MapperConfig(support_min=2)),
            ("support_min=0", MapperConfig(support_min=0)),
            ("voxel=0.10", MapperConfig(voxel_size=0.10)),
            ("shadow_depth=0.2", MapperConfig(shadow_depth=0.2)),
        ]
        print_sweep(sweep(args.db, configs, planner_cfg,
                          per_floor=args.per_floor, max_frames=args.max_frames))
    else:
        t0 = time.time()
        build = build_map(args.db, MapperConfig(), planner_cfg,
                          max_frames=args.max_frames, progress=True)
        print(f"map built: {build.planner!r}  ({time.time() - t0:.0f}s)")
        print_scorecard(evaluate(build, per_floor=args.per_floor))


if __name__ == "__main__":
    main()
