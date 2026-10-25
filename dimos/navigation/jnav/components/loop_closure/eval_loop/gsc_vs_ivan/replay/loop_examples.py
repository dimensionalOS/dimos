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

"""Pick representative gir_stairs2 Mid-360 loop closures (small/big yank x good/bad against post_process ground truth)
and dump everything the yank / confidence explainer page needs: both keyframes' scans, the ICP placement, the pose
graph before and after the loop was applied, and every input to the proposed equations.
usage: loop_examples.py <out.json> --run <dir with a.json/a.kf/.closures/.edges> --name a --gt-db <db> --gt-stream <stream>
       --world <world db> --label <recording label> [--body qx,qy,qz,qw (GSC keyframe frame = GT frame * body)]"""

import base64
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, "dimos/navigation/global_planner/mls_planner/utils")
import argparse

import reform_metrics as rm

from dimos.memory.store.sqlite import SqliteStore

parser = argparse.ArgumentParser()
parser.add_argument("out", type=Path)
parser.add_argument("--run", type=Path, required=True)
parser.add_argument("--name", default="a")
parser.add_argument("--gt-db", type=Path, required=True)
parser.add_argument("--gt-stream", required=True)
parser.add_argument("--world", type=Path, default=None)
parser.add_argument("--label", required=True)
parser.add_argument("--body", default="0,0,0,1")
parser.add_argument("--flag", default="", help="source,target of a loop to include as flagged")
parser.add_argument(
    "--old-edges", action="store_true", help="the run's .edges has the 48-byte records (no yank_z)"
)
parser.add_argument(
    "--timeline", type=int, default=0, help="extra examples evenly spaced in time over all revisits"
)
parser.add_argument(
    "--per-kind", type=int, default=2, help="examples per small/big x good/bad kind, spread over z"
)
parser.add_argument(
    "--icp-dir", type=Path, default=None, help="GSC loop_icp_dump_dir: the exact clouds ICP got"
)
args = parser.parse_args()
RUN, SRC, WORLD = args.run, args.gt_db, args.world
BODY = R.from_quat([float(v) for v in args.body.split(",")])
EDGE = np.dtype(
    [
        ("xyz", "<f4", 3),
        ("target_xyz", "<f4", 3),
        ("source_id", "<u4"),
        ("target_id", "<u4"),
        ("state", "<u4"),
        ("confidence", "<f4"),
        ("yank_z", "<f4"),
        ("ts", "<f8"),
    ]
)

run = json.loads((RUN / f"{args.name}.json").read_text())
config = run["config"]
kf = np.fromfile(RUN / f"{args.name}.kf", rm.KEYFRAME)
t0 = kf["ts"][0]
closures = rm.read_closures(RUN / f"{args.name}.kf.closures")
EDGE_OLD = np.dtype([d for d in EDGE.descr if d[0] != "yank_z"])
edges = np.fromfile(RUN / f"{args.name}.kf.edges", EDGE_OLD if args.old_edges else EDGE)
committed = {
    (e["source_id"], e["target_id"]): e
    for e in run.get("committed_closure_edges", run.get("committed_loop_edges"))
}
accepted = [
    d
    for d in run.get("closure_diagnostics", run.get("loop_diagnostics"))
    if d["outcome"] == "accepted"
]

# ground truth: post_process output, in the same body frame GSC's keyframes use
with SqliteStore(path=str(SRC)) as store:

    def pose_of(o):
        p = o.data.pose if hasattr(o.data, "pose") and hasattr(o.data.pose, "position") else o.data
        return (
            o.ts,
            p.position.x,
            p.position.y,
            p.position.z,
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w,
        )

    gt = np.array([pose_of(o) for o in getattr(store.streams, args.gt_stream).order_by("ts")])


def gt_pose(ts):
    i = int(np.clip(np.searchsorted(gt[:, 0], ts), 0, len(gt) - 1))
    return R.from_quat(gt[i, 4:]) * BODY, gt[i, 1:4]


def gt_relative(source, target):
    rs, ts_ = gt_pose(kf["ts"][source])
    rt, tt = gt_pose(kf["ts"][target])
    return rt.inv() * rs, rt.inv().apply(ts_ - tt)


rows = []
yanks = []
for d in accepted:
    a, b = d["source_id"], d["target_id"]
    edge = committed.get((a, b))
    scale_r = 1.4826 * np.median([y[0] for y in yanks]) if yanks else np.nan
    scale_t = 1.4826 * np.median([y[1] for y in yanks]) if yanks else np.nan
    yanks.append((d["yank_rotation_deg"], d["yank_translation"]))
    if edge is None or not np.isfinite(scale_r):
        continue
    loop_rotation, loop_translation = R.from_matrix(edge["rotation"]), np.array(edge["translation"])
    gt_rotation, gt_translation = gt_relative(a, b)
    err_t = float(np.linalg.norm(loop_translation - gt_translation))
    err_r = float(np.degrees((loop_rotation.inv() * gt_rotation).magnitude()))
    if d.get("sigma_rotation_deg") is not None:  # GSC recorded its own gate inputs: use them as-is
        scale_r, scale_t = d["sigma_rotation_deg"], d["sigma_translation_m"]
    z = float(np.hypot(d["yank_rotation_deg"] / scale_r, d["yank_translation"] / scale_t))
    rows.append(
        dict(
            diag=d,
            edge=edge,
            scale_r=float(scale_r),
            scale_t=float(scale_t),
            z=z,
            err_t=err_t,
            err_r=err_r,
        )
    )


def real(r):
    return abs(r["diag"]["source_id"] - r["diag"]["target_id"]) >= 30  # a revisit, not a neighbour


def good(r):
    return real(r) and r["err_t"] < 0.3 and r["err_r"] < 3


def bad(r):
    return real(r) and (r["err_t"] > 0.5 or r["err_r"] > 5)


def small(r):
    return 0.3 < r["z"] < 1.5


def big(r):
    return r["z"] > 3.0


picks = {}
for name, test in (
    ("small_good", lambda r: small(r) and good(r)),
    ("small_bad", lambda r: small(r) and bad(r)),
    ("big_good", lambda r: big(r) and good(r)),
    ("big_bad", lambda r: big(r) and bad(r)),
):
    pool = [r for r in rows if test(r)]
    # spread the two picks: the median one and the most extreme one of the pool
    pool.sort(key=lambda r: r["z"])
    if args.per_kind == 2:
        picks[name] = (
            [pool[len(pool) // 2], pool[-1] if "big" in name else pool[0]]
            if len(pool) > 2
            else pool
        )
    else:  # evenly spaced over the pool's z order, ends included
        picks[name] = (
            [
                pool[i]
                for i in sorted(
                    {
                        round(k * (len(pool) - 1) / max(1, args.per_kind - 1))
                        for k in range(args.per_kind)
                    }
                )
            ]
            if pool
            else []
        )
    print(
        name,
        len(pool),
        [
            (
                r["diag"]["source_id"],
                r["diag"]["target_id"],
                round(r["z"], 1),
                round(r["err_t"], 2),
                round(r["err_r"], 1),
            )
            for r in pool[:2]
        ],
        flush=True,
    )

if args.timeline:
    taken = {(r["diag"]["source_id"], r["diag"]["target_id"]) for g in picks.values() for r in g}
    pool = sorted(
        [
            r
            for r in rows
            if real(r) and (r["diag"]["source_id"], r["diag"]["target_id"]) not in taken
        ],
        key=lambda r: r["diag"]["source_time"],
    )
    for i in sorted(
        {round(k * (len(pool) - 1) / max(1, args.timeline - 1)) for k in range(args.timeline)}
        if pool
        else []
    ):
        r = pool[i]
        size = "big" if big(r) else "small" if r["z"] < 1.5 else "mid"
        truth = "good" if good(r) else "bad" if bad(r) else "fair"
        picks.setdefault(f"{size}_{truth}", []).append(r)
if args.flag:
    flag = tuple(int(v) for v in args.flag.split(","))
    picks["flagged_by_jeff"] = [
        r for r in rows if (r["diag"]["source_id"], r["diag"]["target_id"]) == flag
    ]
for name, group in picks.items():
    print(
        "picked",
        name,
        [
            (
                r["diag"]["source_id"],
                r["diag"]["target_id"],
                round(r["z"], 1),
                round(r["err_t"], 2),
                round(r["err_r"], 1),
            )
            for r in group
        ],
        flush=True,
    )

# graph states: each keyframe's pose after each closure event
states = []
for ts, records in closures:
    rotation = R.from_quat(records["dq"].astype(float)) * R.from_quat(records["q"].astype(float))
    xyz = (
        R.from_quat(records["dq"].astype(float)).apply(records["xyz"].astype(float)) + records["dt"]
    )
    states.append((ts, records["id"].astype(int), xyz, rotation))
state_ts = np.array([s[0] for s in states])


def graph_at(ts):
    i = int(np.searchsorted(state_ts, ts, side="right")) - 1
    return states[i] if i >= 0 else None


def path_xy(state, upto):
    if state is None:
        return []
    _, ids, xyz, _ = state
    keep = ids <= upto
    pts = xyz[keep][:, :2]
    step = max(1, len(pts) // 900)
    return np.round(pts[::step], 2).tolist()


lidar, lidar_ts = {}, np.zeros(0)
if args.icp_dir is None:
    with SqliteStore(path=str(WORLD)) as store:
        lidar_ts = np.array([o.ts for o in store.streams.lidar.order_by("ts")])

        lidar = {}
        wanted = set()
        for group in picks.values():
            for r in group:
                for k in (r["diag"]["source_id"], r["diag"]["target_id"]):
                    i = int(np.searchsorted(lidar_ts, kf["ts"][k]))
                    wanted.update(range(max(0, i - 2), min(len(lidar_ts), i + 3)))
        for n, o in enumerate(store.streams.lidar.order_by("ts")):
            if n in wanted:
                lidar[n] = np.asarray(o.data.points_f32(), np.float64)


def local_cloud(k):
    raw = kf["raw"][k]
    rotation, translation = R.from_quat(raw[3:]), raw[:3]
    i = int(np.searchsorted(lidar_ts, kf["ts"][k]))
    world = np.concatenate(
        [lidar[j] for j in range(max(0, i - 2), min(len(lidar_ts), i + 3)) if j in lidar]
    )
    local = rotation.inv().apply(world - translation)
    local = local[np.linalg.norm(local[:, :2], axis=1) < 15]
    keys = np.floor(local / 0.15).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    local = local[first]
    if len(local) > 2500:
        local = local[np.random.default_rng(k).choice(len(local), 2500, replace=False)]
    return local


def icp_inputs(a, b):
    """The source cloud and target submap exactly as GSC's ICP got them, and ICP's transform."""
    raw = (args.icp_dir / f"{a}_{b}.icp").read_bytes()
    ns, nt = np.frombuffer(raw, "<u4", 2)
    points = np.frombuffer(raw, "<f4", 3 * (ns + nt), 8).reshape(-1, 3).astype(np.float64)
    tail = np.frombuffer(raw, "<f8", 13, 8 + 12 * (ns + nt))
    return points[:ns], points[ns:], tail[:9].reshape(3, 3), tail[9:12], float(tail[12])


def extended_inputs(a, b):
    """The biggest extended-ICP stage GSC ran for this candidate (stacked source + target keyframes), or None."""
    stages = sorted(
        args.icp_dir.glob(f"{a}_{b}_x*.icp"), key=lambda p: int(p.stem.rsplit("_x", 1)[1])
    )
    if not stages:
        return None
    raw = stages[-1].read_bytes()
    ns, nt = np.frombuffer(raw, "<u4", 2)
    points = np.frombuffer(raw, "<f4", 3 * (ns + nt), 8).reshape(-1, 3)
    tail = np.frombuffer(raw, "<f8", 13, 8 + 12 * (ns + nt))
    # thinned so a page of examples stays under the artifact size limit; the dump keeps every point
    return dict(
        keyframes=int(stages[-1].stem.rsplit("_x", 1)[1]),
        source_points=int(ns),
        target_points=int(nt),
        source_b64=base64.b64encode(
            np.asarray(thin(points[:ns], 40000, a), "<f4").tobytes()
        ).decode(),
        target_b64=base64.b64encode(
            np.asarray(thin(points[ns:], 40000, b), "<f4").tobytes()
        ).decode(),
        rotation=tail[:9].reshape(3, 3).tolist(),
        translation=tail[9:12].tolist(),
        fitness=float(tail[12]),
    )


def thin(points, limit, seed):
    if len(points) > limit:
        points = points[np.random.default_rng(seed).choice(len(points), limit, replace=False)]
    return points


def applied_at(a, b):
    hit = edges[(edges["source_id"] == a) & (edges["target_id"] == b) & (edges["state"] == 6)]
    return float(hit["ts"][0]) if len(hit) else None


examples = []
for name, group in picks.items():
    for r in group:
        d, edge = r["diag"], r["edge"]
        a, b = d["source_id"], d["target_id"]
        if args.icp_dir is not None:
            source, target, icp_r, icp_t, icp_fitness = icp_inputs(a, b)
            target_cloud = target  # every point ICP got, unthinned
            held = source
            placed = source @ icp_r.T + icp_t
        else:
            source_cloud, target_cloud = local_cloud(a), local_cloud(b)
            loop_rotation, loop_translation = (
                R.from_matrix(edge["rotation"]),
                np.array(edge["translation"]),
            )
            placed = loop_rotation.apply(source_cloud) + loop_translation
        # where the graph held the source relative to the target when the loop was proposed
        before = graph_at(d["source_time"])
        graph_relative = None
        if before is not None and a in set(before[1]) and b in set(before[1]):
            ids = list(before[1])
            ra, rb = before[3][ids.index(a)], before[3][ids.index(b)]
            xa, xb = before[2][ids.index(a)], before[2][ids.index(b)]
            graph_relative = (rb.inv() * ra, rb.inv().apply(xa - xb))
        else:
            raw_a, raw_b = kf["raw"][a], kf["raw"][b]
            rb = R.from_quat(raw_b[3:])
            graph_relative = (
                rb.inv() * R.from_quat(raw_a[3:]),
                rb.inv().apply(raw_a[:3] - raw_b[:3]),
            )
        if args.icp_dir is None:
            held = graph_relative[0].apply(source_cloud) + graph_relative[1]
        fit = cKDTree(target_cloud[:, :2]).query(placed[:, :2])[0]
        applied = applied_at(a, b)
        after = graph_at(applied + 1e-6) if applied is not None else None
        examples.append(
            dict(
                kind=name,
                source_id=int(a),
                target_id=int(b),
                proposed_s=round(d["source_time"] - t0, 1),
                target_s=round(d["target_time"] - t0, 1),
                applied_s=round(applied - t0, 1) if applied is not None else None,
                gnc_weight=edge["gnc_weight"],
                inputs=dict(
                    scan_context_distance=d["scan_context_best"],
                    scan_context_threshold=config["scan_context_match_threshold"],
                    icp_fitness=d["icp_fitness"],
                    icp_threshold=config.get(
                        "closure_score_thresh", config.get("loop_score_thresh")
                    ),
                    degeneracy=d["degeneracy_min"],
                    min_degeneracy=config.get(
                        "closure_min_degeneracy", config.get("loop_min_degeneracy")
                    ),
                    yank_rotation_deg=d["yank_rotation_deg"],
                    yank_translation_m=d["yank_translation"],
                    sigma_rotation_deg=r["scale_r"],
                    sigma_translation_m=r["scale_t"],
                ),
                truth=dict(
                    error_translation_m=round(r["err_t"], 2),
                    error_rotation_deg=round(r["err_r"], 1),
                ),
                # little-endian float32 xyz triples, base64: the exact clouds, without rounding or thinning
                target_b64=base64.b64encode(np.asarray(target_cloud, "<f4").tobytes()).decode(),
                source_b64=base64.b64encode(np.asarray(held, "<f4").tobytes()).decode(),
                icp_rotation=icp_r.tolist() if args.icp_dir is not None else None,
                icp_translation=icp_t.tolist() if args.icp_dir is not None else None,
                extended=extended_inputs(a, b) if args.icp_dir is not None else None,
                median_neighbour_m=round(float(np.median(fit)), 3),
                path_before=path_xy(before, a),
                path_after=path_xy(after, a),
                source_xy_before=None
                if before is None
                else np.round(before[2][list(before[1]).index(a)][:2], 2).tolist()
                if a in set(before[1])
                else None,
                target_xy_before=None
                if before is None
                else np.round(before[2][list(before[1]).index(b)][:2], 2).tolist()
                if b in set(before[1])
                else None,
            )
        )
for e in examples:
    e["recording"] = args.label
args.out.write_text(json.dumps(dict(examples=examples)))
print("wrote", args.out, len(examples), "examples", flush=True)
