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

"""Export one recording of the GSC-vs-Ivan PGO report: per-keyframe top-down points (keyframe body frame), each
method's graph states over time, GT poses, GSC loop edges, closure-gate diagnostics and a bounded set of ICP clouds.
usage (from $GSC_WORKTREE, default ~/repos/_mls_reform_wt; PYTHONPATH=$GSC_SCORE_STUBS, default ~/.cache/gsc_score_stubs):
  export_recording.py <manifest entry: JSON file or inline JSON> <out dir>      -> <out dir>/<entry name>/...
Binary blobs are written base64 (.b64) so the folder also works where only text is served. Layouts are documented
next to each writer and decoded by viewer.html."""

import base64
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

WORKTREE = Path(
    os.environ.get("GSC_WORKTREE", Path.home() / "repos/_mls_reform_wt")
)  # dimos checkout the scorers run from
sys.path.insert(0, str(WORKTREE / "dimos/navigation/global_planner/mls_planner/utils"))
sys.path.insert(0, str(Path(__file__).parent))
from gt_poses import correction, gt_poses
import reform_metrics as rm

from dimos.memory.store.sqlite import SqliteStore

SCRIPTS = Path(
    os.environ.get("GSC_SCRIPTS", Path(__file__).resolve().parent.parent / "gsc_vs_ivan")
)  # score_run.py, tag_agreement.py
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
DIAG_COLUMNS = [
    "outcome",
    "source_id",
    "target_id",
    "source_time",
    "candidate_distance",
    "from_scan_context",
    "lowe_ratio",
    "scan_context_best",
    "descriptor_occupancy",
    "degeneracy_min",
    "icp_converged",
    "icp_fitness",
    "icp_overlap",
    "icp_translation_norm",
    "yank_translation",
    "yank_rotation_deg",
    "yank_drift_scale",
    "sigma_translation_m",
    "sigma_rotation_deg",
    "yank_z",
    "scan_context_factor",
    "icp_factor",
    "degeneracy_factor",
    "confidence",
    "required_confidence",
    "extended_keyframes",
    "single_inliers",
    "extended_inliers",
    "extended_fitness",
    "extended_confidence",
    "strain",
]
REJECTS = ("icp_reject", "degeneracy_reject", "yank_confidence_reject")

entry_arg, out_root = sys.argv[1], Path(sys.argv[2])
entry = json.loads(Path(entry_arg).read_text() if Path(entry_arg).exists() else entry_arg)
opts = dict(
    voxel_m=0.2,
    max_points=150,
    scans_per_keyframe=5,
    max_range_m=25.0,
    band_m=[0.3, 2.2],
    state_budget_bytes=4_400_000,
    odom_hz=2.0,
    icp_accepted=40,
    icp_rejected=40,
    icp_single=[1500, 2500],
    icp_extended=[2500, 3500],
)
opts.update(entry.get("options", {}))
out = out_root / entry["name"]
(out / "icp").mkdir(parents=True, exist_ok=True)
for old in (out / "icp").glob("*.b64"):
    old.unlink()
started = time.monotonic()


def log(*a):
    print(f"[{entry['name']} {time.monotonic() - started:5.1f}s]", *a, flush=True)


def write_b64(path, data: bytes):
    path.write_bytes(base64.b64encode(data))
    return path.name


def yaw_of(rotation):
    return rotation.as_euler("zyx")[:, 0]


def read_json(path):
    return json.loads(path.read_text()) if path.exists() else None


# ---- methods: keyframes + graph states ----
methods = {}
for key in ("gsc", "ivan"):
    run = entry.get(key)
    if not run:
        continue
    run = Path(run).expanduser()
    kf = np.fromfile(run / "a.kf", rm.KEYFRAME)
    states = rm.read_closures(run / "a.kf.closures") if (run / "a.kf.closures").exists() else []
    methods[key] = dict(run=run, kf=kf, states=states)
    log(key, run.name, len(kf), "keyframes", len(states), "states")
t0 = min(float(m["kf"]["ts"][0]) for m in methods.values())
t1 = max(float(m["kf"]["ts"][-1]) for m in methods.values())

# ---- GT ----
gt_found = correction(entry.get("gt"))
log("gt:", gt_found[1] if gt_found else "none")

# ---- per-keyframe points from the world db ----
with SqliteStore(path=str(Path(entry["world_db"]).expanduser())) as store:
    lidar = store.streams.lidar
    scan_ts = np.array([o.ts for o in lidar.order_by("ts")])
    odom_rows = []
    for o in store.streams.odom.order_by("ts"):
        p = o.data.pose if hasattr(o.data, "pose") and hasattr(o.data.pose, "position") else o.data
        odom_rows.append(
            (
                o.ts,
                p.position.x,
                p.position.y,
                p.position.z,
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w,
            )
        )
    odom = np.array(odom_rows)
    odom = odom[np.unique(odom[:, 0], return_index=True)[1]]

    # jobs: (window start, window end, method, keyframe index), run once the window's last scan is decoded
    half = opts["scans_per_keyframe"] // 2
    jobs = []
    for key, m in methods.items():
        m["points"] = [None] * len(m["kf"])
        centers = np.clip(np.searchsorted(scan_ts, m["kf"]["ts"]), 0, len(scan_ts) - 1)
        for k, c in enumerate(centers.tolist()):
            jobs.append((max(0, c - half), min(len(scan_ts) - 1, c + half), key, k))
    jobs.sort(key=lambda j: j[1])
    wanted = {i for s, e, _, _ in jobs for i in range(s, e + 1)}
    buffer, next_job = {}, 0
    voxel, band = opts["voxel_m"], opts["band_m"]

    def local_points(key, k, start, end):
        raw = methods[key]["kf"]["raw"][k]
        world = np.concatenate(
            [buffer[i] for i in range(start, end + 1) if i in buffer] or [np.zeros((0, 3))]
        )
        local = R.from_quat(raw[3:]).inv().apply(world - raw[:3]) if len(world) else world
        horizontal = np.linalg.norm(local[:, :2], axis=1)
        local = local[horizontal < opts["max_range_m"]]
        near = local[np.linalg.norm(local[:, :2], axis=1) < 5.0]
        if len(near) > 50:  # drop floor and ceiling: keep a height band above the local floor
            floor = np.percentile(near[:, 2], 5)
            local = local[(local[:, 2] > floor + band[0]) & (local[:, 2] < floor + band[1])]
        cells = np.floor(local[:, :2] / voxel).astype(np.int64)
        _, first = np.unique(cells, axis=0, return_index=True)
        xy = (np.floor(local[first, :2] / voxel) + 0.5) * voxel
        if len(xy) > opts["max_points"]:
            xy = xy[np.random.default_rng(k).choice(len(xy), opts["max_points"], replace=False)]
        return np.round(xy * 100).astype("<i2")

    for i, o in enumerate(lidar.order_by("ts")):
        if i in wanted:
            buffer[i] = np.asarray(o.data.points_f32(), np.float64)
        while next_job < len(jobs) and jobs[next_job][1] <= i:
            s, e, key, k = jobs[next_job]
            methods[key]["points"][k] = local_points(key, k, s, e)
            next_job += 1
        low = jobs[next_job][0] if next_job < len(jobs) else i + 1
        for j in [j for j in buffer if j < low]:
            del buffer[j]
    log("points done,", len(scan_ts), "scans")

# ---- write per method ----
meta_methods = {}
for key, m in methods.items():
    kf = m["kf"]
    raw_rotation = R.from_quat(kf["raw"][:, 3:])
    # points: u32 count per keyframe, then int16 x, y (cm, keyframe body frame) for all keyframes in order
    counts = np.array([len(p) for p in m["points"]], "<u4")
    points_file = write_b64(
        out / f"{key}_points.b64", counts.tobytes() + b"".join(p.tobytes() for p in m["points"])
    )
    # states: per state f64 ts (s since t0), u32 n, then n x int16 (x / xy_scale, y / xy_scale, yaw / pi * 32767), ids 0..n-1
    poses = []
    for ts, rec in m["states"]:
        order = np.argsort(rec["id"])
        rec = rec[order]
        delta = R.from_quat(rec["dq"].astype(float))
        xyz = delta.apply(rec["xyz"].astype(float)) + rec["dt"]
        poses.append(
            (
                ts,
                rec["id"].astype(int),
                xyz[:, :2],
                yaw_of(delta * R.from_quat(rec["q"].astype(float))),
            )
        )
    extent = max(
        [float(np.abs(kf["raw"][:, :2]).max()), float(np.abs(kf["corrected"][:, :2]).max())]
        + [float(np.abs(p[2]).max()) for p in poses]
    )
    xy_scale = max(0.01, extent / 32000)
    total = sum(12 + 6 * len(p[1]) for p in poses)
    stride = max(1, int(np.ceil(total / opts["state_budget_bytes"])))
    keep = sorted(set(range(0, len(poses), stride)) | ({len(poses) - 1} if poses else set()))
    chunks = []
    for s in keep:
        ts, ids, xy, yaw = poses[s]
        assert (ids == np.arange(len(ids))).all(), "state ids must be 0..n-1"
        body = np.stack(
            [
                np.round(xy[:, 0] / xy_scale),
                np.round(xy[:, 1] / xy_scale),
                np.round(yaw / np.pi * 32767),
            ],
            1,
        )
        chunks += [
            struct.pack("<dI", ts - t0, len(ids)),
            np.clip(body, -32767, 32767).astype("<i2").tobytes(),
        ]
    states_file = write_b64(out / f"{key}_states.b64", b"".join(chunks))
    gt, gt_description = (
        gt_poses(None, kf["ts"], kf["raw"], found=gt_found) if gt_found else (None, None)
    )
    final_rotation = R.from_quat(kf["corrected"][:, 3:])
    meta_methods[key] = dict(
        run=m["run"].name,
        keyframes=len(kf),
        states_total=len(poses),
        states_kept=len(keep),
        xy_scale=xy_scale,
        points=points_file,
        states=states_file,
        kf_ts=np.round(kf["ts"] - t0, 3).tolist(),
        raw=np.round(
            np.stack([kf["raw"][:, 0], kf["raw"][:, 1], yaw_of(raw_rotation)], 1), 3
        ).tolist(),
        final=np.round(
            np.stack([kf["corrected"][:, 0], kf["corrected"][:, 1], yaw_of(final_rotation)], 1), 3
        ).tolist(),
        gt=None
        if gt is None
        else [
            None if np.isnan(g[0]) else [round(g[0], 3), round(g[1], 3), round(float(y), 3)]
            for g, y in zip(
                gt.tolist(),
                yaw_of(R.from_quat([*np.nan_to_num(gt[:, 3:], nan=0.0), 0, 0, 0, 1e-09])),
                strict=False,
            )
        ],
    )
    log(key, f"{counts.sum()} points, states {len(keep)}/{len(poses)} kept (stride {stride})")

# ---- GSC loop edges, gate diagnostics, ICP clouds ----
gsc_meta = {}
if "gsc" in methods:
    run = methods["gsc"]["run"]
    a = read_json(run / "a.json") or {}
    edges = []
    if (run / "a.kf.edges").exists():
        records = np.fromfile(run / "a.kf.edges", EDGE)
        pairs = {}
        for r in records:
            p = pairs.setdefault(
                (int(r["source_id"]), int(r["target_id"])),
                dict(first=float(r["ts"]), applied=None, last=0, conf=0.0),
            )
            p["last"] = int(r["state"])
            p["conf"] = max(p["conf"], float(r["confidence"]))
            if r["state"] == 6 and p["applied"] is None:
                p["applied"] = float(r["ts"])
        # [source, target, ts shown (applied, else first event), state: 6 applied / 5 GNC-rejected / other last state]
        for (s, t), p in pairs.items():
            if p["applied"] is not None or p["last"] in (2, 3, 4, 5):
                edges.append(
                    [
                        s,
                        t,
                        round((p["applied"] or p["first"]) - t0, 2),
                        6 if p["applied"] is not None else p["last"],
                        round(p["conf"], 3),
                    ]
                )
    else:
        ts_of = methods["gsc"]["kf"]["ts"]
        edges = [
            [e["source_id"], e["target_id"], round(float(ts_of[e["source_id"]]) - t0, 2), 6, 1.0]
            for e in a.get("committed_closure_edges", [])
        ]
    config = a.get("config", {})
    diag = a.get("closure_diagnostics", [])
    rows = []
    for d in diag:
        row = [d.get(c) for c in DIAG_COLUMNS]
        row[3] = None if d.get("source_time") is None else round(d["source_time"] - t0, 2)
        rows.append([round(v, 4) if isinstance(v, float) else v for v in row])
    thresholds = dict(
        icp_fitness=config.get("closure_score_thresh"),
        degeneracy_min=config.get("closure_min_degeneracy"),
    )

    def margin(d):
        """How far past its gate a rejected candidate was (0 = on the threshold)."""
        try:
            if d["outcome"] == "icp_reject":
                return abs(d["icp_fitness"] / thresholds["icp_fitness"] - 1)
            if d["outcome"] == "degeneracy_reject":
                return abs(1 - d["degeneracy_min"] / thresholds["degeneracy_min"])
            if d["outcome"] == "yank_confidence_reject":
                return abs(1 - d["confidence"] / d["required_confidence"])
        except (TypeError, ZeroDivisionError):
            pass
        return 1e9

    icp_dir = Path(config.get("closure_icp_dump_dir") or run / "icp")
    if not icp_dir.exists():
        icp_dir = run / "icp"
    picks = []
    if icp_dir.exists():
        accepted = [i for i, d in enumerate(diag) if d["outcome"] == "accepted"]
        if len(accepted) > opts["icp_accepted"]:
            accepted = [
                accepted[int(j)] for j in np.linspace(0, len(accepted) - 1, opts["icp_accepted"])
            ]
        by_outcome = {
            o: sorted(
                (i for i, d in enumerate(diag) if d["outcome"] == o), key=lambda i: margin(diag[i])
            )
            for o in REJECTS
        }
        rejected = []
        while len(rejected) < opts["icp_rejected"] and any(by_outcome.values()):
            for o in REJECTS:
                if by_outcome[o] and len(rejected) < opts["icp_rejected"]:
                    rejected.append(by_outcome[o].pop(0))
        picks = accepted + rejected

    def read_icp(path):
        raw = path.read_bytes()
        ns, nt = (int(v) for v in np.frombuffer(raw, "<u4", 2))
        points = np.frombuffer(raw, "<f4", 3 * (ns + nt), 8).reshape(-1, 3)
        tail = np.frombuffer(raw, "<f8", 13, 8 + 12 * (ns + nt))
        return points[:ns], points[ns:], tail

    def pack_cloud(points, limit, seed):
        """f32 offset xyz, u32 n, int16 cm xyz relative to the offset."""
        if len(points) > limit:
            points = points[np.random.default_rng(seed).choice(len(points), limit, replace=False)]
        offset = points.mean(0) if len(points) else np.zeros(3)
        rel = np.clip(np.round((points - offset) * 100), -32767, 32767).astype("<i2")
        return struct.pack("<3fI", *offset, len(points)) + rel.tobytes()

    icp_index = {}
    for i in picks:
        d = diag[i]
        stem = f"{d['source_id']}_{d['target_id']}"
        stages = [
            (1, icp_dir / f"{stem}.icp"),
            *sorted(
                ((int(p.stem.rsplit("_x", 1)[1]), p) for p in icp_dir.glob(f"{stem}_x*.icp")),
                key=lambda s: s[0],
            ),
        ]
        stages = [s for s in stages if s[1].exists()]
        if not stages:
            continue
        info, blobs = [], []
        largest = stages[-1][0]
        for keyframes, path in stages:
            source, target, tail = read_icp(path)
            entry_info = dict(
                keyframes=keyframes,
                source_points=len(source),
                target_points=len(target),
                fitness=round(float(tail[12]), 4),
                rotation_deg=round(
                    float(np.degrees(R.from_matrix(tail[:9].reshape(3, 3)).magnitude())), 2
                ),
                translation_m=round(float(np.linalg.norm(tail[9:12])), 3),
            )
            info.append(entry_info)
            # clouds only for the single round and the largest extended round; the rest keep their numbers
            if keyframes == 1 or keyframes == largest:
                limits = opts["icp_single"] if keyframes == 1 else opts["icp_extended"]
                # stage: u32 keyframes, f64 rotation (9, row-major) + translation (3) + fitness, source cloud, target cloud
                blobs.append(
                    struct.pack("<I", keyframes)
                    + tail.astype("<f8").tobytes()
                    + pack_cloud(source, limits[0], i)
                    + pack_cloud(target, limits[1], i + 1)
                )
        icp_index[str(i)] = dict(
            file="icp/"
            + write_b64(
                out / "icp" / f"{stem}.b64", struct.pack("<I", len(blobs)) + b"".join(blobs)
            ),
            stages=info,
        )
    log(f"diag {len(diag)} candidates, icp clouds for {len(icp_index)}")
    gsc_meta = dict(
        edges=edges,
        diag_columns=DIAG_COLUMNS,
        diag=rows,
        thresholds=thresholds,
        icp=icp_index,
        gate_counts=a.get("gate_counts"),
        runtime_s=a.get("runtime_s"),
        realtime_factor=a.get("realtime_factor"),
        committed=len(a.get("committed_closure_edges", [])),
        data_duration_s=a.get("data_duration_s"),
    )
    meta_methods["gsc"].update(edges=edges)

# ---- summary: existing score.json / tags.json, computed if missing and --score requested ----
summary = {}
for key, m in methods.items():
    s = dict(keyframes=len(m["kf"]), states=len(m["states"]))
    for name in ("score", "tags"):
        path = m["run"] / f"{name}.json"
        if not path.exists() and entry.get("score_args", {}).get(name):
            script = SCRIPTS / {"score": "score_run.py", "tags": "tag_agreement.py"}[name]
            cmd = [
                "nice",
                "-n",
                "10",
                str(WORKTREE / ".venv/bin/python"),
                str(script),
                str(m["run"]),
                *entry["score_args"][name],
            ]
            try:
                subprocess.run(
                    cmd,
                    cwd=WORKTREE,
                    timeout=180,
                    check=True,
                    capture_output=True,
                    env={
                        **os.environ,
                        "PYTHONPATH": os.environ.get(
                            "GSC_SCORE_STUBS", str(Path.home() / ".cache/gsc_score_stubs")
                        ),
                    },
                )
            except Exception as e:
                log(f"{key} {name} failed: {e}")
        data = read_json(path)
        if data and name == "score":
            s["score"] = {k: v for k, v in data.items()}
        elif data:
            s["tag_spread_m"] = {k: v.get("mean_spread_m") for k, v in data.items()}
    if key == "ivan":
        s.update(
            {
                k: v
                for k, v in (read_json(m["run"] / "ivan.json") or {}).items()
                if k in ("loops", "dropped", "frames", "busy_s", "wall_s")
            }
        )
    summary[key] = s
if gsc_meta:
    summary["gsc"].update(
        loops=gsc_meta["committed"],
        gate_counts=gsc_meta["gate_counts"],
        runtime_s=gsc_meta["runtime_s"],
    )

# robot odometry (raw, world db frame), decimated
step = max(1, round(len(odom) / max(1.0, (odom[-1, 0] - odom[0, 0]) * opts["odom_hz"])))
od = odom[::step]
odom_out = np.round(
    np.stack([od[:, 0] - t0, od[:, 1], od[:, 2], yaw_of(R.from_quat(od[:, 4:]))], 1), 3
).tolist()
gt_odom = None
if gt_found:
    g, _ = gt_poses(None, od[:, 0], od[:, 1:], found=gt_found)
    gt_odom = [
        None if np.isnan(r[0]) else [round(r[0], 3), round(r[1], 3), round(float(y), 3)]
        for r, y in zip(
            g.tolist(),
            yaw_of(R.from_quat([*np.nan_to_num(g[:, 3:], nan=0.0), 0, 0, 0, 1e-09])),
            strict=False,
        )
    ]

plan = {}
for key, path in (entry.get("plan") or {}).items():
    path = Path(path).expanduser()
    if path.exists():
        (out / f"{key}_plan.json").write_text(path.read_text())
        plan[key] = f"{key}_plan.json"

meta = dict(
    name=entry["name"],
    label=entry.get("label", entry["name"]),
    t0=t0,
    duration_s=round(t1 - t0, 2),
    gt=gt_found[1] if gt_found else None,
    odom=odom_out,
    gt_odom=gt_odom,
    methods=meta_methods,
    summary=summary,
    loops=gsc_meta or None,
    plan=plan,
    options=opts,
)
(out / "meta.json").write_text(json.dumps(meta, separators=(",", ":")))
sizes = {p.name: p.stat().st_size for p in out.glob("*") if p.is_file()}
icp_bytes = sum(p.stat().st_size for p in (out / "icp").glob("*"))
log("wrote", {k: f"{v / 1e6:.2f} MB" for k, v in sizes.items()}, f"icp {icp_bytes / 1e6:.2f} MB")
