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

"""For each example, re-solve the pose graph as it stood when the loop was proposed, with the loop held almost rigid,
so the page can show what a fully trusted closure would do. Odometry and earlier applied loops keep GSC's noise."""

import json
from pathlib import Path
import sys

import gtsam
import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, "dimos/navigation/global_planner/mls_planner/utils")
import itertools

import reform_metrics as rm

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
APPLIED = 6


def pose(rot, xyz):
    return gtsam.Pose3(gtsam.Rot3(rot.as_matrix()), gtsam.Point3(*xyz))


# usage: strong.py <run dir> <examples.json> [<run dir> <examples.json> ...]
for run, path in zip(map(Path, sys.argv[1::2]), map(Path, sys.argv[2::2]), strict=False):
    name = path.stem.removeprefix("loop_examples_")
    J = json.loads((run / "a.json").read_text())
    cfg = J["config"]
    states = []
    for ts, rec in rm.read_closures(run / "a.kf.closures"):
        dq = R.from_quat(rec["dq"].astype(float))
        states.append(
            (
                ts,
                list(rec["id"].astype(int)),
                dq.apply(rec["xyz"].astype(float)) + rec["dt"],
                dq * R.from_quat(rec["q"].astype(float)),
            )
        )
    state_ts = np.array([s[0] for s in states])
    edges = np.fromfile(run / "a.kf.edges", EDGE)
    applied_ts = {}
    for e in edges[edges["state"] == APPLIED]:
        applied_ts.setdefault((int(e["source_id"]), int(e["target_id"])), float(e["ts"]))
    committed = {
        (c["source_id"], c["target_id"]): c
        for c in J.get("committed_closure_edges", J.get("committed_loop_edges"))
    }
    proposed = {
        (d["source_id"], d["target_id"]): d["source_time"]
        for d in J.get("closure_diagnostics", J.get("loop_diagnostics"))
    }
    odom_noise = gtsam.noiseModel.Diagonal.Variances(
        np.array(
            [cfg["odom_rot_roll_pitch_var"]] * 2
            + [cfg["odom_rot_yaw_var"]]
            + [cfg["odom_trans_xy_var"]] * 2
            + [cfg["odom_trans_z_var"]]
        )
    )
    rigid = gtsam.noiseModel.Isotropic.Sigma(6, 1e-4)

    data = json.loads(path.read_text())
    for ex in data["examples"]:
        a, b = ex["source_id"], ex["target_id"]
        t = proposed[(a, b)]
        _, ids, xyz, rot = states[int(np.searchsorted(state_ts, t, side="right")) - 1]
        index = {k: i for i, k in enumerate(ids)}
        keep = sorted(k for k in ids if k <= a)
        graph, values = gtsam.NonlinearFactorGraph(), gtsam.Values()
        P = {k: pose(rot[index[k]], xyz[index[k]]) for k in keep}
        for k in keep:
            values.insert(k, P[k])
        graph.add(
            gtsam.PriorFactorPose3(keep[0], P[keep[0]], gtsam.noiseModel.Isotropic.Sigma(6, 1e-6))
        )
        for i, j in itertools.pairwise(keep):
            graph.add(gtsam.BetweenFactorPose3(i, j, P[i].between(P[j]), odom_noise))
        # loops already applied when this one was proposed: held at where the graph had them, with GSC's noise
        earlier = 0
        for (s, tg), c in committed.items():
            if (s, tg) != (a, b) and applied_ts.get((s, tg), np.inf) <= t and s in P and tg in P:
                noise = gtsam.noiseModel.Robust.Create(
                    gtsam.noiseModel.mEstimator.Huber.Create(
                        cfg.get("closure_robust_huber_k", cfg.get("loop_robust_huber_k"))
                    ),
                    gtsam.noiseModel.Isotropic.Variance(6, c["score"]),
                )
                graph.add(gtsam.BetweenFactorPose3(tg, s, P[tg].between(P[s]), noise))
                earlier += 1
        c = committed[(a, b)]
        measured = gtsam.Pose3(gtsam.Rot3(np.array(c["rotation"])), gtsam.Point3(*c["translation"]))
        graph.add(gtsam.BetweenFactorPose3(b, a, measured, rigid))
        out = gtsam.LevenbergMarquardtOptimizer(graph, values).optimize()
        step = max(1, len(keep) // 900)
        strong = np.array([out.atPose3(k).translation() for k in keep])
        ex["path_strong"] = np.round(strong[::step, :2], 3).tolist()
        tb = out.atPose3(b)
        ring = tb.compose(measured).translation()
        ex["icp_source_xy_strong"] = np.round(ring[:2], 3).tolist()
        ex["gap_strong_m"] = round(float(np.linalg.norm(ring - out.atPose3(a).translation())), 3)
        ex["strong_earlier_loops"] = earlier
        assert step == ex.get("path_step", step) and len(ex["path_strong"]) == len(
            ex["path_before"]
        ), (len(ex["path_strong"]), len(ex["path_before"]))
        print(
            name,
            a,
            b,
            "earlier loops",
            earlier,
            "gap",
            ex["gap_before_m"],
            "-> strong",
            ex["gap_strong_m"],
            "max move {:.2f} m".format(
                np.max(
                    np.linalg.norm(
                        strong[:, :2] - np.array([xyz[index[k]][:2] for k in keep]), axis=1
                    )
                )
            ),
            flush=True,
        )
    path.write_text(json.dumps(data))
