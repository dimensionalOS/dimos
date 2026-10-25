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

"""Keyframe position error of a gsc_run run against ground truth, after one rigid fit (the finished graph's, applied to all three):
final = each keyframe where the finished graph puts it; live = each keyframe where GSC had it the moment it was made
(the newest correction before then applied to its raw pose, which is what the robot navigated with).
usage: score_run.py <run dir> <gt db> <gt stream>   (run from the dimos worktree) -> prints and writes <run dir>/score.json"""

import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, "dimos/navigation/global_planner/mls_planner/utils")
import reform_metrics as rm

from dimos.memory.store.sqlite import SqliteStore

run, gt_db, gt_stream = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
with SqliteStore(path=gt_db) as store:
    rows = []
    for o in getattr(store.streams, gt_stream).order_by("ts"):
        p = o.data.pose if hasattr(o.data, "pose") and hasattr(o.data.pose, "position") else o.data
        rows.append((o.ts, p.position.x, p.position.y, p.position.z))
gt = np.array(rows)
kf = np.fromfile(run / "a.kf", rm.KEYFRAME)

live = kf["raw"][:, :3].copy()
raw_rot = R.from_quat(kf["raw"][:, 3:])
for ts, rec in rm.read_closures(run / "a.kf.closures"):
    dq = R.from_quat(rec["dq"].astype(float))
    corrected_xyz = dq.apply(rec["xyz"].astype(float)) + rec["dt"]
    corrected_rot = dq * R.from_quat(rec["q"].astype(float))
    newest = int(np.argmax(rec["id"]))
    j = int(rec["id"][newest])
    # correction = corrected_j * raw_j^-1, applied to every keyframe made after this state until the next one
    correction_rot = corrected_rot[newest] * raw_rot[j].inv()
    correction_t = corrected_xyz[newest] - correction_rot.apply(kf["raw"][j, :3])
    later = kf["ts"] > ts
    live[later] = correction_rot.apply(kf["raw"][later, :3]) + correction_t


def aligner(estimate, target):
    a_mean, b_mean = estimate.mean(0), target.mean(0)
    u, _, vt = np.linalg.svd((estimate - a_mean).T @ (target - b_mean))
    rotation = (u @ np.diag([1, 1, np.sign(np.linalg.det(u @ vt))]) @ vt).T
    return lambda x: (rotation @ (x - a_mean).T).T + b_mean


def error(estimate, truth):
    err = np.linalg.norm(align(estimate) - truth, axis=1)
    return dict(
        median_m=round(float(np.median(err)), 3), p95_m=round(float(np.percentile(err, 95)), 3)
    )


inside = (kf["ts"] >= gt[0, 0]) & (kf["ts"] <= gt[-1, 0])
truth = gt[np.clip(np.searchsorted(gt[:, 0], kf["ts"][inside]), 0, len(gt) - 1), 1:4]
# one rigid fit (the finished graph's) for all three, so live and raw are judged in the same frame, not each its own best one
align = aligner(kf["corrected"][inside, :3], truth)
out = dict(
    final=error(kf["corrected"][inside, :3], truth),
    live=error(live[inside], truth),
    raw=error(kf["raw"][inside, :3], truth),
)
(run / "score.json").write_text(json.dumps(out, indent=1))
print(run.name, json.dumps(out))
