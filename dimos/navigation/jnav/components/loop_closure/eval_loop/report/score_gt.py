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

"""Keyframe position error of a run against ground truth (the manifest entry's `gt` specs, read by gt_poses.py).
final = where the finished graph puts each keyframe; live = where the method had it when it was made (the newest
correction then, applied to its raw pose). One rigid fit of the final graph onto GT is applied to final, live and raw
alike, so all three are judged in one frame.
usage: score_gt.py <manifest.json> <recording name> <run dir> [<run dir> ...]   -> one JSON line per run"""

import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).parent))
import gt_poses

KEYFRAME = np.dtype([("ts", "<f8"), ("raw", "<f8", 7), ("corrected", "<f8", 7)])
CLOSURE_RECORD = np.dtype(
    [
        ("xyz", "<f4", 3),
        ("id", "<u4"),
        ("ts", "<f8"),
        ("q", "<f4", 4),
        ("dt", "<f4", 3),
        ("dq", "<f4", 4),
    ]
)


def read_states(path):
    data, offset, states = path.read_bytes(), 0, []
    while offset + 12 <= len(data):
        ts = np.frombuffer(data, "<f8", 1, offset)[0]
        count = int(np.frombuffer(data, "<u4", 1, offset + 8)[0])
        offset += 12
        states.append((float(ts), np.frombuffer(data, CLOSURE_RECORD, count, offset)))
        offset += count * CLOSURE_RECORD.itemsize
    return states


def live_positions(run, kf):
    live = kf["raw"][:, :3].copy()
    raw_rotation = R.from_quat(kf["raw"][:, 3:])
    for ts, rec in read_states(run / "a.kf.closures"):
        dq = R.from_quat(rec["dq"].astype(float))
        corrected_xyz = dq.apply(rec["xyz"].astype(float)) + rec["dt"]
        corrected_rotation = dq * R.from_quat(rec["q"].astype(float))
        newest = int(np.argmax(rec["id"]))
        j = int(rec["id"][newest])
        rotation = corrected_rotation[newest] * raw_rotation[j].inv()
        translation = corrected_xyz[newest] - rotation.apply(kf["raw"][j, :3])
        later = kf["ts"] > ts
        live[later] = rotation.apply(kf["raw"][later, :3]) + translation
    return live


def fit(estimate, target):
    a_mean, b_mean = estimate.mean(0), target.mean(0)
    u, _, vt = np.linalg.svd((estimate - a_mean).T @ (target - b_mean))
    rotation = (u @ np.diag([1, 1, np.sign(np.linalg.det(u @ vt))]) @ vt).T
    return lambda x: (rotation @ (x - a_mean).T).T + b_mean


def error(estimate, truth):
    e = np.linalg.norm(estimate - truth, axis=1)
    return dict(median_m=round(float(np.median(e)), 3), p95_m=round(float(np.percentile(e, 95)), 3))


manifest = json.loads(Path(sys.argv[1]).expanduser().read_text())
entry = next(r for r in manifest["recordings"] if r["name"] == sys.argv[2])
found = gt_poses.correction(entry.get("gt"))
for run in map(Path, sys.argv[3:]):
    run = run.expanduser()
    kf = np.fromfile(run / "a.kf", KEYFRAME)
    if found is None:
        print(json.dumps(dict(run=run.name, gt=None)))
        continue
    gt, description = gt_poses.gt_poses(entry.get("gt"), kf["ts"], kf["raw"], found)
    ok = np.isfinite(gt[:, 0])
    truth = gt[ok, :3]
    align = fit(kf["corrected"][ok, :3], truth)
    final, live, raw = (
        error(align(x[ok]), truth)
        for x in (kf["corrected"][:, :3], live_positions(run, kf), kf["raw"][:, :3])
    )
    print(json.dumps(dict(run=run.name, gt=description, final=final, live=live, raw=raw)))
