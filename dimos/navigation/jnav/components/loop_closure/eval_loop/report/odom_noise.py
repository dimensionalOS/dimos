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

"""Odometry drift per keyframe step against ground truth: relative pose error of raw odometry vs GT over k-keyframe
spans, variance / k (a random walk's per-step variance), per axis in the start keyframe's frame.
usage: odom_noise.py <manifest.json> <recording> <run dir>"""

import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).parent))
import gt_poses

KEYFRAME = np.dtype([("ts", "<f8"), ("raw", "<f8", 7), ("corrected", "<f8", 7)])
manifest = json.loads(Path(sys.argv[1]).expanduser().read_text())
entry = next(r for r in manifest["recordings"] if r["name"] == sys.argv[2])
kf = np.fromfile(Path(sys.argv[3]).expanduser() / "a.kf", KEYFRAME)
gt, description = gt_poses.gt_poses(entry["gt"], kf["ts"], kf["raw"])
ok = np.isfinite(gt[:, 0])
raw, gt = kf["raw"][ok], gt[ok]
raw_rotation, gt_rotation = R.from_quat(raw[:, 3:]), R.from_quat(gt[:, 3:])
out = {"recording": sys.argv[2], "gt": description}
for k in (10, 50, 200):
    i = np.arange(0, len(raw) - k, max(1, k // 5))
    if len(i) < 5:
        continue
    raw_relative = raw_rotation[i].inv() * raw_rotation[i + k]
    gt_relative = gt_rotation[i].inv() * gt_rotation[i + k]
    rotation_error = (raw_relative.inv() * gt_relative).as_rotvec()
    translation_error = raw_rotation[i].inv().apply(raw[i + k, :3] - raw[i, :3]) - gt_rotation[
        i
    ].inv().apply(gt[i + k, :3] - gt[i, :3])
    out[f"k{k}"] = dict(
        roll_pitch=float(np.mean(np.var(rotation_error[:, :2], axis=0)) / k),
        yaw=float(np.var(rotation_error[:, 2]) / k),
        xy=float(np.mean(np.var(translation_error[:, :2], axis=0)) / k),
        z=float(np.var(translation_error[:, 2]) / k),
    )
print(json.dumps(out))
