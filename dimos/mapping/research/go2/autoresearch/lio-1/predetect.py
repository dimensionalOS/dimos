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

"""Preprocess a recording for eval3d. The expensive, trajectory-INDEPENDENT work —
reading the mcap, decoding video, detecting AprilTags + solvePnP, and the robot_odom
horizontal-path / lidar-clock constants — is done ONCE here and cached, so the eval
loop never re-reads the mcap or runs cv2 (eval drops from seconds/minutes to ~ms).

Writes predetect/<dataset>.json:
  { first_lidar_pub_ns, gt_xy_path_m, detections: [{t_ns, id, tvec}] }

Run once per dataset:
  python predetect.py --mcap <recording.mcap>
"""

import argparse
import json
import os
import time

import eval3d as E


def main():
    ap = argparse.ArgumentParser(description="pre-detect AprilTags + cache constants for eval3d")
    ap.add_argument("--mcap", required=True)
    ap.add_argument("--out", help="default: predetect/<dataset>.json next to eval3d.py")
    a = ap.parse_args()
    out = a.out or E._predetect_path(a.mcap)
    if out is None:
        raise SystemExit("unknown dataset (mcap path has no known go2dds_dataN key); pass --out")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    t0 = time.time()
    _, rt_pos, _, flp = E.load_robot_odom(a.mcap)  # per-dataset constants (gt xy path + clock)
    gt_xy = E._path_len(rt_pos[:, :2])
    t1 = time.time()
    dets = E._detect_tags(a.mcap)  # the expensive video + cv2 pass
    t2 = time.time()

    json.dump(
        {
            "first_lidar_pub_ns": int(flp),
            "gt_xy_path_m": float(gt_xy),
            "detections": [{"t_ns": t, "id": i, "tvec": v} for t, i, v in dets],
        },
        open(out, "w"),
    )
    print(f"odom pass {t1 - t0:.1f}s, detect pass {t2 - t1:.1f}s; {len(dets)} detections")
    print(f"wrote {out}  ({os.path.getsize(out)} bytes)")


if __name__ == "__main__":
    main()
