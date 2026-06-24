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

# Untyped analysis script: gtsam/open3d/cv2 lack type stubs.
# mypy: ignore-errors
"""Combined comparison rrd: raw lidar cloud + EVERY gt_*_lidar version present in the db, each as
its own colored entity, plus AprilTag landmarks + trajectories. Re-run after adding a new GT method
and it picks the new stream up automatically.

Importable: `build(...)` writes the rrd and returns its path (used by post_process.py).
Standalone: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/make_rrd.py --rec=PATH [--lidar=...] [--odom=...] [--tags=...] [--out=...]
"""

import json
from pathlib import Path
import sys

from gtsam import Point3, Pose3, Rot3
import numpy as np
import rerun as rr

from dimos.navigation.jnav.utils import recording_db as rdb

SCAN_STRIDE, VOXEL = 8, 0.10
COLORS = {"raw": [220, 60, 60]}
PALETTE = [
    [60, 120, 230],
    [60, 210, 90],
    [230, 180, 50],
    [200, 80, 220],
    [80, 220, 220],
    [240, 130, 60],
]
# same relaxed gates post_process uses, for placing landmark markers
GATE = dict(s=25.0, r=3.5, px=12.0, d=1.5, a=65.0, lv=1.5, av=150.0)


def build(
    rec,
    lidar_stream="pointlio_lidar",
    odom_stream="pointlio_odometry",
    tag_stream="raw_april_tags",
    out_name="gt_compare.rrd",
):
    rec = Path(rec).expanduser()
    db = rec / "mem2.db"
    out = rec / out_name
    st = rdb.store(db)
    intr = json.loads((rec / "camera_intrinsics.json").read_text())
    ext = np.array(intr["optical_in_base"], float)
    Tbo = Pose3(Rot3.Quaternion(ext[6], ext[3], ext[4], ext[5]), Point3(ext[0], ext[1], ext[2]))

    def accumulate(name):
        pts = []
        for i, o in enumerate(st.stream(name)):
            if i % SCAN_STRIDE:
                continue
            xyz = np.asarray(o.data.points_f32())
            if len(xyz):
                pts.append(xyz[::3])
        a = np.concatenate(pts, 0)
        _, idx = np.unique(np.floor(a / VOXEL).astype(np.int64), axis=0, return_index=True)
        return a[idx]

    def traj(name):
        return np.array(
            [
                [o.data.pose.position.x, o.data.pose.position.y, o.data.pose.position.z]
                for o in st.stream(name)
            ],
            np.float32,
        )

    def landmarks(gt_odom):
        gt = [
            (
                o.ts,
                Pose3(
                    Rot3.Quaternion(
                        o.data.pose.orientation.w,
                        o.data.pose.orientation.x,
                        o.data.pose.orientation.y,
                        o.data.pose.orientation.z,
                    ),
                    Point3(o.data.pose.position.x, o.data.pose.position.y, o.data.pose.position.z),
                ),
            )
            for o in st.stream(gt_odom)
        ]
        gts = np.array([t for t, _ in gt])
        pos = {}
        for obs in st.stream(tag_stream):
            t = obs.tags
            if not (
                float(t["sharpness"]) >= GATE["s"]
                and float(t["reproj_px"]) <= GATE["r"]
                and float(t["tag_px"]) >= GATE["px"]
                and float(t["distance_m"]) <= GATE["d"]
                and float(t["view_angle_deg"]) <= GATE["a"]
                and (float(t["lin_speed"]) < 0 or float(t["lin_speed"]) <= GATE["lv"])
                and (float(t["ang_speed"]) < 0 or float(t["ang_speed"]) <= GATE["av"])
            ):
                continue
            ps = obs.data
            cb = gt[int(np.argmin(np.abs(gts - float(obs.ts))))][1]
            Tw = cb.compose(Tbo).compose(
                Pose3(
                    Rot3.Quaternion(
                        ps.orientation.w, ps.orientation.x, ps.orientation.y, ps.orientation.z
                    ),
                    Point3(ps.x, ps.y, ps.z),
                )
            )
            pos.setdefault(int(t["marker_id"]), []).append(np.asarray(Tw.translation()))
        means = [np.mean(value, 0) for key, value in sorted(pos.items())]
        lbl = [f"tag{key}" for key in sorted(pos)]
        return np.array(means), lbl

    streams = st.list_streams()
    gt_lidars = sorted(s for s in streams if s.startswith("gt_") and "_lidar" in s)
    print("raw + GT lidar streams:", gt_lidars)

    rr.init("gt_compare")
    rr.save(str(out))
    rr.log(
        "raw/cloud",
        rr.Points3D(accumulate(lidar_stream), colors=COLORS["raw"], radii=0.02),
        static=True,
    )
    rr.log(
        "raw/trajectory", rr.LineStrips3D([traj(odom_stream)], colors=[255, 120, 120]), static=True
    )
    for k, name in enumerate(gt_lidars):
        color = PALETTE[k % len(PALETTE)]
        cloud = accumulate(name)
        rr.log(f"{name}/cloud", rr.Points3D(cloud, colors=color, radii=0.02), static=True)
        print(f"  logged {name}: {len(cloud):,} pts")
        odom = name.replace("_lidar", "_odometry")
        if odom in streams:
            rr.log(f"{name}/trajectory", rr.LineStrips3D([traj(odom)], colors=color), static=True)
    # landmarks placed against the first available gt odometry
    gt_odoms = sorted(s for s in streams if s.startswith("gt_") and "_odometry" in s)
    if gt_odoms:
        lm, lbl = landmarks(gt_odoms[0])
        if len(lm):
            rr.log(
                "landmarks",
                rr.Points3D(lm, colors=[255, 230, 0], radii=0.25, labels=lbl),
                static=True,
            )
            print(f"  logged {len(lbl)} landmarks")
    print("wrote", out)
    return out


def _arg(flag, default=None):
    return next((a.split("=", 1)[1] for a in sys.argv if a.startswith(flag + "=")), default)


if __name__ == "__main__":
    rec_arg = _arg("--rec")
    if not rec_arg:
        sys.exit(
            "usage: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/make_rrd.py --rec=PATH [--lidar=...] [--odom=...] "
            "[--tags=...] [--out=...]   (--rec is required)"
        )
    build(
        rec_arg,
        lidar_stream=_arg("--lidar", "pointlio_lidar"),
        odom_stream=_arg("--odom", "pointlio_odometry"),
        tag_stream=_arg("--tags", "raw_april_tags"),
        out_name=_arg("--out", "gt_compare.rrd"),
    )
