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
"""Combined comparison rrd: raw lidar cloud + EVERY *_corrected*_lidar version present in the db,
each as its own colored entity, plus AprilTag landmarks + trajectories. Re-run after adding a new
corrected method and it picks the new stream up automatically.

Importable: `build(...)` writes the rrd and returns its path (used by post_process.py).
Standalone: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/scripts/make_rrd.py --rec=PATH [--lidar=...] [--odom=...] [--tags=...] [--out=...]
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
    out_name="corrected_compare.rrd",
):
    recording_dir = Path(rec).expanduser()
    db_path = recording_dir / "mem2.db"
    out_path = recording_dir / out_name
    store = rdb.store(db_path)
    intrinsics = json.loads((recording_dir / "camera_intrinsics.json").read_text())
    optical_in_base = np.array(intrinsics["optical_in_base"], float)
    base_to_optical = Pose3(
        Rot3.Quaternion(
            optical_in_base[6], optical_in_base[3], optical_in_base[4], optical_in_base[5]
        ),
        Point3(optical_in_base[0], optical_in_base[1], optical_in_base[2]),
    )

    def accumulate(stream_name):
        scans = []
        for scan_index, observation in enumerate(store.stream(stream_name)):
            if scan_index % SCAN_STRIDE:
                continue
            points = np.asarray(observation.data.points_f32())
            if len(points):
                scans.append(points[::3])
        all_points = np.concatenate(scans, 0)
        _, unique_indices = np.unique(
            np.floor(all_points / VOXEL).astype(np.int64), axis=0, return_index=True
        )
        return all_points[unique_indices]

    def traj(stream_name):
        return np.array(
            [
                [
                    observation.data.pose.position.x,
                    observation.data.pose.position.y,
                    observation.data.pose.position.z,
                ]
                for observation in store.stream(stream_name)
            ],
            np.float32,
        )

    def landmarks(gt_odom):
        odom_poses = [
            (
                observation.ts,
                Pose3(
                    Rot3.Quaternion(
                        observation.data.pose.orientation.w,
                        observation.data.pose.orientation.x,
                        observation.data.pose.orientation.y,
                        observation.data.pose.orientation.z,
                    ),
                    Point3(
                        observation.data.pose.position.x,
                        observation.data.pose.position.y,
                        observation.data.pose.position.z,
                    ),
                ),
            )
            for observation in store.stream(gt_odom)
        ]
        odom_timestamps = np.array([timestamp for timestamp, _ in odom_poses])
        positions_by_marker = {}
        for tag_observation in store.stream(tag_stream):
            tag_metrics = tag_observation.tags
            if not (
                float(tag_metrics["sharpness"]) >= GATE["s"]
                and float(tag_metrics["reproj_px"]) <= GATE["r"]
                and float(tag_metrics["tag_px"]) >= GATE["px"]
                # older tag streams lack distance/view-angle; unknown passes the gate
                and float(tag_metrics.get("distance_m", 0.0)) <= GATE["d"]
                and float(tag_metrics.get("view_angle_deg", 0.0)) <= GATE["a"]
                and (
                    float(tag_metrics["lin_speed"]) < 0
                    or float(tag_metrics["lin_speed"]) <= GATE["lv"]
                )
                and (
                    float(tag_metrics["ang_speed"]) < 0
                    or float(tag_metrics["ang_speed"]) <= GATE["av"]
                )
            ):
                continue
            tag_pose = tag_observation.data
            closest_base_pose = odom_poses[
                int(np.argmin(np.abs(odom_timestamps - float(tag_observation.ts))))
            ][1]
            tag_in_world = closest_base_pose.compose(base_to_optical).compose(
                Pose3(
                    Rot3.Quaternion(
                        tag_pose.orientation.w,
                        tag_pose.orientation.x,
                        tag_pose.orientation.y,
                        tag_pose.orientation.z,
                    ),
                    Point3(tag_pose.x, tag_pose.y, tag_pose.z),
                )
            )
            positions_by_marker.setdefault(int(tag_metrics["marker_id"]), []).append(
                np.asarray(tag_in_world.translation())
            )
        mean_positions = [
            np.mean(positions, 0) for marker_id, positions in sorted(positions_by_marker.items())
        ]
        labels = [f"tag{marker_id}" for marker_id in sorted(positions_by_marker)]
        return np.array(mean_positions), labels

    streams = store.list_streams()
    corrected_lidars = sorted(
        stream_name
        for stream_name in streams
        if "_corrected" in stream_name and "_lidar" in stream_name
    )
    print("raw + corrected lidar streams:", corrected_lidars)

    rr.init("corrected_compare")
    rr.save(str(out_path))
    rr.log(
        "raw/cloud",
        rr.Points3D(accumulate(lidar_stream), colors=COLORS["raw"], radii=0.02),
        static=True,
    )
    rr.log(
        "raw/trajectory", rr.LineStrips3D([traj(odom_stream)], colors=[255, 120, 120]), static=True
    )
    for lidar_index, lidar_name in enumerate(corrected_lidars):
        color = PALETTE[lidar_index % len(PALETTE)]
        cloud = accumulate(lidar_name)
        rr.log(f"{lidar_name}/cloud", rr.Points3D(cloud, colors=color, radii=0.02), static=True)
        print(f"  logged {lidar_name}: {len(cloud):,} pts")
        odom_name = lidar_name.replace("_lidar", "_odometry")
        if odom_name in streams:
            rr.log(
                f"{lidar_name}/trajectory",
                rr.LineStrips3D([traj(odom_name)], colors=color),
                static=True,
            )
    # landmarks placed against the first available corrected odometry
    corrected_odoms = sorted(
        stream_name
        for stream_name in streams
        if "_corrected" in stream_name and "_odometry" in stream_name
    )
    if corrected_odoms:
        landmark_positions, labels = landmarks(corrected_odoms[0])
        if len(landmark_positions):
            rr.log(
                "landmarks",
                rr.Points3D(landmark_positions, colors=[255, 230, 0], radii=0.25, labels=labels),
                static=True,
            )
            print(f"  logged {len(labels)} landmarks")
    print("wrote", out_path)
    return out_path


def _arg(flag, default=None):
    return next((arg.split("=", 1)[1] for arg in sys.argv if arg.startswith(flag + "=")), default)


if __name__ == "__main__":
    rec_arg = _arg("--rec")
    if not rec_arg:
        sys.exit(
            "usage: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/scripts/make_rrd.py --rec=PATH [--lidar=...] [--odom=...] "
            "[--tags=...] [--out=...]   (--rec is required)"
        )
    build(
        rec_arg,
        lidar_stream=_arg("--lidar", "pointlio_lidar"),
        odom_stream=_arg("--odom", "pointlio_odometry"),
        tag_stream=_arg("--tags", "raw_april_tags"),
        out_name=_arg("--out", "corrected_compare.rrd"),
    )
