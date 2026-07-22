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

"""Convert world-registered (fast|point)lio lidar scans back into the sensor frame, in place.

Each scan is un-registered by the most recent sample of the sibling ``*_odometry``
stream (the latched pose the lio node registered it with — per Jeff: the scan row's own
pose field is not trustworthy), then rewritten with ``frame_id=mid360_link`` and the
odom pose kept on the row — so ``p_world = pose * p_sensor`` reconstructs the original.

Crash-safe: the converted stream is fully written to ``<name>__sensor_tmp`` before the
original is replaced, so no data is ever lost mid-run. Streams already in the target
frame are redone from their stored row pose (exact inversion) if they have one, else
treated as natively sensor-frame and skipped. Kitti dbs are excluded per Jeff.

Usage: python .../scripts/to_sensor_frame.py [--db=path.db | --root=~/datasets/RECORDINGS_DIR]
       [--frame=mid360_link] [--dry-run]
"""

from pathlib import Path
import re
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.scripts.make_rrd import cli_arg
from dimos.navigation.jnav.utils import recording_db as rdb

DEFAULT_FRAME = "mid360_link"
LIDAR_PATTERN = re.compile(r"^(old_)?(fastlio|pointlio)_lidar(_no_cap)?$")
LOG_EVERY = 5000
EXCLUDED_DATASETS = ("kitti", "hk_village")  # no mid360 sensor in these, per Jeff


def pose_to_xyzquat(pose):
    return (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )


def rewrite_stream(store, source_name, out_name, transform):
    """Copy ``source_name`` into ``out_name`` (created fresh), mapping each scan through
    ``transform(observation) -> (PointCloud2, pose_xyzquat)``; returns the scan count."""
    if out_name in store.list_streams():
        store.delete_stream(out_name)
    out_stream = store.stream(out_name, PointCloud2)
    count = 0
    for observation in store.stream(source_name):
        cloud, pose = transform(observation)
        out_stream.append(cloud, ts=float(observation.ts), pose=pose)
        count += 1
        if count % LOG_EVERY == 0:
            print(f"    {count} scans...", flush=True)
    return count


def convert_stream(store, db_path, lidar_name, odom_name, target_frame):
    odom_rows = rdb.odometry_rows(db_path, odom_name)
    if len(odom_rows):
        odom_rows = odom_rows[np.all(np.isfinite(odom_rows), axis=1)]  # lio startup NaN rows
    if not len(odom_rows):
        print(f"  {lidar_name}: SKIP ({odom_name} has no finite poses)", flush=True)
        return
    odom_times = odom_rows[:, 0]

    def to_sensor(observation):
        points = np.asarray(observation.data.points_f32())
        if observation.data.frame_id == target_frame:
            # redo a previous conversion: reconstruct world points from the stored pose
            row = np.asarray(pose_to_xyzquat(observation.pose), float)
            points = points @ Rotation.from_quat(row[3:7]).as_matrix().T + row[:3]
        latched = max(np.searchsorted(odom_times, float(observation.ts), side="right") - 1, 0)
        xyzquat = odom_rows[latched][1:]
        rotation = Rotation.from_quat(xyzquat[3:7]).as_matrix()
        sensor_points = ((points - xyzquat[:3]) @ rotation).astype(np.float32)
        intensities = observation.data.intensities_f32()
        cloud = PointCloud2.from_numpy(
            sensor_points,
            frame_id=target_frame,
            intensities=(np.asarray(intensities) if intensities is not None else None),
        )
        cloud.ts = float(observation.ts)
        return cloud, tuple(float(value) for value in xyzquat)

    def copy_through(observation):
        cloud = observation.data
        return cloud, tuple(float(value) for value in pose_to_xyzquat(observation.pose))

    tmp_name = f"{lidar_name}__sensor_tmp"
    count = rewrite_stream(store, lidar_name, tmp_name, to_sensor)
    radius = float("nan")
    for observation in store.stream(tmp_name):
        radius = float(np.median(np.linalg.norm(np.asarray(observation.data.points_f32()), axis=1)))
        break
    store.delete_stream(lidar_name)
    rewrite_stream(store, tmp_name, lidar_name, copy_through)
    store.delete_stream(tmp_name)
    print(
        f"  {lidar_name}: {count} scans -> {target_frame} (via {odom_name}, "
        f"first-scan median range {radius:.1f} m)",
        flush=True,
    )


def convert_db(db_path, target_frame, dry_run):
    store = rdb.store(db_path)
    streams = store.list_streams()
    pairs = []
    for lidar_name in sorted(streams):
        if not LIDAR_PATTERN.match(lidar_name):
            continue
        odom_name = lidar_name.replace("_lidar", "_odometry")
        if odom_name not in streams:
            print(f"  {lidar_name}: SKIP (no {odom_name})", flush=True)
            continue
        for observation in store.stream(lidar_name):
            native_sensor = observation.data.frame_id == target_frame and observation.pose is None
            break
        else:
            continue  # empty stream
        if native_sensor:
            print(f"  {lidar_name}: already {target_frame} (native)", flush=True)
            continue
        pairs.append((lidar_name, odom_name))
    for lidar_name, odom_name in pairs:
        if dry_run:
            print(f"  {lidar_name}: would convert (via {odom_name})", flush=True)
        else:
            convert_stream(store, db_path, lidar_name, odom_name, target_frame)


def main():
    target_frame = cli_arg("--frame", DEFAULT_FRAME)
    dry_run = "--dry-run" in sys.argv
    single_db = cli_arg("--db")
    root_arg = cli_arg("--root")
    if single_db:
        db_paths = [Path(single_db).expanduser()]
    else:
        if not root_arg:
            sys.exit("pass --db=<file.db> or --root=<recordings dir>")
        root = Path(root_arg).expanduser()
        db_paths = sorted(
            path
            for path in root.rglob("*.db")
            if "orig" not in path.name
            and path.is_file()
            and not any(excluded in str(path) for excluded in EXCLUDED_DATASETS)
        )
    for db_path in db_paths:
        print(f"{db_path}:", flush=True)
        try:
            convert_db(db_path, target_frame, dry_run)
        except Exception as error:  # keep going: one broken db shouldn't stop the sweep
            print(f"  ERROR: {error}", flush=True)
    rdb.close_all()


if __name__ == "__main__":
    main()
