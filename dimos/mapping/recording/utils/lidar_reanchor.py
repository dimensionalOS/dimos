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

"""Re-anchor a world-frame lidar stream onto a corrected trajectory.

A lidar stream is stored in world frame *relative to some odometry* (e.g.
`fastlio_lidar` is relative to `fastlio_odometry`, `lidar` is relative to the
Go2 onboard `odom`). When a drift-corrected trajectory exists (`gtsam_odom`,
from AprilTag landmark SLAM), each cloud can be re-anchored onto it:

    P_corrected = T_gtsam(t) · T_odom(t)^-1 · P_world

i.e. subtract the cloud's own odometry (back to body) and re-apply the corrected
pose at the nearest timestamp. The actual point transform runs through
open3d via ``PointCloud2.transform``.
"""

from __future__ import annotations

import sqlite3

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _load_poses(db_path: str, stream: str):
    """(ts (N,), pose (N,7) x y z qx qy qz qw) for a pose-bearing stream, via SQL."""
    conn = sqlite3.connect(db_path)
    rows = conn.execute(
        f"SELECT ts,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw "
        f'FROM "{stream}" WHERE pose_qw IS NOT NULL ORDER BY ts'
    ).fetchall()
    conn.close()
    if not rows:
        raise ValueError(f"no populated poses in stream '{stream}'")
    return np.array([r[0] for r in rows]), np.array([r[1:8] for r in rows], dtype=np.float64)


def _mat(p):
    M = np.eye(4)
    M[:3, :3] = Rotation.from_quat(p[3:7]).as_matrix()
    M[:3, 3] = p[:3]
    return M


def _to_transform(M):
    q = Rotation.from_matrix(M[:3, :3]).as_quat()
    return Transform(
        translation=Vector3(float(M[0, 3]), float(M[1, 3]), float(M[2, 3])),
        rotation=Quaternion(float(q[0]), float(q[1]), float(q[2]), float(q[3])),
    )


def _nearest(ts_arr: np.ndarray, t: float) -> int:
    j = int(np.searchsorted(ts_arr, t))
    j = min(max(j, 0), len(ts_arr) - 1)
    if j > 0 and abs(ts_arr[j - 1] - t) < abs(ts_arr[j] - t):
        j -= 1
    return j


def reanchor_stream(
    store, db_path: str, *, lidar_stream: str, odom_stream: str, gtsam_stream: str, out_stream: str
) -> int:
    """Write `out_stream`: every cloud of `lidar_stream` re-anchored from
    `odom_stream` onto `gtsam_stream`. Returns the number of clouds written."""
    odom_ts, odom_pose = _load_poses(db_path, odom_stream)
    gt_ts, gt_pose = _load_poses(db_path, gtsam_stream)
    # precompute per-odom-node the corrected relative transform isn't possible
    # (it's per-cloud by timestamp), but caching odom/gtsam matrices is cheap.
    odom_M = [None] * len(odom_ts)
    gt_M = [None] * len(gt_ts)

    src = store.stream(lidar_stream, PointCloud2).to_list()
    if out_stream in store.list_streams():
        store.delete_stream(out_stream)
    out = store.stream(out_stream, PointCloud2)
    n = 0
    for obs in src:
        t = obs.ts
        i = _nearest(odom_ts, t)
        j = _nearest(gt_ts, t)
        if odom_M[i] is None:
            odom_M[i] = _mat(odom_pose[i])
        if gt_M[j] is None:
            gt_M[j] = _mat(gt_pose[j])
        rel = gt_M[j] @ np.linalg.inv(odom_M[i])
        cloud = obs.data
        corrected = cloud.transform(_to_transform(rel))  # open3d under the hood
        nc = PointCloud2.from_numpy(
            corrected.points_f32(), timestamp=t, intensities=cloud.intensities_f32()
        )
        out.append(nc, ts=t, pose=tuple(gt_pose[j]))
        n += 1
    print(
        f"   {out_stream}: {n} clouds re-anchored ({lidar_stream} via {odom_stream} -> {gtsam_stream})"
    )
    return n
