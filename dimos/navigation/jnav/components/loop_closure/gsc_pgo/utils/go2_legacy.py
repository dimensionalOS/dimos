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

"""Normalize legacy go2 recordings so the generic post-processing pipeline can run.

Legacy go2 recordings differ from every other rig in three ways: the odom stream is
named ``odom``/``go2_odom`` and carries a bare ``Pose`` (not ``Odometry``), the lidar is
stored already world-registered, and the sensor frame (``l1_link``) is never published
into tf. :func:`normalize_go2_legacy` detects that shape and writes the three streams the
generic pipeline expects, then returns the streams/edge to feed it. Any other recording is
a no-op that returns its inputs unchanged, so post-processing stays generic apart from one
call.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store

LEGACY_ODOM_STREAMS = {"odom", "go2_odom"}
WORLD_FRAME = "world"
SENSOR_FRAME = "l1_link"
BASE_FRAME = "base_link"
TF_STREAM = "tf"
OUT_LIDAR = "l1_cloud"
OUT_ODOM = "go2_odometry"
LOG_EVERY = 5000


def _is_go2_legacy(store: Store, odom_stream: str) -> bool:
    """True when ``odom_stream`` is a go2-legacy bare-``Pose`` odom (not ``Odometry``)."""
    if odom_stream not in LEGACY_ODOM_STREAMS:
        return False
    observation = next(iter(store.stream(odom_stream, PoseStamped)), None)
    if observation is None:
        return False
    return not isinstance(observation.data, Odometry)


def _odom_pose_rows(store: Store, odom_stream: str) -> np.ndarray:
    """``(N, 8)`` ``ts, x, y, z, qx, qy, qz, qw`` from the odom ``Pose`` payloads, NaN-filtered."""
    rows = [
        (
            float(observation.ts),
            observation.data.position.x,
            observation.data.position.y,
            observation.data.position.z,
            observation.data.orientation.x,
            observation.data.orientation.y,
            observation.data.orientation.z,
            observation.data.orientation.w,
        )
        for observation in store.stream(odom_stream, PoseStamped)
    ]
    array = np.asarray(rows, dtype=float).reshape(-1, 8)
    if len(array):
        array = array[np.all(np.isfinite(array), axis=1)]
    return array


def _write_l1_cloud(store: Store, source_lidar: str, odom_rows: np.ndarray) -> None:
    """Un-register the world-framed ``source_lidar`` into ``l1_cloud`` (``l1_link`` frame).

    Each scan is brought back into the sensor frame by the latched odom pose, and that
    pose is kept on the row so ``p_world = pose * p_l1`` reconstructs the original."""
    odom_times = odom_rows[:, 0]
    if OUT_LIDAR in store.list_streams():
        store.delete_stream(OUT_LIDAR)
    out_stream = store.stream(OUT_LIDAR, PointCloud2)
    count = 0
    for observation in store.stream(source_lidar, PointCloud2):
        scan_ts = float(observation.ts)
        world_points = np.asarray(observation.data.points_f32())
        latched = max(int(np.searchsorted(odom_times, scan_ts, side="right")) - 1, 0)
        xyzquat = odom_rows[latched][1:]
        rotation = Rotation.from_quat(xyzquat[3:7]).as_matrix()
        l1_points = ((world_points - xyzquat[:3]) @ rotation).astype(np.float32)
        intensities = observation.data.intensities_f32()
        cloud = PointCloud2.from_numpy(
            l1_points,
            frame_id=SENSOR_FRAME,
            intensities=(np.asarray(intensities) if intensities is not None else None),
        )
        cloud.ts = scan_ts
        out_stream.append(cloud, ts=scan_ts, pose=tuple(float(value) for value in xyzquat))
        count += 1
        if count % LOG_EVERY == 0:
            print(f"  {OUT_LIDAR}: {count} scans...", flush=True)
    print(f"wrote {OUT_LIDAR}: {count} scans in {SENSOR_FRAME} frame", flush=True)


def _write_static_tf(store: Store, stamp: float) -> None:
    """Add an identity ``base_link -> l1_link`` edge so the sensor frame joins the tf tree."""
    edge = Transform(frame_id=BASE_FRAME, child_frame_id=SENSOR_FRAME, ts=stamp)
    store.stream(TF_STREAM, TFMessage).append(TFMessage(edge), ts=stamp)
    print(f"wrote static tf {BASE_FRAME} -> {SENSOR_FRAME} (identity)", flush=True)


def _write_go2_odometry(store: Store, odom_rows: np.ndarray) -> None:
    """Rewrite the bare-``Pose`` odom as a proper ``world -> l1_link`` ``Odometry`` stream."""
    if OUT_ODOM in store.list_streams():
        store.delete_stream(OUT_ODOM)
    stream = store.stream(OUT_ODOM, Odometry)
    for row in odom_rows:
        stamp = float(row[0])
        x, y, z, qx, qy, qz, qw = (float(value) for value in row[1:])
        stream.append(
            Odometry(
                ts=stamp,
                frame_id=WORLD_FRAME,
                child_frame_id=SENSOR_FRAME,
                pose=Pose(x, y, z, qx, qy, qz, qw),
            ),
            ts=stamp,
            pose=(x, y, z, qx, qy, qz, qw),
        )
    print(f"wrote {OUT_ODOM}: {len(odom_rows)} poses", flush=True)


def normalize_go2_legacy(
    store: Store, odom_tf: str, odom_stream: str, lidar_stream: str
) -> tuple[str, str, str]:
    """Convert a legacy go2 recording in place, returning ``(odom_tf, odom, lidar)`` to use.

    On a go2-legacy recording this derives ``l1_cloud``, a static ``base_link -> l1_link``
    tf, and a ``go2_odometry`` stream, then returns ``("world:l1_link", "go2_odometry",
    "l1_cloud")``. Any other recording is untouched and its inputs are returned unchanged.
    """
    if not _is_go2_legacy(store, odom_stream):
        return odom_tf, odom_stream, lidar_stream
    odom_rows = _odom_pose_rows(store, odom_stream)
    if not len(odom_rows):
        return odom_tf, odom_stream, lidar_stream
    print("go2 legacy recording: deriving l1_cloud / go2_odometry / l1_link tf", flush=True)
    _write_l1_cloud(store, lidar_stream, odom_rows)
    _write_static_tf(store, float(odom_rows[0][0]))
    _write_go2_odometry(store, odom_rows)
    return f"{WORLD_FRAME}:{SENSOR_FRAME}", OUT_ODOM, OUT_LIDAR
