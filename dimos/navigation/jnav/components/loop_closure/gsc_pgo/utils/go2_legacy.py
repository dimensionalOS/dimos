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

"""Its bad to have lidar in ''world'' frame, but thats what the go2 does
This file undoes that by making a new stream (l1_cloud) that is in sensor-frame
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.jnav.utils.recording_tf import RecordingTF

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store

TF_STREAM = "tf"
# go2's stuff is in a fake world frame, so we correct that with a new db stream
LEGACY_ODOM_STREAMS = {"odom", "go2_odom"}
GO2_CORRECTED_LIDAR_STREAM_NAME = "l1_cloud"
GO2_CORRECTED_LIDAR_FRAME = "l1_link"
# proper "Odometry" type instead of Pose
GO2_CORRECTED_ODOMETRY_STREAM_NAME = "go2_odometry"
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


def _write_l1_cloud(
    store: Store, source_lidar: str, odom_rows: np.ndarray, world_frame: str, tf: RecordingTF
) -> None:
    """Write ``source_lidar`` into ``l1_cloud`` (``l1_link`` frame), un-registering via ``tf``.

    ``tf`` carries an ephemeral ``world_frame -> l1_link`` edge (the odom trajectory), so a
    world-registered scan is pulled back into the sensor frame by the tf chain rather than
    hand-rolled quat math; scans already in a sensor frame pass through unchanged. Either way
    the latched odom pose is kept on the row so ``p_world = pose * p_l1`` reconstructs the map."""
    odom_times = odom_rows[:, 0]
    if GO2_CORRECTED_LIDAR_STREAM_NAME in store.list_streams():
        store.delete_stream(GO2_CORRECTED_LIDAR_STREAM_NAME)
    out_stream = store.stream(GO2_CORRECTED_LIDAR_STREAM_NAME, PointCloud2)
    count = 0
    for observation in store.stream(source_lidar, PointCloud2):
        scan_ts = float(observation.ts)
        source_points = np.asarray(observation.data.points_f32())
        latched = max(int(np.searchsorted(odom_times, scan_ts, side="right")) - 1, 0)
        xyzquat = odom_rows[latched][1:]
        if observation.data.frame_id == world_frame:
            world_to_sensor = tf.get(GO2_CORRECTED_LIDAR_FRAME, world_frame, scan_ts)
            assert world_to_sensor is not None
            matrix = world_to_sensor.to_matrix()
            l1_points = (source_points @ matrix[:3, :3].T + matrix[:3, 3]).astype(np.float32)
        else:
            l1_points = source_points.astype(np.float32)
        intensities = observation.data.intensities_f32()
        cloud = PointCloud2.from_numpy(
            l1_points,
            frame_id=GO2_CORRECTED_LIDAR_FRAME,
            intensities=(np.asarray(intensities) if intensities is not None else None),
        )
        cloud.ts = scan_ts
        out_stream.append(cloud, ts=scan_ts, pose=tuple(float(value) for value in xyzquat))
        count += 1
        if count % LOG_EVERY == 0:
            print(f"  {GO2_CORRECTED_LIDAR_STREAM_NAME}: {count} scans...", flush=True)
    print(
        f"wrote {GO2_CORRECTED_LIDAR_STREAM_NAME}: {count} scans "
        f"in {GO2_CORRECTED_LIDAR_FRAME} frame",
        flush=True,
    )


def _write_static_tf(store: Store, stamp: float, base_frame: str) -> None:
    """Add an identity ``base_frame -> l1_link`` edge so the sensor frame joins the tf tree."""
    edge = Transform(frame_id=base_frame, child_frame_id=GO2_CORRECTED_LIDAR_FRAME, ts=stamp)
    store.stream(TF_STREAM, TFMessage).append(TFMessage(edge), ts=stamp)
    print(f"wrote static tf {base_frame} -> {GO2_CORRECTED_LIDAR_FRAME} (identity)", flush=True)


def _write_go2_odometry(store: Store, odom_rows: np.ndarray, world_frame: str) -> None:
    """Rewrite the bare-``Pose`` odom as a proper ``world -> l1_link`` ``Odometry`` stream."""
    if GO2_CORRECTED_ODOMETRY_STREAM_NAME in store.list_streams():
        store.delete_stream(GO2_CORRECTED_ODOMETRY_STREAM_NAME)
    stream = store.stream(GO2_CORRECTED_ODOMETRY_STREAM_NAME, Odometry)
    for row in odom_rows:
        stamp = float(row[0])
        x, y, z, qx, qy, qz, qw = (float(value) for value in row[1:])
        stream.append(
            Odometry(
                ts=stamp,
                frame_id=world_frame,
                child_frame_id=GO2_CORRECTED_LIDAR_FRAME,
                pose=Pose(x, y, z, qx, qy, qz, qw),
            ),
            ts=stamp,
            pose=(x, y, z, qx, qy, qz, qw),
        )
    print(f"wrote {GO2_CORRECTED_ODOMETRY_STREAM_NAME}: {len(odom_rows)} poses", flush=True)


def normalize_go2_legacy(
    store: Store, odom_tf: str, odom_stream: str, lidar_stream: str
) -> tuple[str, str, str]:
    """Convert a legacy go2 recording in place, returning ``(odom_tf, odom, lidar)`` to use.

    On a go2-legacy recording this derives ``l1_cloud``, a static ``base_link -> l1_link``
    tf, and a ``go2_odometry`` stream, then returns ``("<world>:l1_link", "go2_odometry",
    "l1_cloud")``. Any other recording is untouched and its inputs are returned unchanged.
    """
    if not _is_go2_legacy(store, odom_stream):
        return odom_tf, odom_stream, lidar_stream
    odom_rows = _odom_pose_rows(store, odom_stream)
    if not len(odom_rows):
        return odom_tf, odom_stream, lidar_stream
    world_frame, _, base_frame = odom_tf.partition(":")
    print("go2 legacy recording: deriving l1_cloud / go2_odometry / l1_link tf", flush=True)
    # Ephemeral world->l1_link edge (the odom trajectory) so _write_l1_cloud un-registers
    # world-framed scans through the tf chain instead of hand-rolled quat math.
    tf = RecordingTF(store.stream(TF_STREAM, TFMessage))
    tf.override_edge(world_frame, GO2_CORRECTED_LIDAR_FRAME, odom_rows[:, 0], odom_rows[:, 1:8])
    _write_l1_cloud(store, lidar_stream, odom_rows, world_frame, tf)
    _write_static_tf(store, float(odom_rows[0][0]), base_frame)
    _write_go2_odometry(store, odom_rows, world_frame)
    return (
        f"{world_frame}:{GO2_CORRECTED_LIDAR_FRAME}",
        GO2_CORRECTED_ODOMETRY_STREAM_NAME,
        GO2_CORRECTED_LIDAR_STREAM_NAME,
    )
