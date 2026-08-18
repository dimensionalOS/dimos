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

"""One-off: inspect the Alfred recording's depth stream, intrinsics and tf tree."""

from __future__ import annotations

from collections import Counter
import sqlite3
import sys

import numpy as np

from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


def rows(connection: sqlite3.Connection, stream: str, limit: int, offset: int = 0):
    return connection.execute(
        f'SELECT s.ts, b.data FROM "{stream}" s JOIN "{stream}_blob" b ON b.id = s.id '
        f"ORDER BY s.ts LIMIT ? OFFSET ?",
        (limit, offset),
    ).fetchall()


def main(db_path: str) -> None:
    connection = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)

    ts, blob = rows(connection, "depth_camera_info", 1)[0]
    info = CameraInfo.lcm_decode(blob)
    print("depth_camera_info:", info.width, "x", info.height, "frame", info.frame_id)
    print("  K =", np.asarray(info.K).reshape(3, 3).tolist())
    print("  D =", list(info.D))

    ts, blob = rows(connection, "depth_image", 1, offset=500)[0]
    depth = Image.lcm_decode(blob)
    array = depth.data
    print(
        "depth_image:",
        array.shape,
        array.dtype,
        "format",
        depth.format,
        "frame",
        depth.frame_id,
    )
    finite = array[array > 0]
    print(
        f"  nonzero {finite.size}/{array.size} ({100 * finite.size / array.size:.1f}%)",
        f"min {finite.min()} max {finite.max()} median {np.median(finite):.0f}",
    )

    edges: Counter[tuple[str, str]] = Counter()
    for _ts, blob in rows(connection, "tf_corrected", 400):
        for transform in TFMessage.lcm_decode(blob).transforms:
            edges[(transform.frame_id, transform.child_frame_id)] += 1
    print("tf_corrected edges (first 400 msgs):")
    for (parent, child), count in edges.most_common():
        print(f"  {parent} -> {child}  x{count}")

    ts, blob = rows(connection, "odometry", 1)[0]
    odom = Odometry.lcm_decode(blob)
    print("odometry:", odom.frame_id, "->", odom.child_frame_id)

    ts, blob = rows(connection, "lidar", 1)[0]
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    cloud = PointCloud2.lcm_decode(blob)
    lidar_points = cloud.points_f32()
    ranges = np.linalg.norm(lidar_points, axis=1)
    print("lidar:", lidar_points.shape, "frame", cloud.frame_id)
    print("  extent", lidar_points.min(axis=0).round(2), lidar_points.max(axis=0).round(2))
    print(f"  range min {ranges.min():.2f} median {np.median(ranges):.2f} max {ranges.max():.2f}")


if __name__ == "__main__":
    main(sys.argv[1])
