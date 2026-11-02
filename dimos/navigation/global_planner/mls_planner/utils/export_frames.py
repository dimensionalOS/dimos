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

"""Export a recording's lidar clouds with their tf poses for the native replay bench.

The clouds stay in the sensor frame and each carries the pose a tf lookup at its
stamp returns, built from an odometry stream, so the bench registers them as the
live ray tracer does. Frame layout, little-endian: f64 stamp, 7 f32 pose
(position xyz, orientation xyzw), u32 point count, then f32 xyz triples.

Usage:
    uv run python -m dimos.navigation.global_planner.mls_planner.utils.export_frames \\
        mem2.db china.frames --lidar-stream pointlio_lidar_optimized \\
        --odometry-stream pointlio_odometry_optimized
"""

from __future__ import annotations

import math
from pathlib import Path
import struct

import numpy as np
import typer

from dimos.mapping.ray_tracing.module import TF_MATCH_TOLERANCE_S
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.protocol.tf.tf import MultiTBuffer

FRAME_HEADER = struct.Struct("<d7fI")


def main(
    db: Path,
    out: Path,
    lidar_stream: str = typer.Option(..., help="Sensor-frame cloud stream the ray tracer reads"),
    odometry_stream: str = typer.Option(..., help="Odometry stream that stands in for tf"),
    world_frame: str = typer.Option("odom", help="Frame the poses are looked up in"),
) -> None:
    store = SqliteStore(path=str(db))
    tf = MultiTBuffer(buffer_size=math.inf)
    for obs in store.stream(odometry_stream).order_by("ts"):
        odom = obs.data
        tf.receive_transform(
            Transform(
                translation=odom.position,
                rotation=odom.orientation,
                frame_id=odom.frame_id,
                child_frame_id=odom.child_frame_id,
                ts=obs.ts,
            )
        )
    frames = 0
    with out.open("wb") as f:
        for obs in store.stream(lidar_stream).order_by("ts"):
            cloud = obs.data
            pose = tf.get(
                world_frame, cloud.frame_id, time_point=obs.ts, time_tolerance=TF_MATCH_TOLERANCE_S
            )
            if pose is None:
                continue
            points = np.asarray(cloud.pointcloud.points, dtype="<f4")
            t, r = pose.translation, pose.rotation
            f.write(FRAME_HEADER.pack(obs.ts, t.x, t.y, t.z, r.x, r.y, r.z, r.w, len(points)))
            f.write(points.tobytes())
            frames += 1
    print(f"{frames} frames -> {out}")


if __name__ == "__main__":
    typer.run(main)
