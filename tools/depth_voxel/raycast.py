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

"""Rebuild both maps with the Rust raycast mapper instead of column carving."""

from __future__ import annotations

from collections.abc import Iterator
import json
import sys
import time
from typing import Any

import numpy as np

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.memory.cli.dataset import open_store
from dimos.memory.tf import StreamTF
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from tools.depth_voxel.filters import DepthFilter
from tools.depth_voxel.maps import VOXEL_SIZE_METERS, compare, shared_window
from tools.depth_voxel.pipeline import DepthProjector, PoseTrack
from tools.depth_voxel.run_experiment import FRAME_STRIDE, RESULTS_DIR

RAYCAST_MAX_RANGE_METERS = 30.0
"""The RayTracingVoxelMap module default. Also drops the lidar map's ±80 m strays."""

WINNING_FILTER = DepthFilter(name="02-range-4m", max_range_m=4.0)


def raycast_map(frames: Iterator[tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
    """Integer voxel keys of a map accumulated from (points, sensor origin) frames."""
    mapper = VoxelRayMapper(voxel_size=VOXEL_SIZE_METERS, max_range=RAYCAST_MAX_RANGE_METERS)
    for points, origin in frames:
        mapper.add_frame(np.ascontiguousarray(points, dtype=np.float32), tuple(origin))
    centers = mapper.global_map().astype(np.float64)
    return np.floor(centers / VOXEL_SIZE_METERS).astype(np.int64)


def to_world(points: np.ndarray, matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    world = points @ matrix[:3, :3].T.astype(np.float32) + matrix[:3, 3].astype(np.float32)
    return world, matrix[:3, 3]


def lidar_frames(
    store: Any, poses: PoseTrack, start: float, end: float
) -> Iterator[tuple[np.ndarray, np.ndarray]]:
    for obs in store.stream("lidar", PointCloud2).after(start).before(end):
        pose = poses.at(float(obs.ts))
        points = obs.data.points_f32()
        if pose is None or not len(points):
            continue
        yield to_world(points, pose.to_matrix())


def depth_frames(
    store: Any,
    projector: DepthProjector,
    start: float,
    end: float,
    depth_filter: DepthFilter,
) -> Iterator[tuple[np.ndarray, np.ndarray]]:
    depth_filter.reset()
    stream = store.stream("depth_image", Image).after(start).before(end)
    for index, obs in enumerate(stream):
        if index % FRAME_STRIDE:
            continue
        odom_to_lidar = projector.poses.at(float(obs.ts))
        if odom_to_lidar is None:
            continue
        points = projector.camera_points(depth_filter(obs.data.data))
        if not len(points):
            continue
        yield to_world(points, odom_to_lidar.to_matrix() @ projector.lidar_to_depth)


def main(db_path: str) -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    store = open_store(db_path)
    with store:
        tf = StreamTF.from_store(store, "tf_corrected")
        assert tf is not None
        poses = PoseTrack.from_store(store)
        projector = DepthProjector.from_store(store, tf, poses)
        start, end = shared_window(store)

        began = time.time()
        lidar_keys = raycast_map(lidar_frames(store, poses, start, end))
        np.save(RESULTS_DIR / "raycast_lidar_keys.npy", lidar_keys)
        print(f"raycast lidar map: {len(lidar_keys)} voxels in {time.time() - began:.0f} s")

        began = time.time()
        depth_keys = raycast_map(depth_frames(store, projector, start, end, WINNING_FILTER))
        np.save(RESULTS_DIR / "raycast_depth_keys.npy", depth_keys)
        print(f"raycast depth map: {len(depth_keys)} voxels in {time.time() - began:.0f} s")

    stats = compare(depth_keys, lidar_keys)
    (RESULTS_DIR / "raycast.json").write_text(
        json.dumps({"name": "raycast-" + WINNING_FILTER.name, **stats.as_dict()}, indent=2)
    )
    print(stats.summary())


if __name__ == "__main__":
    main(sys.argv[1])
