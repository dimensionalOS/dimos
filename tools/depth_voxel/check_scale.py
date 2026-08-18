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

"""Sanity-check the depth unit and the extrinsic chain against concurrent lidar sweeps."""

from __future__ import annotations

import sys

import numpy as np
import open3d as o3d

from dimos.memory.cli.dataset import open_store
from dimos.memory.tf import StreamTF
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from tools.depth_voxel import pipeline
from tools.depth_voxel.pipeline import DepthProjector, PoseTrack

CANDIDATE_UNITS = (0.001, 0.0001)
SWEEP_HALF_WINDOW_SECONDS = 0.5


def nearest_distances(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(target.astype(np.float64))
    tree = o3d.geometry.KDTreeFlann(cloud)
    distances = np.empty(len(source))
    for index, point in enumerate(source.astype(np.float64)):
        _, _, squared = tree.search_knn_vector_3d(point, 1)
        distances[index] = np.sqrt(squared[0])
    return distances


def main(db_path: str) -> None:
    store = open_store(db_path)
    with store:
        tf = StreamTF.from_store(store, "tf_corrected")
        assert tf is not None
        poses = PoseTrack.from_store(store)
        projector = DepthProjector.from_store(store, tf, poses)

        depth_observations = list(
            store.stream("depth_image", Image)
            .after(poses.timestamps[0] + 20.0)
            .before(poses.timestamps[0] + 20.5)
        )
        depth_obs = depth_observations[0]
        timestamp = float(depth_obs.ts)

        sweeps = list(
            store.stream("lidar", PointCloud2)
            .after(timestamp - SWEEP_HALF_WINDOW_SECONDS)
            .before(timestamp + SWEEP_HALF_WINDOW_SECONDS)
        )
        lidar_world = np.concatenate(
            [
                _to_world(obs.data.points_f32(), poses, float(obs.ts))
                for obs in sweeps
                if poses.at(float(obs.ts)) is not None
            ]
        )
        print(f"depth ts {timestamp:.3f}  lidar sweeps {len(sweeps)}  points {len(lidar_world)}")

        for unit in CANDIDATE_UNITS:
            pipeline.DEPTH_UNIT_METERS = unit
            cloud = projector.world_cloud(depth_obs.data.data, timestamp)
            assert cloud is not None
            points = cloud.points_f32()
            sample = points[:: max(1, len(points) // 4000)]
            distances = nearest_distances(sample, lidar_world)
            print(
                f"unit {unit:<8} depth points {len(points):7d} "
                f"median-nn {np.median(distances):6.3f} m  "
                f"mean-nn {distances.mean():6.3f} m  "
                f"frac<0.25m {np.mean(distances < 0.25):.3f}"
            )


def _to_world(points: np.ndarray, poses: PoseTrack, timestamp: float) -> np.ndarray:
    matrix = poses.at(timestamp).to_matrix()  # type: ignore[union-attr]
    return points @ matrix[:3, :3].T.astype(np.float32) + matrix[:3, 3].astype(np.float32)


if __name__ == "__main__":
    main(sys.argv[1])
