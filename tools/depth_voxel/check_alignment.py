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

"""Check the depth->world extrinsic chain by ICP-ing a depth cloud onto accumulated lidar."""

from __future__ import annotations

import sys

import numpy as np
import open3d as o3d

from dimos.memory.cli.dataset import open_store
from dimos.memory.tf import StreamTF
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from tools.depth_voxel.pipeline import DepthProjector, PoseTrack

LIDAR_WINDOW_SECONDS = 4.0
MAX_DEPTH_RANGE_METERS = 8.0
ICP_MAX_CORRESPONDENCE_METERS = 0.5
SAMPLE_OFFSETS_SECONDS = (20.0, 50.0, 80.0, 110.0)


def to_o3d(points: np.ndarray) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    return cloud


def main(db_path: str) -> None:
    store = open_store(db_path)
    with store:
        tf = StreamTF.from_store(store, "tf_corrected")
        assert tf is not None
        poses = PoseTrack.from_store(store)
        projector = DepthProjector.from_store(store, tf, poses)
        start = float(poses.timestamps[0])

        for offset in SAMPLE_OFFSETS_SECONDS:
            depth_obs = next(
                iter(
                    store.stream("depth_image", Image)
                    .after(start + offset)
                    .before(start + offset + 0.5)
                ),
                None,
            )
            if depth_obs is None:
                print(f"t+{offset:g}s: no depth frame")
                continue
            timestamp = float(depth_obs.ts)

            lidar_points = [
                _to_world(obs.data.points_f32(), poses, float(obs.ts))
                for obs in store.stream("lidar", PointCloud2)
                .after(timestamp - LIDAR_WINDOW_SECONDS)
                .before(timestamp + LIDAR_WINDOW_SECONDS)
                if poses.at(float(obs.ts)) is not None
            ]
            lidar = to_o3d(np.concatenate(lidar_points)).voxel_down_sample(0.05)

            cloud = projector.world_cloud(depth_obs.data.data, timestamp)
            assert cloud is not None
            depth_points = cloud.points_f32()
            camera_origin = poses.at(timestamp).to_matrix()[:3, 3]  # type: ignore[union-attr]
            near = depth_points[
                np.linalg.norm(depth_points - camera_origin, axis=1) < MAX_DEPTH_RANGE_METERS
            ]
            depth = to_o3d(near).voxel_down_sample(0.05)

            result = o3d.pipelines.registration.registration_icp(
                depth,
                lidar,
                ICP_MAX_CORRESPONDENCE_METERS,
                np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            )
            translation = result.transformation[:3, 3]
            print(
                f"t+{offset:g}s depth<{MAX_DEPTH_RANGE_METERS:g}m {len(depth.points):6d} vox | "
                f"lidar {len(lidar.points):6d} vox | fitness {result.fitness:.3f} "
                f"rmse {result.inlier_rmse:.3f} | icp shift {np.round(translation, 3).tolist()} "
                f"({np.linalg.norm(translation):.3f} m)"
            )


def _to_world(points: np.ndarray, poses: PoseTrack, timestamp: float) -> np.ndarray:
    matrix = poses.at(timestamp).to_matrix()  # type: ignore[union-attr]
    return points @ matrix[:3, :3].T.astype(np.float32) + matrix[:3, 3].astype(np.float32)


if __name__ == "__main__":
    main(sys.argv[1])
