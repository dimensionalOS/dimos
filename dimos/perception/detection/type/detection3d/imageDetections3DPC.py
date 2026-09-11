# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC, lattice_quantum
from dimos.perception.detection.type.imageDetections import ImageDetections

if TYPE_CHECKING:
    from dimos_lcm.sensor_msgs import CameraInfo

    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
    from dimos.perception.detection.type.detection3d.pointcloud_filters import PointCloudFilter


class ImageDetections3DPC(ImageDetections[Detection3DPC]):
    """Specialized class for 3D detections in an image."""

    @classmethod
    def from_2d(
        cls,
        detections_2d: ImageDetections2D,
        world_pointcloud: PointCloud2,
        camera_info: CameraInfo,
        world_to_optical_transform: Transform,
        filters: list[PointCloudFilter] | None = None,
    ) -> ImageDetections3DPC:
        """Project every 2D detection into 3D, dropping any that yield no valid points.

        The cloud is projected through the camera once; each detection then
        selects its points from that shared projection, with the mask splat
        radius taken from the cloud's own lattice pitch.
        """
        world_points, points_2d = Detection3DPC.project_cloud(
            world_pointcloud, camera_info, world_to_optical_transform
        )
        quantum = lattice_quantum(world_points)
        detections_3d = [
            d3d
            for det in detections_2d
            if (
                d3d := Detection3DPC.from_projection(
                    det,
                    world_points,
                    points_2d,
                    camera_info,
                    world_to_optical_transform,
                    world_pointcloud.frame_id,
                    world_pointcloud.ts,
                    filters,
                    splat_m=quantum / 2 if quantum is not None else None,
                )
            )
            is not None
        ]
        return cls(image=detections_2d.image, detections=detections_3d)

    @classmethod
    def from_depth(
        cls,
        detections_2d: ImageDetections2D,
        depth: Image,
        camera_info: CameraInfo,
        world_to_optical_transform: Transform,
        filters: list[PointCloudFilter] | None = None,
    ) -> ImageDetections3DPC:
        """Unproject every detection's depth pixels into 3D, dropping empty results."""
        detections_3d = [
            d3d
            for det in detections_2d
            if (
                d3d := Detection3DPC.from_depth(
                    det,
                    depth,
                    camera_info,
                    world_to_optical_transform,
                    filters,
                )
            )
            is not None
        ]
        return cls(image=detections_2d.image, detections=detections_3d)
