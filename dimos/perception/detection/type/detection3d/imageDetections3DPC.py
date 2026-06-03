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

from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.perception.detection.type.imageDetections import ImageDetections

if TYPE_CHECKING:
    from dimos_lcm.sensor_msgs import CameraInfo

    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
    from dimos.perception.detection.type.detection3d.pointcloud_filters import PointCloudFilter


class ImageDetections3DPC(ImageDetections[Detection3DPC]):
    """Specialized class for 3D detections in an image."""

    def to_ros_detection3d_array(self, frame_id: str | None = None) -> Detection3DArray:
        """Pack the per-object pointcloud detections into one ``Detection3DArray``.

        Mirrors :meth:`ImageDetections3D.to_ros_detection3d_array` so the
        pointcloud fusion path can publish a ``Detection3DArray`` alongside its
        ``Detection2DArray``. With no detections this yields an empty-but-present
        array stamped at the image time (distinct from "no frame this tick").
        """
        resolved_frame_id = frame_id
        if resolved_frame_id is None:
            resolved_frame_id = self.detections[0].frame_id if self.detections else ""

        detections = [det.to_detection3d_msg() for det in self.detections]
        return Detection3DArray(
            detections_length=len(detections),
            header=Header(self.image.ts, resolved_frame_id),
            detections=detections,
        )

    @classmethod
    def from_2d(
        cls,
        detections_2d: ImageDetections2D,
        world_pointcloud: PointCloud2,
        camera_info: CameraInfo,
        world_to_optical_transform: Transform,
        filters: list[PointCloudFilter] | None = None,
    ) -> ImageDetections3DPC:
        """Project every 2D detection into 3D, dropping any that yield no valid points."""
        detections_3d = [
            d3d
            for det in detections_2d
            if (
                d3d := Detection3DPC.from_2d(
                    det,
                    world_pointcloud,
                    camera_info,
                    world_to_optical_transform,
                    filters,
                )
            )
            is not None
        ]
        return cls(image=detections_2d.image, detections=detections_3d)
