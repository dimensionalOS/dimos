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

from lcm_msgs.foxglove_msgs import SceneUpdate  # type: ignore[import-not-found]

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

    def to_foxglove_scene_update(self) -> SceneUpdate:
        """Convert all detections to a Foxglove SceneUpdate message.

        Returns:
            SceneUpdate containing SceneEntity objects for all detections
        """

        # Create SceneUpdate message with all detections
        scene_update = SceneUpdate()
        scene_update.deletions_length = 0
        scene_update.deletions = []
        scene_update.entities = []

        # Process each detection
        for i, detection in enumerate(self.detections):
            entity = detection.to_foxglove_scene_entity(entity_id=f"detection_{detection.name}_{i}")
            scene_update.entities.append(entity)

        scene_update.entities_length = len(scene_update.entities)
        return scene_update
