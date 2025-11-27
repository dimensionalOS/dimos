# Copyright 2025 Dimensional Inc.
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

from lcm_msgs.foxglove_msgs import SceneUpdate  # type: ignore[import-not-found]

from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.perception.detection.type.imageDetections import ImageDetections


class ImageDetections3DPC(ImageDetections[Detection3DPC]):
    """Specialized class for 3D detections in an image."""

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
