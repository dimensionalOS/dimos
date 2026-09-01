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

from typing import Generic

from typing_extensions import TypeVar

from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.bbox import Detection3DBBox
from dimos.perception.detection.type.imageDetections import ImageDetections

T3D = TypeVar("T3D", bound=Detection3DBBox, default=Detection3DBBox)


class ImageDetections3D(ImageDetections[T3D], Generic[T3D]):
    """Image-scoped 3D detections backed by 2D debug bboxes."""

    def to_ros_detection3d_array(self, frame_id: str | None = None) -> Detection3DArray:
        resolved_frame_id = frame_id
        if resolved_frame_id is None:
            resolved_frame_id = self.detections[0].frame_id if self.detections else ""

        detections = [det.to_detection3d_msg() for det in self.detections]
        return Detection3DArray(
            detections_length=len(detections),
            header=Header(self.image.ts, resolved_frame_id),
            detections=detections,
        )
