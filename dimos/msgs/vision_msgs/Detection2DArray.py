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
from typing import Any

from dimos_lcm.vision_msgs.Detection2DArray import (
    Detection2DArray as LCMDetection2DArray,
)

from dimos.msgs.vision_msgs.Detection3DArray import label_for_detection
from dimos.types.timestamped import to_timestamp


class Detection2DArray(LCMDetection2DArray):  # type: ignore[misc]
    msg_name = "vision_msgs.Detection2DArray"

    # for _get_field_type() to work when decoding in _decode_one()
    __annotations__ = LCMDetection2DArray.__annotations__

    @property
    def ts(self) -> float:
        return to_timestamp(self.header.stamp)

    @property
    def frame_id(self) -> str:
        return str(self.header.frame_id)

    def to_rerun(self) -> Any:
        """Convert detections to a Rerun Boxes2D archetype in image-pixel space.

        Mirrors :meth:`Detection3DArray.to_rerun` so 2D overlays clear and label
        the same way 3D boxes do: each ``bbox`` (center + size) becomes an XYWH
        pixel-space box and shares the same ``id``/``class_id`` label logic. An
        empty array yields an empty ``Boxes2D`` so a marker-free frame clears the
        overlay instead of leaving stale boxes on screen.
        """
        import rerun as rr

        boxes: list[tuple[float, float, float, float]] = []
        labels: list[str] = []

        for detection in self.detections[: self.detections_length]:
            bbox = detection.bbox
            center_x = bbox.center.position.x
            center_y = bbox.center.position.y
            width = bbox.size_x
            height = bbox.size_y

            # rr.Box2DFormat.XYWH wants the top-left corner plus size.
            boxes.append((center_x - width / 2.0, center_y - height / 2.0, width, height))
            labels.append(label_for_detection(detection))

        return rr.Boxes2D(
            array=boxes,
            array_format=rr.Box2DFormat.XYWH,
            labels=labels,
        )
