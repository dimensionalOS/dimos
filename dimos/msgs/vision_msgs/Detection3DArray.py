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

from dimos_lcm.vision_msgs.Detection3DArray import Detection3DArray as LCMDetection3DArray

from dimos.types.timestamped import to_timestamp


class Detection3DArray(LCMDetection3DArray):  # type: ignore[misc]
    msg_name = "vision_msgs.Detection3DArray"

    # for _get_field_type() to work when decoding in _decode_one()
    __annotations__ = LCMDetection3DArray.__annotations__

    @property
    def ts(self) -> float:
        return to_timestamp(self.header.stamp)

    @property
    def frame_id(self) -> str:
        return str(self.header.frame_id)

    def to_rerun(self) -> Any:
        """Convert detections to a Rerun Boxes3D archetype."""
        import rerun as rr

        centers: list[tuple[float, float, float]] = []
        half_sizes: list[tuple[float, float, float]] = []
        quaternions: list[tuple[float, float, float, float]] = []
        labels: list[str] = []

        for detection in self.detections[: self.detections_length]:
            bbox = detection.bbox
            center = bbox.center.position
            orientation = bbox.center.orientation
            size = bbox.size

            centers.append((center.x, center.y, center.z))
            half_sizes.append((size.x / 2.0, size.y / 2.0, size.z / 2.0))
            quaternions.append(
                (
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )
            )
            labels.append(_label_for_detection(detection))

        return rr.Boxes3D(
            centers=centers,
            half_sizes=half_sizes,
            quaternions=quaternions,
            labels=labels,
        )


def _label_for_detection(detection: Any) -> str:
    marker_id = str(getattr(detection, "id", "")).strip()
    for result in detection.results[: detection.results_length]:
        class_id = str(result.hypothesis.class_id).strip()
        if marker_id and class_id:
            return f"{class_id} id={marker_id}"
        if class_id:
            return class_id
    if marker_id:
        return f"id={marker_id}"
    return ""
