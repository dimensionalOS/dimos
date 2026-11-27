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

from typing import TYPE_CHECKING, Any, Generic

from typing_extensions import TypeVar

from dimos.perception.detection.type.detection2d.base import Detection2D
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.imageDetections import ImageDetections

if TYPE_CHECKING:
    from ultralytics.engine.results import Results

    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray

T2D = TypeVar("T2D", bound=Detection2D, default=Detection2DBBox)


class ImageDetections2D(ImageDetections[T2D], Generic[T2D]):
    @classmethod
    def from_ros_detection2d_array(
        cls,
        image: Image,
        ros_detections: Detection2DArray,
        **kwargs: Any,
    ) -> ImageDetections2D[Detection2DBBox]:
        """Convert from ROS Detection2DArray message to ImageDetections2D object."""
        detections: list[Detection2DBBox] = []
        for ros_det in ros_detections.detections:
            detection = Detection2DBBox.from_ros_detection2d(ros_det, image=image, **kwargs)
            if detection.is_valid():
                detections.append(detection)

        return ImageDetections2D(image=image, detections=detections)

    @classmethod
    def from_ultralytics_result(
        cls, image: Image, results: list[Results]
    ) -> ImageDetections2D[Detection2DBBox]:
        # The adapter imports ImageDetections2D, so import it here to avoid a cycle.
        from dimos.perception.detection.detectors.ultralytics_utils import detections_from_results

        return detections_from_results(image, results)
