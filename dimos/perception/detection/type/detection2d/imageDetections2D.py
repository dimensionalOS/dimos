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

from typing import TYPE_CHECKING, Generic

from typing_extensions import TypeVar

from dimos.perception.detection.type.detection2d.base import Detection2D
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.imageDetections import ImageDetections

if TYPE_CHECKING:
    from dimos_lcm.vision_msgs import Detection2DArray  # type: ignore[import-untyped]
    from ultralytics.engine.results import Results

    from dimos.msgs.sensor_msgs import Image

# TypeVar with default - Detection2DBBox is the default when no type param given
T2D = TypeVar("T2D", bound=Detection2D, default=Detection2DBBox)


class ImageDetections2D(ImageDetections[T2D], Generic[T2D]):
    @classmethod
    def from_ros_detection2d_array(  # type: ignore[no-untyped-def]
        cls, image: Image, ros_detections: Detection2DArray, **kwargs
    ) -> ImageDetections2D[Detection2DBBox]:
        """Convert from ROS Detection2DArray message to ImageDetections2D object."""
        detections: list[Detection2DBBox] = []
        for ros_det in ros_detections.detections:
            detection = Detection2DBBox.from_ros_detection2d(ros_det, image=image, **kwargs)
            if detection.is_valid():
                detections.append(detection)

        return ImageDetections2D(image=image, detections=detections)

    @classmethod
    def from_ultralytics_result(  # type: ignore[no-untyped-def]
        cls, image: Image, results: list[Results], **kwargs
    ) -> ImageDetections2D[Detection2DBBox]:
        """Create ImageDetections2D from ultralytics Results.

        Dispatches to appropriate Detection2D subclass based on result type:
        - If keypoints present: creates Detection2DPerson
        - Otherwise: creates Detection2DBBox

        Args:
            image: Source image
            results: List of ultralytics Results objects
            **kwargs: Additional arguments passed to detection constructors

        Returns:
            ImageDetections2D containing appropriate detection types
        """
        from dimos.perception.detection.type.detection2d.person import Detection2DPerson

        detections: list[Detection2DBBox] = []
        for result in results:
            if result.boxes is None:
                continue

            num_detections = len(result.boxes.xyxy)
            for i in range(num_detections):
                detection: Detection2DBBox
                if result.keypoints is not None:
                    # Pose detection with keypoints
                    detection = Detection2DPerson.from_ultralytics_result(result, i, image)
                else:
                    # Regular bbox detection
                    detection = Detection2DBBox.from_ultralytics_result(result, i, image)
                if detection.is_valid():
                    detections.append(detection)

        return ImageDetections2D(image=image, detections=detections)
