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

from typing import TYPE_CHECKING, Generic, List, Optional, TypeVar

from dimos_lcm.vision_msgs import Detection2DArray

from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.std_msgs import Header
from dimos.perception.detection.type.utils import TableStr

if TYPE_CHECKING:
    from dimos.perception.detection.type.detection2d.base import Detection2D

    T = TypeVar("T", bound=Detection2D)
else:
    from dimos.perception.detection.type.detection2d.base import Detection2D

    T = TypeVar("T", bound=Detection2D)


class ImageDetections(Generic[T], TableStr):
    image: Image
    detections: List[T]

    @property
    def ts(self) -> float:
        return self.image.ts

    def __init__(self, image: Image, detections: Optional[List[T]] = None):
        self.image = image
        self.detections = detections or []
        for det in self.detections:
            if not det.ts:
                det.ts = image.ts

    def __len__(self):
        return len(self.detections)

    def __iter__(self):
        return iter(self.detections)

    def __getitem__(self, index):
        return self.detections[index]

    def to_ros_detection2d_array(self) -> Detection2DArray:
        return Detection2DArray(
            detections_length=len(self.detections),
            header=Header(self.image.ts, "camera_optical"),
            detections=[det.to_ros_detection2d() for det in self.detections],
        )

    def to_foxglove_annotations(self) -> ImageAnnotations:
        def flatten(xss):
            return [x for xs in xss for x in xs]

        texts = flatten(det.to_text_annotation() for det in self.detections)
        points = flatten(det.to_points_annotation() for det in self.detections)

        return ImageAnnotations(
            texts=texts,
            texts_length=len(texts),
            points=points,
            points_length=len(points),
        )
