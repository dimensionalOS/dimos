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

import sys
from typing import TYPE_CHECKING, Generic, TypeVar

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

from dimos_generated.vision_msgs.msg import Detection2DArray

from dimos.msgs.image import image_from_array, image_to_bgr
from dimos.msgs.time import to_seconds
from dimos.perception.detection.type.utils import TableStr

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from dimos_generated.sensor_msgs.msg import Image

    from dimos.perception.detection.type.detection2d.base import Detection2D

    T = TypeVar("T", bound=Detection2D)
else:
    from dimos.perception.detection.type.detection2d.base import Detection2D

    T = TypeVar("T", bound=Detection2D)


class ImageDetections(Generic[T], TableStr):
    image: Image
    detections: list[T]

    @property
    def ts(self) -> float:
        return to_seconds(self.image.header.stamp)

    def __init__(self, image: Image, detections: list[T] | None = None) -> None:
        self.image = image
        self.detections = detections or []
        for det in self.detections:
            if not det.ts:
                det.ts = to_seconds(image.header.stamp)

    def __len__(self) -> int:
        return len(self.detections)

    def __iter__(self) -> Iterator:  # type: ignore[type-arg]
        return iter(self.detections)

    def __getitem__(self, index):  # type: ignore[no-untyped-def]
        return self.detections[index]

    def filter(self, *predicates: Callable[[T], bool]) -> Self:
        """Filter detections using one or more predicate functions.

        Multiple predicates are applied in cascade (all must return True).

        Args:
            *predicates: Functions that take a detection and return True to keep it

        Returns:
            A new instance of the same class with filtered detections
        """
        filtered_detections = [det for det in self.detections if all(p(det) for p in predicates)]
        return type(self)(self.image, filtered_detections)

    def to_ros_detection2d_array(self) -> Detection2DArray:
        return Detection2DArray(
            header=self.image.header,
            detections=[det.to_ros_detection2d() for det in self.detections],
        )

    def annotated_image(self, scale: float = 1.0) -> Image:
        """Return the image with all detection bboxes and labels drawn on it."""
        img = image_to_bgr(self.image)
        for det in self.detections:
            if hasattr(det, "draw_on"):
                det.draw_on(img, scale=scale)

        return image_from_array(img, encoding="bgr8", header=self.image.header)
