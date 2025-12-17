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

from abc import abstractmethod
from collections.abc import Callable

from dimos_lcm.vision_msgs import Detection2D as ROSDetection2D  # type: ignore[import-untyped]

from dimos.msgs.foxglove_msgs import ImageAnnotations
from dimos.msgs.sensor_msgs import Image
from dimos.types.timestamped import Timestamped


class Detection2D(Timestamped):
    """Abstract base class for 2D detections."""

    @abstractmethod
    def cropped_image(self, padding: int = 20) -> Image:
        """Return a cropped version of the image focused on the detection area."""
        ...

    @abstractmethod
    def to_image_annotations(self) -> ImageAnnotations:
        """Convert detection to Foxglove ImageAnnotations for visualization."""
        ...

    @abstractmethod
    def to_ros_detection2d(self) -> ROSDetection2D:
        """Convert detection to ROS Detection2D message."""
        ...

    @abstractmethod
    def is_valid(self) -> bool:
        """Check if the detection is valid."""
        ...


Filter2D = Callable[[Detection2D], bool]
