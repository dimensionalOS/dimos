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

from abc import abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

from dimos.perception.detection.type.detection2d import Detection2DBBox

if TYPE_CHECKING:
    from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]

    from dimos.msgs.geometry_msgs import Transform


@dataclass
class Detection3D(Detection2DBBox):
    """Abstract base class for 3D detections."""

    transform: Transform
    frame_id: str

    @classmethod
    @abstractmethod
    def from_2d(
        cls,
        det: Detection2DBBox,
        distance: float,
        camera_info: CameraInfo,
        world_to_optical_transform: Transform,
    ) -> Detection3D | None:
        """Create a 3D detection from a 2D detection."""
        ...
