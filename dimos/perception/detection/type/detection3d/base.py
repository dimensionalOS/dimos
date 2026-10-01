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

from abc import abstractmethod
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped

from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox

if TYPE_CHECKING:
    from dimos_generated.sensor_msgs.msg import CameraInfo


@dataclass
class Detection3D(Detection2DBBox):
    """Abstract base class for 3D detections."""

    frame_id: str = ""
    transform: TransformStamped = field(
        default_factory=lambda: TransformStamped(transform=Transform(rotation=Quaternion(w=1)))
    )

    @classmethod
    @abstractmethod
    def from_2d(
        cls,
        det: Detection2DBBox,
        distance: float,
        camera_info: CameraInfo,
        world_to_optical_transform: TransformStamped,
    ) -> Detection3D | None:
        """Create a 3D detection from a 2D detection."""
        ...
