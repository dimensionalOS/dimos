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

from dataclasses import dataclass
import functools
from typing import Any

from dimos.msgs.geometry_msgs import PoseStamped, Transform, Vector3
from dimos.perception.detection.type.detection2d import Detection2DBBox


@dataclass
class Detection3DBBox(Detection2DBBox):
    """3D bounding box detection with center, size, and orientation.

    Represents a 3D detection as an oriented bounding box in world space.
    """

    transform: Transform  # Camera to world transform
    frame_id: str  # Frame ID (e.g., "world", "map")
    center: Vector3  # Center point in world frame
    size: Vector3  # Width, height, depth
    orientation: tuple[float, float, float, float]  # Quaternion (x, y, z, w)

    @functools.cached_property
    def pose(self) -> PoseStamped:
        """Convert detection to a PoseStamped using bounding box center.

        Returns pose in world frame with the detection's orientation.
        """
        return PoseStamped(
            ts=self.ts,
            frame_id=self.frame_id,
            position=self.center,
            orientation=self.orientation,
        )

    def to_repr_dict(self) -> dict[str, Any]:
        # Calculate distance from camera
        camera_pos = self.transform.translation
        distance = (self.center - camera_pos).magnitude()

        parent_dict = super().to_repr_dict()
        # Remove bbox key if present
        parent_dict.pop("bbox", None)

        return {
            **parent_dict,
            "dist": f"{distance:.2f}m",
            "size": f"[{self.size.x:.2f},{self.size.y:.2f},{self.size.z:.2f}]",
        }
