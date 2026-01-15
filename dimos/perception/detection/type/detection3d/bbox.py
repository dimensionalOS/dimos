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

from dataclasses import dataclass, field
import functools
from typing import Any

from dimos_lcm.vision_msgs import ObjectHypothesis, ObjectHypothesisWithPose

from dimos.msgs.geometry_msgs import Pose, PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.std_msgs import Header
from dimos.msgs.vision_msgs import Detection3D
from dimos.perception.detection.type.detection2d import Detection2DBBox


@dataclass
class Detection3DBBox(Detection2DBBox):
    """3D bounding box detection with center, size, and orientation.

    Represents a 3D detection as an oriented bounding box in world space.
    """

    center: Vector3  # Center point in world frame
    size: Vector3  # Width, height, depth
    transform: Transform | None = None  # Camera to world transform
    frame_id: str = ""  # Frame ID (e.g., "world", "map")
    orientation: Quaternion = field(default_factory=lambda: Quaternion(0.0, 0.0, 0.0, 1.0))

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

    def to_detection3d_msg(self) -> Detection3D:
        """Convert to ROS Detection3D message."""
        msg = Detection3D()
        msg.header = Header(self.ts, self.frame_id)

        # Results
        msg.results = [
            ObjectHypothesisWithPose(
                hypothesis=ObjectHypothesis(
                    class_id=str(self.class_id),
                    score=self.confidence,
                )
            )
        ]

        # Bounding Box
        msg.bbox.center = Pose(
            position=self.center,
            orientation=self.orientation,
        )
        msg.bbox.size = self.size

        return msg

    def to_repr_dict(self) -> dict[str, Any]:
        # Calculate distance from camera
        if self.transform is None:
            return super().to_repr_dict()
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
