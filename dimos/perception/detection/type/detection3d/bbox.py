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

from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    TransformStamped,
    Vector3,
)
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)

from dimos.msgs.geometry import point_distance
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox


@dataclass
class Detection3DBBox(Detection2DBBox):
    """3D bounding box detection with center, size, and orientation.

    Represents a 3D detection as an oriented bounding box in world space.
    """

    center: Vector3  # Center point in world frame
    size: Vector3  # Width, height, depth
    transform: TransformStamped | None = None  # Camera to world transform
    frame_id: str = ""  # Frame ID (e.g., "world", "map")
    orientation: Quaternion = field(default_factory=lambda: Quaternion(w=1))

    @functools.cached_property
    def pose(self) -> PoseStamped:
        """Convert detection to a PoseStamped using bounding box center.

        Returns pose in world frame with the detection's orientation.
        """
        return PoseStamped(
            header=Header(stamp=self.image.header.stamp, frame_id=self.frame_id),
            pose=Pose(
                position=Point(x=self.center.x, y=self.center.y, z=self.center.z),
                orientation=self.orientation,
            ),
        )

    def to_detection3d_msg(self) -> Detection3D:
        """Convert to ROS Detection3D message."""
        msg = Detection3D()
        msg.header = Header(stamp=self.image.header.stamp, frame_id=self.frame_id)

        # Results
        msg.results = [
            ObjectHypothesisWithPose(
                hypothesis=ObjectHypothesis(
                    class_id=str(self.class_id),
                    score=self.confidence,
                )
            )
        ]

        msg.bbox = BoundingBox3D(
            center=Pose(
                position=Point(x=self.center.x, y=self.center.y, z=self.center.z),
                orientation=self.orientation,
            ),
            size=self.size,
        )

        return msg

    def to_repr_dict(self) -> dict[str, Any]:
        # Calculate distance from camera
        if self.transform is None:
            return super().to_repr_dict()
        camera_pos = self.transform.transform.translation
        distance = point_distance(
            Point(x=self.center.x, y=self.center.y, z=self.center.z),
            Point(x=camera_pos.x, y=camera_pos.y, z=camera_pos.z),
        )

        parent_dict = super().to_repr_dict()
        # Remove bbox key if present
        parent_dict.pop("bbox", None)

        return {
            **parent_dict,
            "dist": f"{distance:.2f}m",
            "size": f"[{self.size.x:.2f},{self.size.y:.2f},{self.size.z:.2f}]",
        }
