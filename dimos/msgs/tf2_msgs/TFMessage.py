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

# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

from typing import TYPE_CHECKING, BinaryIO

from dimos_lcm.tf2_msgs import TFMessage as LCMTFMessage  # type: ignore[import-untyped]

try:
    from geometry_msgs.msg import (  # type: ignore[attr-defined, import-untyped]
        TransformStamped as ROSTransformStamped,
    )
    from tf2_msgs.msg import TFMessage as ROSTFMessage  # type: ignore[attr-defined, import-untyped]
except ImportError:
    ROSTFMessage = None  # type: ignore[assignment, misc]
    ROSTransformStamped = None  # type: ignore[assignment, misc]

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

if TYPE_CHECKING:
    from collections.abc import Iterator


class TFMessage:
    """TFMessage that accepts Transform objects and encodes to LCM format."""

    transforms: list[Transform]
    msg_name = "tf2_msgs.TFMessage"

    def __init__(self, *transforms: Transform) -> None:
        self.transforms = list(transforms)

    def add_transform(self, transform: Transform, child_frame_id: str = "base_link") -> None:
        """Add a transform to the message."""
        self.transforms.append(transform)
        self.transforms_length = len(self.transforms)

    def lcm_encode(self) -> bytes:
        """Encode as LCM TFMessage.

        Args:
            child_frame_ids: Optional list of child frame IDs for each transform.
                           If not provided, defaults to "base_link" for all.
        """

        res = list(map(lambda t: t.lcm_transform(), self.transforms))

        lcm_msg = LCMTFMessage(
            transforms_length=len(self.transforms),
            transforms=res,
        )

        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> TFMessage:
        """Decode from LCM TFMessage bytes."""
        lcm_msg = LCMTFMessage.lcm_decode(data)

        # Convert LCM TransformStamped objects to Transform objects
        transforms = []
        for lcm_transform_stamped in lcm_msg.transforms:
            # Extract timestamp
            ts = lcm_transform_stamped.header.stamp.sec + (
                lcm_transform_stamped.header.stamp.nsec / 1_000_000_000
            )

            # Create Transform with our custom types
            lcm_trans = lcm_transform_stamped.transform.translation
            lcm_rot = lcm_transform_stamped.transform.rotation

            transform = Transform(
                translation=Vector3(lcm_trans.x, lcm_trans.y, lcm_trans.z),
                rotation=Quaternion(lcm_rot.x, lcm_rot.y, lcm_rot.z, lcm_rot.w),
                frame_id=lcm_transform_stamped.header.frame_id,
                child_frame_id=lcm_transform_stamped.child_frame_id,
                ts=ts,
            )
            transforms.append(transform)

        return cls(*transforms)

    def __len__(self) -> int:
        """Return number of transforms."""
        return len(self.transforms)

    def __getitem__(self, index: int) -> Transform:
        """Get transform by index."""
        return self.transforms[index]

    def __iter__(self) -> Iterator:  # type: ignore[type-arg]
        """Iterate over transforms."""
        return iter(self.transforms)

    def __repr__(self) -> str:
        return f"TFMessage({len(self.transforms)} transforms)"

    def __str__(self) -> str:
        lines = [f"TFMessage with {len(self.transforms)} transforms:"]
        for i, transform in enumerate(self.transforms):
            lines.append(f"  [{i}] {transform.frame_id} @ {transform.ts:.3f}")
        return "\n".join(lines)

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSTFMessage) -> TFMessage:
        """Create a TFMessage from a ROS tf2_msgs/TFMessage message.

        Args:
            ros_msg: ROS TFMessage message

        Returns:
            TFMessage instance
        """
        transforms = []
        for ros_transform_stamped in ros_msg.transforms:
            # Convert from ROS TransformStamped to our Transform
            transform = Transform.from_ros_transform_stamped(ros_transform_stamped)
            transforms.append(transform)

        return cls(*transforms)

    def to_ros_msg(self) -> ROSTFMessage:
        """Convert to a ROS tf2_msgs/TFMessage message.

        Returns:
            ROS TFMessage message
        """
        ros_msg = ROSTFMessage()  # type: ignore[no-untyped-call]

        # Convert each Transform to ROS TransformStamped
        for transform in self.transforms:
            ros_msg.transforms.append(transform.to_ros_transform_stamped())

        return ros_msg
