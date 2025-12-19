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

import time
from typing import BinaryIO

from dimos_lcm.geometry_msgs import Transform as LCMTransform
from dimos_lcm.geometry_msgs import TransformStamped as LCMTransformStamped

try:
    from geometry_msgs.msg import TransformStamped as ROSTransformStamped
    from geometry_msgs.msg import Transform as ROSTransform
    from geometry_msgs.msg import Vector3 as ROSVector3
    from geometry_msgs.msg import Quaternion as ROSQuaternion
except ImportError:
    ROSTransformStamped = None
    ROSTransform = None
    ROSVector3 = None
    ROSQuaternion = None

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs import Header
from dimos.types.timestamped import Timestamped


class Transform(Timestamped):
    translation: Vector3
    rotation: Quaternion
    ts: float
    frame_id: str
    child_frame_id: str
    msg_name = "tf2_msgs.TFMessage"

    def __init__(
        self,
        translation: Vector3 | None = None,
        rotation: Quaternion | None = None,
        frame_id: str = "world",
        child_frame_id: str = "unset",
        ts: float = 0.0,
        **kwargs,
    ) -> None:
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        self.ts = ts if ts != 0.0 else time.time()
        self.translation = translation if translation is not None else Vector3()
        self.rotation = rotation if rotation is not None else Quaternion()

    def __repr__(self) -> str:
        return f"Transform(translation={self.translation!r}, rotation={self.rotation!r})"

    def __str__(self) -> str:
        return f"Transform:\n {self.frame_id} -> {self.child_frame_id} Translation: {self.translation}\n  Rotation: {self.rotation}"

    def __eq__(self, other) -> bool:
        """Check if two transforms are equal."""
        if not isinstance(other, Transform):
            return False
        return self.translation == other.translation and self.rotation == other.rotation

    @classmethod
    def identity(cls) -> Transform:
        """Create an identity transform."""
        return cls()

    def lcm_transform(self) -> LCMTransformStamped:
        return LCMTransformStamped(
            child_frame_id=self.child_frame_id,
            header=Header(self.ts, self.frame_id),
            transform=LCMTransform(
                translation=self.translation,
                rotation=self.rotation,
            ),
        )

    def __add__(self, other: "Transform") -> "Transform":
        """Compose two transforms (transform composition).

        The operation self + other represents applying transformation 'other'
        in the coordinate frame defined by 'self'. This is equivalent to:
        - First apply transformation 'self' (from frame A to frame B)
        - Then apply transformation 'other' (from frame B to frame C)

        Args:
            other: The transform to compose with this one

        Returns:
            A new Transform representing the composed transformation

        Example:
            t1 = Transform(Vector3(1, 0, 0), Quaternion(0, 0, 0, 1))
            t2 = Transform(Vector3(2, 0, 0), Quaternion(0, 0, 0, 1))
            t3 = t1 + t2  # Combined transform: translation (3, 0, 0)
        """
        if not isinstance(other, Transform):
            raise TypeError(f"Cannot add Transform and {type(other).__name__}")

        # Compose orientations: self.rotation * other.rotation
        new_rotation = self.rotation * other.rotation

        # Transform other's translation by self's rotation, then add to self's translation
        rotated_translation = self.rotation.rotate_vector(other.translation)
        new_translation = self.translation + rotated_translation

        return Transform(
            translation=new_translation,
            rotation=new_rotation,
            frame_id=self.frame_id,
            child_frame_id=other.child_frame_id,
            ts=self.ts,
        )

    def inverse(self) -> "Transform":
        """Compute the inverse transform.

        The inverse transform reverses the direction of the transformation.
        If this transform goes from frame A to frame B, the inverse goes from B to A.

        Returns:
            A new Transform representing the inverse transformation
        """
        # Inverse rotation
        inv_rotation = self.rotation.inverse()

        # Inverse translation: -R^(-1) * t
        inv_translation = inv_rotation.rotate_vector(self.translation)
        inv_translation = Vector3(-inv_translation.x, -inv_translation.y, -inv_translation.z)

        return Transform(
            translation=inv_translation,
            rotation=inv_rotation,
            frame_id=self.child_frame_id,  # Swap frame references
            child_frame_id=self.frame_id,
            ts=self.ts,
        )

    @classmethod
    def from_ros_transform_stamped(cls, ros_msg: ROSTransformStamped) -> "Transform":
        """Create a Transform from a ROS geometry_msgs/TransformStamped message.

        Args:
            ros_msg: ROS TransformStamped message

        Returns:
            Transform instance
        """

        # Convert timestamp
        ts = ros_msg.header.stamp.sec + (ros_msg.header.stamp.nanosec / 1_000_000_000)

        # Convert translation
        translation = Vector3(
            ros_msg.transform.translation.x,
            ros_msg.transform.translation.y,
            ros_msg.transform.translation.z,
        )

        # Convert rotation
        rotation = Quaternion(
            ros_msg.transform.rotation.x,
            ros_msg.transform.rotation.y,
            ros_msg.transform.rotation.z,
            ros_msg.transform.rotation.w,
        )

        return cls(
            translation=translation,
            rotation=rotation,
            frame_id=ros_msg.header.frame_id,
            child_frame_id=ros_msg.child_frame_id,
            ts=ts,
        )

    def to_ros_transform_stamped(self) -> ROSTransformStamped:
        """Convert to a ROS geometry_msgs/TransformStamped message.

        Returns:
            ROS TransformStamped message
        """

        ros_msg = ROSTransformStamped()

        # Set header
        ros_msg.header.frame_id = self.frame_id
        ros_msg.header.stamp.sec = int(self.ts)
        ros_msg.header.stamp.nanosec = int((self.ts - int(self.ts)) * 1_000_000_000)

        # Set child frame
        ros_msg.child_frame_id = self.child_frame_id

        # Set transform
        ros_msg.transform.translation = ROSVector3(
            x=self.translation.x, y=self.translation.y, z=self.translation.z
        )
        ros_msg.transform.rotation = ROSQuaternion(
            x=self.rotation.x, y=self.rotation.y, z=self.rotation.z, w=self.rotation.w
        )

        return ros_msg

    def __neg__(self) -> "Transform":
        """Unary minus operator returns the inverse transform."""
        return self.inverse()

    @classmethod
    def from_pose(cls, frame_id: str, pose: "Pose | PoseStamped") -> "Transform":
        """Create a Transform from a Pose or PoseStamped.

        Args:
            pose: A Pose or PoseStamped object to convert

        Returns:
            A Transform with the same translation and rotation as the pose
        """
        # Import locally to avoid circular imports
        from dimos.msgs.geometry_msgs.Pose import Pose
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        # Handle both Pose and PoseStamped
        if isinstance(pose, PoseStamped):
            return cls(
                translation=pose.position,
                rotation=pose.orientation,
                frame_id=pose.frame_id,
                child_frame_id=frame_id,
                ts=pose.ts,
            )
        elif isinstance(pose, Pose):
            return cls(
                translation=pose.position,
                rotation=pose.orientation,
                child_frame_id=frame_id,
            )
        else:
            raise TypeError(f"Expected Pose or PoseStamped, got {type(pose).__name__}")

    def to_pose(self) -> "PoseStamped":
        """Create a Transform from a Pose or PoseStamped.

        Args:
            pose: A Pose or PoseStamped object to convert

        Returns:
            A Transform with the same translation and rotation as the pose
        """
        # Import locally to avoid circular imports
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        # Handle both Pose and PoseStamped
        return PoseStamped(
            position=self.translation,
            orientation=self.rotation,
            frame_id=self.frame_id,
        )

    def lcm_encode(self) -> bytes:
        # we get a circular import otherwise
        from dimos.msgs.tf2_msgs.TFMessage import TFMessage

        return TFMessage(self).lcm_encode()

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> Transform:
        """Decode from LCM TFMessage bytes."""
        from dimos_lcm.tf2_msgs import TFMessage as LCMTFMessage

        lcm_msg = LCMTFMessage.lcm_decode(data)

        if not lcm_msg.transforms:
            raise ValueError("No transforms found in LCM message")

        # Get the first transform from the message
        lcm_transform_stamped = lcm_msg.transforms[0]

        # Extract timestamp from header
        ts = lcm_transform_stamped.header.stamp.sec + (
            lcm_transform_stamped.header.stamp.nsec / 1_000_000_000
        )

        # Create and return Transform instance
        return cls(
            translation=Vector3(
                lcm_transform_stamped.transform.translation.x,
                lcm_transform_stamped.transform.translation.y,
                lcm_transform_stamped.transform.translation.z,
            ),
            rotation=Quaternion(
                lcm_transform_stamped.transform.rotation.x,
                lcm_transform_stamped.transform.rotation.y,
                lcm_transform_stamped.transform.rotation.z,
                lcm_transform_stamped.transform.rotation.w,
            ),
            frame_id=lcm_transform_stamped.header.frame_id,
            child_frame_id=lcm_transform_stamped.child_frame_id,
            ts=ts,
        )
