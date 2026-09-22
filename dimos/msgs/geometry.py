# Copyright 2026 Dimensional Inc.
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

"""Geometry operations on generated ROS value types."""

import math

from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.std_msgs.msg import Header
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation


def yaw(rotation: Quaternion) -> float:
    """Return rotation about Z in radians (ROS XYZW quaternion convention)."""
    return math.atan2(
        2 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def transform_from_odometry(message: Odometry) -> TransformStamped:
    """Copy the pose and declared frames into TF, preserving the exact source stamp."""
    position = message.pose.pose.position
    return TransformStamped(
        header=message.header,
        child_frame_id=message.child_frame_id,
        transform=Transform(
            translation=Vector3(x=position.x, y=position.y, z=position.z),
            rotation=message.pose.pose.orientation,
        ),
    )


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert fixed-axis XYZ angles in radians to a normalized ROS quaternion."""
    if not all(math.isfinite(value) for value in (roll, pitch, yaw)):
        raise ValueError("Euler angles must be finite")
    x, y, z, w = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_quat()
    return Quaternion(x=x, y=y, z=z, w=w)


def _rotation(quaternion: Quaternion) -> Rotation:
    values = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Quaternion components must be finite")
    # SciPy rejects zero norm and normalizes non-unit input.
    return Rotation.from_quat(values)


def _translation(transform: Transform) -> NDArray[np.float64]:
    translation = transform.translation
    values = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
    if not np.isfinite(values).all():
        raise ValueError("Translation components must be finite")
    return values


def _transform(translation: NDArray[np.float64], rotation: Rotation) -> Transform:
    x, y, z, w = rotation.as_quat()
    return Transform(
        translation=Vector3(x=translation[0], y=translation[1], z=translation[2]),
        rotation=Quaternion(x=x, y=y, z=z, w=w),
    )


def transform_matrix(transform: Transform) -> NDArray[np.float64]:
    """Copy a generated transform into a homogeneous 4-by-4 matrix."""
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = _rotation(transform.rotation).as_matrix()
    matrix[:3, 3] = _translation(transform)
    return matrix


def compose_transforms(first: TransformStamped, second: TransformStamped) -> TransformStamped:
    """Compose A←B and B←C, preserving A's exact source stamp in the A←C result.

    Inputs are copied into a new value. The shared frame must match; callers
    querying a time-varying chain are responsible for choosing each edge's time.
    """
    if first.child_frame_id != second.header.frame_id:
        raise ValueError(
            f"Cannot compose frames {first.child_frame_id!r} and {second.header.frame_id!r}"
        )
    rotation = _rotation(first.transform.rotation)
    translation = _translation(first.transform) + rotation.apply(_translation(second.transform))
    return TransformStamped(
        header=first.header,
        child_frame_id=second.child_frame_id,
        transform=_transform(translation, rotation * _rotation(second.transform.rotation)),
    )


def inverse_transform(message: TransformStamped) -> TransformStamped:
    """Return B←A from A←B, swapping frame names and preserving the exact stamp."""
    rotation = _rotation(message.transform.rotation).inv()
    return TransformStamped(
        header=Header(stamp=message.header.stamp, frame_id=message.child_frame_id),
        child_frame_id=message.header.frame_id,
        transform=_transform(-rotation.apply(_translation(message.transform)), rotation),
    )


def pose_from_transform(message: TransformStamped) -> PoseStamped:
    """Copy the child frame's pose in the parent frame into a generated PoseStamped."""
    translation = message.transform.translation
    return PoseStamped(
        header=message.header,
        pose=Pose(
            position=Point(x=translation.x, y=translation.y, z=translation.z),
            orientation=message.transform.rotation,
        ),
    )


def transform_from_pose(message: PoseStamped, *, child_frame_id: str) -> TransformStamped:
    """Copy a stamped pose into TF with an explicit child and unchanged source header."""
    position = message.pose.position
    return TransformStamped(
        header=message.header,
        child_frame_id=child_frame_id,
        transform=Transform(
            translation=Vector3(x=position.x, y=position.y, z=position.z),
            rotation=message.pose.orientation,
        ),
    )


def pose_matrix(pose: Pose) -> NDArray[np.float64]:
    """Copy a generated pose into a homogeneous matrix, rejecting invalid geometry."""
    values = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
    if not np.isfinite(values).all():
        raise ValueError("Position components must be finite")
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = _rotation(pose.orientation).as_matrix()
    result[:3, 3] = values
    return result


def quaternion_from_matrix(matrix: NDArray[np.float64]) -> Quaternion:
    """Convert a finite proper 3-by-3 rotation matrix into a generated quaternion."""
    values = np.asarray(matrix, dtype=np.float64)
    if values.shape != (3, 3) or not np.isfinite(values).all():
        raise ValueError("Rotation matrix must be finite and 3-by-3")
    if not np.allclose(values.T @ values, np.eye(3), atol=1e-6, rtol=0.0) or not np.isclose(
        np.linalg.det(values), 1.0, atol=1e-6, rtol=0.0
    ):
        raise ValueError("Rotation matrix must be orthonormal with determinant +1")
    x, y, z, w = Rotation.from_matrix(values).as_quat()
    return Quaternion(x=x, y=y, z=z, w=w)


def pose_from_matrix(matrix: NDArray[np.float64]) -> Pose:
    """Copy a finite rigid 4-by-4 transform into a generated pose."""
    values = np.asarray(matrix, dtype=np.float64)
    if values.shape != (4, 4) or not np.isfinite(values).all():
        raise ValueError("Pose matrix must be finite and 4-by-4")
    if not np.allclose(values[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9, rtol=0.0):
        raise ValueError("Pose matrix must have homogeneous last row [0, 0, 0, 1]")
    return Pose(
        position=Point(x=values[0, 3], y=values[1, 3], z=values[2, 3]),
        orientation=quaternion_from_matrix(values[:3, :3]),
    )


def quaternion_euler(rotation: Quaternion) -> tuple[float, float, float]:
    """Return fixed-axis XYZ Euler angles in radians for a generated quaternion."""
    roll, pitch, yaw = _rotation(rotation).as_euler("xyz")
    return float(roll), float(pitch), float(yaw)


def translate_pose_local(pose: Pose, offset: Vector3) -> Pose:
    """Copy a pose displaced by an offset expressed in its local axes."""
    matrix = pose_matrix(pose)
    values = np.array([offset.x, offset.y, offset.z], dtype=np.float64)
    if not np.isfinite(values).all():
        raise ValueError("Offset components must be finite")
    delta = matrix[:3, :3] @ values
    return Pose(
        position=Point(
            x=pose.position.x + delta[0], y=pose.position.y + delta[1], z=pose.position.z + delta[2]
        ),
        orientation=pose.orientation,
    )
