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

from typing import TYPE_CHECKING, TypeAlias

from dimos_lcm.geometry_msgs import (  # type: ignore[import-untyped]
    PoseWithCovariance as LCMPoseWithCovariance,
)
import numpy as np
from plum import dispatch

try:
    from geometry_msgs.msg import (  # type: ignore[attr-defined, import-untyped]
        PoseWithCovariance as ROSPoseWithCovariance,
    )
except ImportError:
    ROSPoseWithCovariance = None  # type: ignore[assignment, misc]

from dimos.msgs.geometry_msgs.Pose import Pose, PoseConvertable

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

# Types that can be converted to/from PoseWithCovariance
PoseWithCovarianceConvertable: TypeAlias = (
    tuple[PoseConvertable, list[float] | np.ndarray]  # type: ignore[type-arg]
    | LCMPoseWithCovariance
    | dict[str, PoseConvertable | list[float] | np.ndarray]  # type: ignore[type-arg]
)


class PoseWithCovariance(LCMPoseWithCovariance):  # type: ignore[misc]
    pose: Pose
    msg_name = "geometry_msgs.PoseWithCovariance"

    @dispatch
    def __init__(self) -> None:
        """Initialize with default pose and zero covariance."""
        self.pose = Pose()
        self.covariance = np.zeros(36)

    @dispatch  # type: ignore[no-redef]
    def __init__(
        self,
        pose: Pose | PoseConvertable,
        covariance: list[float] | np.ndarray | None = None,  # type: ignore[type-arg]
    ) -> None:
        """Initialize with pose and optional covariance."""
        self.pose = Pose(pose) if not isinstance(pose, Pose) else pose
        if covariance is None:
            self.covariance = np.zeros(36)
        else:
            self.covariance = np.array(covariance, dtype=float).reshape(36)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, pose_with_cov: PoseWithCovariance) -> None:
        """Initialize from another PoseWithCovariance (copy constructor)."""
        self.pose = Pose(pose_with_cov.pose)
        self.covariance = np.array(pose_with_cov.covariance).copy()

    @dispatch  # type: ignore[no-redef]
    def __init__(self, lcm_pose_with_cov: LCMPoseWithCovariance) -> None:
        """Initialize from an LCM PoseWithCovariance."""
        self.pose = Pose(lcm_pose_with_cov.pose)
        self.covariance = np.array(lcm_pose_with_cov.covariance)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, pose_dict: dict[str, PoseConvertable | list[float] | np.ndarray]) -> None:  # type: ignore[type-arg]
        """Initialize from a dictionary with 'pose' and 'covariance' keys."""
        self.pose = Pose(pose_dict["pose"])
        covariance = pose_dict.get("covariance")
        if covariance is None:
            self.covariance = np.zeros(36)
        else:
            self.covariance = np.array(covariance, dtype=float).reshape(36)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, pose_tuple: tuple[PoseConvertable, list[float] | np.ndarray]) -> None:  # type: ignore[type-arg]
        """Initialize from a tuple of (pose, covariance)."""
        self.pose = Pose(pose_tuple[0])
        self.covariance = np.array(pose_tuple[1], dtype=float).reshape(36)

    def __getattribute__(self, name: str):  # type: ignore[no-untyped-def]
        """Override to ensure covariance is always returned as numpy array."""
        if name == "covariance":
            cov = object.__getattribute__(self, "covariance")
            if not isinstance(cov, np.ndarray):
                return np.array(cov, dtype=float)
            return cov
        return super().__getattribute__(name)

    def __setattr__(self, name: str, value) -> None:  # type: ignore[no-untyped-def]
        """Override to ensure covariance is stored as numpy array."""
        if name == "covariance":
            if not isinstance(value, np.ndarray):
                value = np.array(value, dtype=float).reshape(36)
        super().__setattr__(name, value)

    @property
    def x(self) -> float:
        """X coordinate of position."""
        return self.pose.x

    @property
    def y(self) -> float:
        """Y coordinate of position."""
        return self.pose.y

    @property
    def z(self) -> float:
        """Z coordinate of position."""
        return self.pose.z

    @property
    def position(self) -> Vector3:
        """Position vector."""
        return self.pose.position

    @property
    def orientation(self) -> Quaternion:
        """Orientation quaternion."""
        return self.pose.orientation

    @property
    def roll(self) -> float:
        """Roll angle in radians."""
        return self.pose.roll

    @property
    def pitch(self) -> float:
        """Pitch angle in radians."""
        return self.pose.pitch

    @property
    def yaw(self) -> float:
        """Yaw angle in radians."""
        return self.pose.yaw

    @property
    def covariance_matrix(self) -> np.ndarray:  # type: ignore[type-arg]
        """Get covariance as 6x6 matrix."""
        return self.covariance.reshape(6, 6)  # type: ignore[has-type, no-any-return]

    @covariance_matrix.setter
    def covariance_matrix(self, value: np.ndarray) -> None:  # type: ignore[type-arg]
        """Set covariance from 6x6 matrix."""
        self.covariance = np.array(value).reshape(36)  # type: ignore[has-type]

    def __repr__(self) -> str:
        return f"PoseWithCovariance(pose={self.pose!r}, covariance=<{self.covariance.shape[0] if isinstance(self.covariance, np.ndarray) else len(self.covariance)} elements>)"  # type: ignore[has-type]

    def __str__(self) -> str:
        return (
            f"PoseWithCovariance(pos=[{self.x:.3f}, {self.y:.3f}, {self.z:.3f}], "
            f"euler=[{self.roll:.3f}, {self.pitch:.3f}, {self.yaw:.3f}], "
            f"cov_trace={np.trace(self.covariance_matrix):.3f})"
        )

    def __eq__(self, other) -> bool:  # type: ignore[no-untyped-def]
        """Check if two PoseWithCovariance are equal."""
        if not isinstance(other, PoseWithCovariance):
            return False
        return self.pose == other.pose and np.allclose(self.covariance, other.covariance)  # type: ignore[has-type]

    def lcm_encode(self) -> bytes:
        """Encode to LCM binary format."""
        lcm_msg = LCMPoseWithCovariance()
        lcm_msg.pose = self.pose
        # LCM expects list, not numpy array
        if isinstance(self.covariance, np.ndarray):  # type: ignore[has-type]
            lcm_msg.covariance = self.covariance.tolist()  # type: ignore[has-type]
        else:
            lcm_msg.covariance = list(self.covariance)  # type: ignore[has-type]
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> PoseWithCovariance:
        """Decode from LCM binary format."""
        lcm_msg = LCMPoseWithCovariance.lcm_decode(data)
        pose = Pose(
            position=[lcm_msg.pose.position.x, lcm_msg.pose.position.y, lcm_msg.pose.position.z],
            orientation=[
                lcm_msg.pose.orientation.x,
                lcm_msg.pose.orientation.y,
                lcm_msg.pose.orientation.z,
                lcm_msg.pose.orientation.w,
            ],
        )
        return cls(pose, lcm_msg.covariance)

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSPoseWithCovariance) -> PoseWithCovariance:
        """Create a PoseWithCovariance from a ROS geometry_msgs/PoseWithCovariance message.

        Args:
            ros_msg: ROS PoseWithCovariance message

        Returns:
            PoseWithCovariance instance
        """

        pose = Pose.from_ros_msg(ros_msg.pose)
        return cls(pose, list(ros_msg.covariance))

    def to_ros_msg(self) -> ROSPoseWithCovariance:
        """Convert to a ROS geometry_msgs/PoseWithCovariance message.

        Returns:
            ROS PoseWithCovariance message
        """

        ros_msg = ROSPoseWithCovariance()  # type: ignore[no-untyped-call]
        ros_msg.pose = self.pose.to_ros_msg()
        # ROS expects list, not numpy array
        if isinstance(self.covariance, np.ndarray):  # type: ignore[has-type]
            ros_msg.covariance = self.covariance.tolist()  # type: ignore[has-type]
        else:
            ros_msg.covariance = list(self.covariance)  # type: ignore[has-type]
        return ros_msg
