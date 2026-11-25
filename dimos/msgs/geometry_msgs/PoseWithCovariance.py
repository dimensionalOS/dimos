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

from typing import TYPE_CHECKING, Any, TypeAlias

from dimos_lcm.geometry_msgs import (
    PoseWithCovariance as LCMPoseWithCovariance,
)
import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose, PoseConvertable

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

# Types that can be converted to/from PoseWithCovariance
PoseWithCovarianceConvertable: TypeAlias = (
    tuple[PoseConvertable, list[float] | np.ndarray]
    | LCMPoseWithCovariance
    | dict[str, PoseConvertable | list[float] | np.ndarray]
)

COVARIANCE_SIZE = 36


def _is_value_covariance_pair(value: Any) -> bool:
    """True for a 2-element (value, covariance) pair, where covariance is 36 long."""
    if not isinstance(value, tuple | list) or len(value) != 2:
        return False
    covariance = value[1]
    return isinstance(covariance, list | np.ndarray) and np.size(covariance) == COVARIANCE_SIZE


class PoseWithCovariance(LCMPoseWithCovariance):  # type: ignore[misc]
    pose: Pose
    covariance: np.ndarray[tuple[int], np.dtype[np.floating[Any]]]
    msg_name = "geometry_msgs.PoseWithCovariance"

    def __init__(
        self,
        pose: Pose | PoseConvertable | PoseWithCovarianceConvertable | None = None,
        covariance: list[float] | np.ndarray | None = None,
    ) -> None:
        """Initialize a pose with covariance.

        Supported forms:
            PoseWithCovariance()                        # origin, zero covariance
            PoseWithCovariance(pose)
            PoseWithCovariance(pose, covariance)
            PoseWithCovariance(pose=..., covariance=...)
            PoseWithCovariance((pose, covariance))      # pair
            PoseWithCovariance({"pose": ..., "covariance": ...})
            PoseWithCovariance(other)                   # copy constructor
            PoseWithCovariance(lcm_pose_with_cov)       # from LCM message
        """
        source: Any = pose
        cov: Any = covariance

        # PoseWithCovariance before LCMPoseWithCovariance (it is a subclass).
        if isinstance(source, PoseWithCovariance):
            if cov is None:
                cov = np.array(source.covariance).copy()
            source = Pose(source.pose)  # copy constructor: don't alias the source pose
        elif isinstance(source, LCMPoseWithCovariance):
            if cov is None:
                cov = np.array(source.covariance)
            source = source.pose
        elif isinstance(source, dict) and "pose" in source:
            cov = source.get("covariance", cov)
            source = source["pose"]
        elif _is_value_covariance_pair(source):
            source, cov = source

        self.pose = source if isinstance(source, Pose) else Pose(source)
        self.covariance = (
            np.zeros(COVARIANCE_SIZE)
            if cov is None
            else np.array(cov, dtype=float).reshape(COVARIANCE_SIZE)
        )

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
        if name == "covariance" and not isinstance(value, np.ndarray):
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
    def covariance_matrix(self) -> np.ndarray:
        """Get covariance as 6x6 matrix."""
        return self.covariance.reshape(6, 6)

    @covariance_matrix.setter
    def covariance_matrix(self, value: np.ndarray) -> None:
        """Set covariance from 6x6 matrix."""
        self.covariance = np.array(value).reshape(36)

    def __repr__(self) -> str:
        return f"PoseWithCovariance(pose={self.pose!r}, covariance=<{self.covariance.shape[0] if isinstance(self.covariance, np.ndarray) else len(self.covariance)} elements>)"

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
        return self.pose == other.pose and np.allclose(self.covariance, other.covariance)

    def lcm_encode(self) -> bytes:
        """Encode to LCM binary format."""
        lcm_msg = LCMPoseWithCovariance()
        lcm_msg.pose = self.pose
        # LCM expects list, not numpy array
        if isinstance(self.covariance, np.ndarray):
            lcm_msg.covariance = self.covariance.tolist()
        else:
            lcm_msg.covariance = list(self.covariance)
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
