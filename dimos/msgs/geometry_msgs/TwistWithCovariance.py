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

from typing import TypeAlias

from dimos_lcm.geometry_msgs import (  # type: ignore[import-untyped]
    TwistWithCovariance as LCMTwistWithCovariance,
)
import numpy as np
from plum import dispatch

try:
    from geometry_msgs.msg import (  # type: ignore[attr-defined, import-untyped]
        TwistWithCovariance as ROSTwistWithCovariance,
    )
except ImportError:
    ROSTwistWithCovariance = None  # type: ignore[assignment, misc]

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3, VectorConvertable

# Types that can be converted to/from TwistWithCovariance
TwistWithCovarianceConvertable: TypeAlias = (
    tuple[Twist | tuple[VectorConvertable, VectorConvertable], list[float] | np.ndarray]  # type: ignore[type-arg]
    | LCMTwistWithCovariance
    | dict[str, Twist | tuple[VectorConvertable, VectorConvertable] | list[float] | np.ndarray]  # type: ignore[type-arg]
)


class TwistWithCovariance(LCMTwistWithCovariance):  # type: ignore[misc]
    twist: Twist
    msg_name = "geometry_msgs.TwistWithCovariance"

    @dispatch
    def __init__(self) -> None:
        """Initialize with default twist and zero covariance."""
        self.twist = Twist()
        self.covariance = np.zeros(36)

    @dispatch  # type: ignore[no-redef]
    def __init__(
        self,
        twist: Twist | tuple[VectorConvertable, VectorConvertable],
        covariance: list[float] | np.ndarray | None = None,  # type: ignore[type-arg]
    ) -> None:
        """Initialize with twist and optional covariance."""
        if isinstance(twist, Twist):
            self.twist = twist
        else:
            # Assume it's a tuple of (linear, angular)
            self.twist = Twist(twist[0], twist[1])

        if covariance is None:
            self.covariance = np.zeros(36)
        else:
            self.covariance = np.array(covariance, dtype=float).reshape(36)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, twist_with_cov: TwistWithCovariance) -> None:
        """Initialize from another TwistWithCovariance (copy constructor)."""
        self.twist = Twist(twist_with_cov.twist)
        self.covariance = np.array(twist_with_cov.covariance).copy()

    @dispatch  # type: ignore[no-redef]
    def __init__(self, lcm_twist_with_cov: LCMTwistWithCovariance) -> None:
        """Initialize from an LCM TwistWithCovariance."""
        self.twist = Twist(lcm_twist_with_cov.twist)
        self.covariance = np.array(lcm_twist_with_cov.covariance)

    @dispatch  # type: ignore[no-redef]
    def __init__(
        self,
        twist_dict: dict[  # type: ignore[type-arg]
            str, Twist | tuple[VectorConvertable, VectorConvertable] | list[float] | np.ndarray
        ],
    ) -> None:
        """Initialize from a dictionary with 'twist' and 'covariance' keys."""
        twist = twist_dict["twist"]
        if isinstance(twist, Twist):
            self.twist = twist
        else:
            # Assume it's a tuple of (linear, angular)
            self.twist = Twist(twist[0], twist[1])

        covariance = twist_dict.get("covariance")
        if covariance is None:
            self.covariance = np.zeros(36)
        else:
            self.covariance = np.array(covariance, dtype=float).reshape(36)

    @dispatch  # type: ignore[no-redef]
    def __init__(
        self,
        twist_tuple: tuple[  # type: ignore[type-arg]
            Twist | tuple[VectorConvertable, VectorConvertable], list[float] | np.ndarray
        ],
    ) -> None:
        """Initialize from a tuple of (twist, covariance)."""
        twist = twist_tuple[0]
        if isinstance(twist, Twist):
            self.twist = twist
        else:
            # Assume it's a tuple of (linear, angular)
            self.twist = Twist(twist[0], twist[1])
        self.covariance = np.array(twist_tuple[1], dtype=float).reshape(36)

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
    def linear(self) -> Vector3:
        """Linear velocity vector."""
        return self.twist.linear

    @property
    def angular(self) -> Vector3:
        """Angular velocity vector."""
        return self.twist.angular

    @property
    def covariance_matrix(self) -> np.ndarray:  # type: ignore[type-arg]
        """Get covariance as 6x6 matrix."""
        return self.covariance.reshape(6, 6)  # type: ignore[has-type, no-any-return]

    @covariance_matrix.setter
    def covariance_matrix(self, value: np.ndarray) -> None:  # type: ignore[type-arg]
        """Set covariance from 6x6 matrix."""
        self.covariance = np.array(value).reshape(36)  # type: ignore[has-type]

    def __repr__(self) -> str:
        return f"TwistWithCovariance(twist={self.twist!r}, covariance=<{self.covariance.shape[0] if isinstance(self.covariance, np.ndarray) else len(self.covariance)} elements>)"  # type: ignore[has-type]

    def __str__(self) -> str:
        return (
            f"TwistWithCovariance(linear=[{self.linear.x:.3f}, {self.linear.y:.3f}, {self.linear.z:.3f}], "
            f"angular=[{self.angular.x:.3f}, {self.angular.y:.3f}, {self.angular.z:.3f}], "
            f"cov_trace={np.trace(self.covariance_matrix):.3f})"
        )

    def __eq__(self, other) -> bool:  # type: ignore[no-untyped-def]
        """Check if two TwistWithCovariance are equal."""
        if not isinstance(other, TwistWithCovariance):
            return False
        return self.twist == other.twist and np.allclose(self.covariance, other.covariance)  # type: ignore[has-type]

    def is_zero(self) -> bool:
        """Check if this is a zero twist (no linear or angular velocity)."""
        return self.twist.is_zero()

    def __bool__(self) -> bool:
        """Boolean conversion - False if zero twist, True otherwise."""
        return not self.is_zero()

    def lcm_encode(self) -> bytes:
        """Encode to LCM binary format."""
        lcm_msg = LCMTwistWithCovariance()
        lcm_msg.twist = self.twist
        # LCM expects list, not numpy array
        if isinstance(self.covariance, np.ndarray):  # type: ignore[has-type]
            lcm_msg.covariance = self.covariance.tolist()  # type: ignore[has-type]
        else:
            lcm_msg.covariance = list(self.covariance)  # type: ignore[has-type]
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> TwistWithCovariance:
        """Decode from LCM binary format."""
        lcm_msg = LCMTwistWithCovariance.lcm_decode(data)
        twist = Twist(
            linear=[lcm_msg.twist.linear.x, lcm_msg.twist.linear.y, lcm_msg.twist.linear.z],
            angular=[lcm_msg.twist.angular.x, lcm_msg.twist.angular.y, lcm_msg.twist.angular.z],
        )
        return cls(twist, lcm_msg.covariance)

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSTwistWithCovariance) -> TwistWithCovariance:
        """Create a TwistWithCovariance from a ROS geometry_msgs/TwistWithCovariance message.

        Args:
            ros_msg: ROS TwistWithCovariance message

        Returns:
            TwistWithCovariance instance
        """

        twist = Twist.from_ros_msg(ros_msg.twist)
        return cls(twist, list(ros_msg.covariance))

    def to_ros_msg(self) -> ROSTwistWithCovariance:
        """Convert to a ROS geometry_msgs/TwistWithCovariance message.

        Returns:
            ROS TwistWithCovariance message
        """

        ros_msg = ROSTwistWithCovariance()  # type: ignore[no-untyped-call]
        ros_msg.twist = self.twist.to_ros_msg()
        # ROS expects list, not numpy array
        if isinstance(self.covariance, np.ndarray):  # type: ignore[has-type]
            ros_msg.covariance = self.covariance.tolist()  # type: ignore[has-type]
        else:
            ros_msg.covariance = list(self.covariance)  # type: ignore[has-type]
        return ros_msg
