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
from typing import TypeAlias

from dimos_lcm.geometry_msgs import (  # type: ignore[import-untyped]
    TwistWithCovarianceStamped as LCMTwistWithCovarianceStamped,
)
import numpy as np
from plum import dispatch

try:
    from geometry_msgs.msg import (  # type: ignore[attr-defined, import-untyped]
        TwistWithCovarianceStamped as ROSTwistWithCovarianceStamped,
    )
except ImportError:
    ROSTwistWithCovarianceStamped = None  # type: ignore[assignment, misc]

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistWithCovariance import TwistWithCovariance
from dimos.msgs.geometry_msgs.Vector3 import VectorConvertable
from dimos.types.timestamped import Timestamped

# Types that can be converted to/from TwistWithCovarianceStamped
TwistWithCovarianceStampedConvertable: TypeAlias = (
    tuple[Twist | tuple[VectorConvertable, VectorConvertable], list[float] | np.ndarray]  # type: ignore[type-arg]
    | LCMTwistWithCovarianceStamped
    | dict[
        str,
        Twist
        | tuple[VectorConvertable, VectorConvertable]
        | list[float]
        | np.ndarray  # type: ignore[type-arg]
        | float
        | str,
    ]
)


def sec_nsec(ts):  # type: ignore[no-untyped-def]
    s = int(ts)
    return [s, int((ts - s) * 1_000_000_000)]


class TwistWithCovarianceStamped(TwistWithCovariance, Timestamped):
    msg_name = "geometry_msgs.TwistWithCovarianceStamped"
    ts: float
    frame_id: str

    @dispatch
    def __init__(self, ts: float = 0.0, frame_id: str = "", **kwargs) -> None:
        """Initialize with timestamp and frame_id."""
        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()
        super().__init__(**kwargs)

    @dispatch  # type: ignore[no-redef]
    def __init__(
        self,
        ts: float = 0.0,
        frame_id: str = "",
        twist: Twist | tuple[VectorConvertable, VectorConvertable] | None = None,
        covariance: list[float] | np.ndarray | None = None,  # type: ignore[type-arg]
    ) -> None:
        """Initialize with timestamp, frame_id, twist and covariance."""
        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()
        if twist is None:
            super().__init__()
        else:
            super().__init__(twist, covariance)

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMTwistWithCovarianceStamped()
        lcm_msg.twist.twist = self.twist
        # LCM expects list, not numpy array
        if isinstance(self.covariance, np.ndarray):  # type: ignore[has-type]
            lcm_msg.twist.covariance = self.covariance.tolist()  # type: ignore[has-type]
        else:
            lcm_msg.twist.covariance = list(self.covariance)  # type: ignore[has-type]
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = sec_nsec(self.ts)  # type: ignore[no-untyped-call]
        lcm_msg.header.frame_id = self.frame_id
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> TwistWithCovarianceStamped:
        lcm_msg = LCMTwistWithCovarianceStamped.lcm_decode(data)
        return cls(
            ts=lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000),
            frame_id=lcm_msg.header.frame_id,
            twist=Twist(
                linear=[
                    lcm_msg.twist.twist.linear.x,
                    lcm_msg.twist.twist.linear.y,
                    lcm_msg.twist.twist.linear.z,
                ],
                angular=[
                    lcm_msg.twist.twist.angular.x,
                    lcm_msg.twist.twist.angular.y,
                    lcm_msg.twist.twist.angular.z,
                ],
            ),
            covariance=lcm_msg.twist.covariance,
        )

    def __str__(self) -> str:
        return (
            f"TwistWithCovarianceStamped(linear=[{self.linear.x:.3f}, {self.linear.y:.3f}, {self.linear.z:.3f}], "
            f"angular=[{self.angular.x:.3f}, {self.angular.y:.3f}, {self.angular.z:.3f}], "
            f"cov_trace={np.trace(self.covariance_matrix):.3f})"
        )

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSTwistWithCovarianceStamped) -> TwistWithCovarianceStamped:  # type: ignore[override]
        """Create a TwistWithCovarianceStamped from a ROS geometry_msgs/TwistWithCovarianceStamped message.

        Args:
            ros_msg: ROS TwistWithCovarianceStamped message

        Returns:
            TwistWithCovarianceStamped instance
        """

        # Convert timestamp from ROS header
        ts = ros_msg.header.stamp.sec + (ros_msg.header.stamp.nanosec / 1_000_000_000)

        # Convert twist with covariance
        twist_with_cov = TwistWithCovariance.from_ros_msg(ros_msg.twist)

        return cls(
            ts=ts,
            frame_id=ros_msg.header.frame_id,
            twist=twist_with_cov.twist,
            covariance=twist_with_cov.covariance,  # type: ignore[has-type]
        )

    def to_ros_msg(self) -> ROSTwistWithCovarianceStamped:  # type: ignore[override]
        """Convert to a ROS geometry_msgs/TwistWithCovarianceStamped message.

        Returns:
            ROS TwistWithCovarianceStamped message
        """

        ros_msg = ROSTwistWithCovarianceStamped()  # type: ignore[no-untyped-call]

        # Set header
        ros_msg.header.frame_id = self.frame_id
        ros_msg.header.stamp.sec = int(self.ts)
        ros_msg.header.stamp.nanosec = int((self.ts - int(self.ts)) * 1_000_000_000)

        # Set twist with covariance
        ros_msg.twist.twist = self.twist.to_ros_msg()
        # ROS expects list, not numpy array
        if isinstance(self.covariance, np.ndarray):  # type: ignore[has-type]
            ros_msg.twist.covariance = self.covariance.tolist()  # type: ignore[has-type]
        else:
            ros_msg.twist.covariance = list(self.covariance)  # type: ignore[has-type]

        return ros_msg
