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
    PoseWithCovarianceStamped as LCMPoseWithCovarianceStamped,
)
import numpy as np
from plum import dispatch

try:
    from geometry_msgs.msg import (  # type: ignore[attr-defined, import-untyped]
        PoseWithCovarianceStamped as ROSPoseWithCovarianceStamped,
    )
except ImportError:
    ROSPoseWithCovarianceStamped = None  # type: ignore[assignment, misc]

from dimos.msgs.geometry_msgs.Pose import Pose, PoseConvertable
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance
from dimos.types.timestamped import Timestamped

# Types that can be converted to/from PoseWithCovarianceStamped
PoseWithCovarianceStampedConvertable: TypeAlias = (
    tuple[PoseConvertable, list[float] | np.ndarray]  # type: ignore[type-arg]
    | LCMPoseWithCovarianceStamped
    | dict[str, PoseConvertable | list[float] | np.ndarray | float | str]  # type: ignore[type-arg]
)


def sec_nsec(ts):  # type: ignore[no-untyped-def]
    s = int(ts)
    return [s, int((ts - s) * 1_000_000_000)]


class PoseWithCovarianceStamped(PoseWithCovariance, Timestamped):
    msg_name = "geometry_msgs.PoseWithCovarianceStamped"
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
        pose: Pose | PoseConvertable | None = None,
        covariance: list[float] | np.ndarray | None = None,  # type: ignore[type-arg]
    ) -> None:
        """Initialize with timestamp, frame_id, pose and covariance."""
        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()
        if pose is None:
            super().__init__()
        else:
            super().__init__(pose, covariance)

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMPoseWithCovarianceStamped()
        lcm_msg.pose.pose = self.pose
        # LCM expects list, not numpy array
        if isinstance(self.covariance, np.ndarray):  # type: ignore[has-type]
            lcm_msg.pose.covariance = self.covariance.tolist()  # type: ignore[has-type]
        else:
            lcm_msg.pose.covariance = list(self.covariance)  # type: ignore[has-type]
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = sec_nsec(self.ts)  # type: ignore[no-untyped-call]
        lcm_msg.header.frame_id = self.frame_id
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> PoseWithCovarianceStamped:
        lcm_msg = LCMPoseWithCovarianceStamped.lcm_decode(data)
        return cls(
            ts=lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000),
            frame_id=lcm_msg.header.frame_id,
            pose=Pose(
                position=[
                    lcm_msg.pose.pose.position.x,
                    lcm_msg.pose.pose.position.y,
                    lcm_msg.pose.pose.position.z,
                ],
                orientation=[
                    lcm_msg.pose.pose.orientation.x,
                    lcm_msg.pose.pose.orientation.y,
                    lcm_msg.pose.pose.orientation.z,
                    lcm_msg.pose.pose.orientation.w,
                ],
            ),
            covariance=lcm_msg.pose.covariance,
        )

    def __str__(self) -> str:
        return (
            f"PoseWithCovarianceStamped(pos=[{self.x:.3f}, {self.y:.3f}, {self.z:.3f}], "
            f"euler=[{self.roll:.3f}, {self.pitch:.3f}, {self.yaw:.3f}], "
            f"cov_trace={np.trace(self.covariance_matrix):.3f})"
        )

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSPoseWithCovarianceStamped) -> PoseWithCovarianceStamped:  # type: ignore[override]
        """Create a PoseWithCovarianceStamped from a ROS geometry_msgs/PoseWithCovarianceStamped message.

        Args:
            ros_msg: ROS PoseWithCovarianceStamped message

        Returns:
            PoseWithCovarianceStamped instance
        """

        # Convert timestamp from ROS header
        ts = ros_msg.header.stamp.sec + (ros_msg.header.stamp.nanosec / 1_000_000_000)

        # Convert pose with covariance
        pose_with_cov = PoseWithCovariance.from_ros_msg(ros_msg.pose)

        return cls(
            ts=ts,
            frame_id=ros_msg.header.frame_id,
            pose=pose_with_cov.pose,
            covariance=pose_with_cov.covariance,  # type: ignore[has-type]
        )

    def to_ros_msg(self) -> ROSPoseWithCovarianceStamped:  # type: ignore[override]
        """Convert to a ROS geometry_msgs/PoseWithCovarianceStamped message.

        Returns:
            ROS PoseWithCovarianceStamped message
        """

        ros_msg = ROSPoseWithCovarianceStamped()  # type: ignore[no-untyped-call]

        # Set header
        ros_msg.header.frame_id = self.frame_id
        ros_msg.header.stamp.sec = int(self.ts)
        ros_msg.header.stamp.nanosec = int((self.ts - int(self.ts)) * 1_000_000_000)

        # Set pose with covariance
        ros_msg.pose.pose = self.pose.to_ros_msg()
        # ROS expects list, not numpy array
        if isinstance(self.covariance, np.ndarray):  # type: ignore[has-type]
            ros_msg.pose.covariance = self.covariance.tolist()  # type: ignore[has-type]
        else:
            ros_msg.pose.covariance = list(self.covariance)  # type: ignore[has-type]

        return ros_msg
