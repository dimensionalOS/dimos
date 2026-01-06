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
from typing import TYPE_CHECKING, TypeAlias

from dimos_lcm.nav_msgs import Odometry as LCMOdometry  # type: ignore[import-untyped]
import numpy as np
from plum import dispatch

try:
    from nav_msgs.msg import Odometry as ROSOdometry  # type: ignore[attr-defined, import-untyped]
except ImportError:
    ROSOdometry = None  # type: ignore[assignment, misc]

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistWithCovariance import TwistWithCovariance
from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

# Types that can be converted to/from Odometry
OdometryConvertable: TypeAlias = (
    LCMOdometry | dict[str, float | str | PoseWithCovariance | TwistWithCovariance | Pose | Twist]
)


def sec_nsec(ts):  # type: ignore[no-untyped-def]
    s = int(ts)
    return [s, int((ts - s) * 1_000_000_000)]


class Odometry(LCMOdometry, Timestamped):  # type: ignore[misc]
    pose: PoseWithCovariance
    twist: TwistWithCovariance
    msg_name = "nav_msgs.Odometry"
    ts: float
    frame_id: str
    child_frame_id: str

    @dispatch
    def __init__(
        self,
        ts: float = 0.0,
        frame_id: str = "",
        child_frame_id: str = "",
        pose: PoseWithCovariance | Pose | None = None,
        twist: TwistWithCovariance | Twist | None = None,
    ) -> None:
        """Initialize with timestamp, frame IDs, pose and twist.

        Args:
            ts: Timestamp in seconds (defaults to current time if 0)
            frame_id: Reference frame ID (e.g., "odom", "map")
            child_frame_id: Child frame ID (e.g., "base_link", "base_footprint")
            pose: Pose with covariance (or just Pose, covariance will be zero)
            twist: Twist with covariance (or just Twist, covariance will be zero)
        """
        self.ts = ts if ts != 0 else time.time()
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id

        # Handle pose
        if pose is None:
            self.pose = PoseWithCovariance()
        elif isinstance(pose, PoseWithCovariance):
            self.pose = pose
        elif isinstance(pose, Pose):
            self.pose = PoseWithCovariance(pose)
        else:
            self.pose = PoseWithCovariance(Pose(pose))

        # Handle twist
        if twist is None:
            self.twist = TwistWithCovariance()
        elif isinstance(twist, TwistWithCovariance):
            self.twist = twist
        elif isinstance(twist, Twist):
            self.twist = TwistWithCovariance(twist)
        else:
            self.twist = TwistWithCovariance(Twist(twist))

    @dispatch  # type: ignore[no-redef]
    def __init__(self, odometry: Odometry) -> None:
        """Initialize from another Odometry (copy constructor)."""
        self.ts = odometry.ts
        self.frame_id = odometry.frame_id
        self.child_frame_id = odometry.child_frame_id
        self.pose = PoseWithCovariance(odometry.pose)
        self.twist = TwistWithCovariance(odometry.twist)

    @dispatch  # type: ignore[no-redef]
    def __init__(self, lcm_odometry: LCMOdometry) -> None:
        """Initialize from an LCM Odometry."""
        self.ts = lcm_odometry.header.stamp.sec + (lcm_odometry.header.stamp.nsec / 1_000_000_000)
        self.frame_id = lcm_odometry.header.frame_id
        self.child_frame_id = lcm_odometry.child_frame_id
        self.pose = PoseWithCovariance(lcm_odometry.pose)
        self.twist = TwistWithCovariance(lcm_odometry.twist)

    @dispatch  # type: ignore[no-redef]
    def __init__(
        self,
        odometry_dict: dict[
            str, float | str | PoseWithCovariance | TwistWithCovariance | Pose | Twist
        ],
    ) -> None:
        """Initialize from a dictionary."""
        self.ts = odometry_dict.get("ts", odometry_dict.get("timestamp", time.time()))
        self.frame_id = odometry_dict.get("frame_id", "")
        self.child_frame_id = odometry_dict.get("child_frame_id", "")

        # Handle pose
        pose = odometry_dict.get("pose")
        if pose is None:
            self.pose = PoseWithCovariance()
        elif isinstance(pose, PoseWithCovariance):
            self.pose = pose
        elif isinstance(pose, Pose):
            self.pose = PoseWithCovariance(pose)
        else:
            self.pose = PoseWithCovariance(Pose(pose))

        # Handle twist
        twist = odometry_dict.get("twist")
        if twist is None:
            self.twist = TwistWithCovariance()
        elif isinstance(twist, TwistWithCovariance):
            self.twist = twist
        elif isinstance(twist, Twist):
            self.twist = TwistWithCovariance(twist)
        else:
            self.twist = TwistWithCovariance(Twist(twist))

    @property
    def position(self) -> Vector3:
        """Get position from pose."""
        return self.pose.position

    @property
    def orientation(self):  # type: ignore[no-untyped-def]
        """Get orientation from pose."""
        return self.pose.orientation

    @property
    def linear_velocity(self) -> Vector3:
        """Get linear velocity from twist."""
        return self.twist.linear

    @property
    def angular_velocity(self) -> Vector3:
        """Get angular velocity from twist."""
        return self.twist.angular

    @property
    def x(self) -> float:
        """X position."""
        return self.pose.x

    @property
    def y(self) -> float:
        """Y position."""
        return self.pose.y

    @property
    def z(self) -> float:
        """Z position."""
        return self.pose.z

    @property
    def vx(self) -> float:
        """Linear velocity in X."""
        return self.twist.linear.x

    @property
    def vy(self) -> float:
        """Linear velocity in Y."""
        return self.twist.linear.y

    @property
    def vz(self) -> float:
        """Linear velocity in Z."""
        return self.twist.linear.z

    @property
    def wx(self) -> float:
        """Angular velocity around X (roll rate)."""
        return self.twist.angular.x

    @property
    def wy(self) -> float:
        """Angular velocity around Y (pitch rate)."""
        return self.twist.angular.y

    @property
    def wz(self) -> float:
        """Angular velocity around Z (yaw rate)."""
        return self.twist.angular.z

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

    def __repr__(self) -> str:
        return (
            f"Odometry(ts={self.ts:.6f}, frame_id='{self.frame_id}', "
            f"child_frame_id='{self.child_frame_id}', pose={self.pose!r}, twist={self.twist!r})"
        )

    def __str__(self) -> str:
        return (
            f"Odometry:\n"
            f"  Timestamp: {self.ts:.6f}\n"
            f"  Frame: {self.frame_id} -> {self.child_frame_id}\n"
            f"  Position: [{self.x:.3f}, {self.y:.3f}, {self.z:.3f}]\n"
            f"  Orientation: [roll={self.roll:.3f}, pitch={self.pitch:.3f}, yaw={self.yaw:.3f}]\n"
            f"  Linear Velocity: [{self.vx:.3f}, {self.vy:.3f}, {self.vz:.3f}]\n"
            f"  Angular Velocity: [{self.wx:.3f}, {self.wy:.3f}, {self.wz:.3f}]"
        )

    def __eq__(self, other) -> bool:  # type: ignore[no-untyped-def]
        """Check if two Odometry messages are equal."""
        if not isinstance(other, Odometry):
            return False
        return (
            abs(self.ts - other.ts) < 1e-6
            and self.frame_id == other.frame_id
            and self.child_frame_id == other.child_frame_id
            and self.pose == other.pose
            and self.twist == other.twist
        )

    def lcm_encode(self) -> bytes:
        """Encode to LCM binary format."""
        lcm_msg = LCMOdometry()

        # Set header
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = sec_nsec(self.ts)  # type: ignore[no-untyped-call]
        lcm_msg.header.frame_id = self.frame_id
        lcm_msg.child_frame_id = self.child_frame_id

        # Set pose with covariance
        lcm_msg.pose.pose = self.pose.pose
        if isinstance(self.pose.covariance, np.ndarray):  # type: ignore[has-type]
            lcm_msg.pose.covariance = self.pose.covariance.tolist()  # type: ignore[has-type]
        else:
            lcm_msg.pose.covariance = list(self.pose.covariance)  # type: ignore[has-type]

        # Set twist with covariance
        lcm_msg.twist.twist = self.twist.twist
        if isinstance(self.twist.covariance, np.ndarray):  # type: ignore[has-type]
            lcm_msg.twist.covariance = self.twist.covariance.tolist()  # type: ignore[has-type]
        else:
            lcm_msg.twist.covariance = list(self.twist.covariance)  # type: ignore[has-type]

        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> Odometry:
        """Decode from LCM binary format."""
        lcm_msg = LCMOdometry.lcm_decode(data)

        # Extract timestamp
        ts = lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000)

        # Create pose with covariance
        pose = Pose(
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
        )
        pose_with_cov = PoseWithCovariance(pose, lcm_msg.pose.covariance)

        # Create twist with covariance
        twist = Twist(
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
        )
        twist_with_cov = TwistWithCovariance(twist, lcm_msg.twist.covariance)

        return cls(
            ts=ts,
            frame_id=lcm_msg.header.frame_id,
            child_frame_id=lcm_msg.child_frame_id,
            pose=pose_with_cov,
            twist=twist_with_cov,
        )

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSOdometry) -> Odometry:
        """Create an Odometry from a ROS nav_msgs/Odometry message.

        Args:
            ros_msg: ROS Odometry message

        Returns:
            Odometry instance
        """

        # Convert timestamp from ROS header
        ts = ros_msg.header.stamp.sec + (ros_msg.header.stamp.nanosec / 1_000_000_000)

        # Convert pose and twist with covariance
        pose_with_cov = PoseWithCovariance.from_ros_msg(ros_msg.pose)
        twist_with_cov = TwistWithCovariance.from_ros_msg(ros_msg.twist)

        return cls(
            ts=ts,
            frame_id=ros_msg.header.frame_id,
            child_frame_id=ros_msg.child_frame_id,
            pose=pose_with_cov,
            twist=twist_with_cov,
        )

    def to_ros_msg(self) -> ROSOdometry:
        """Convert to a ROS nav_msgs/Odometry message.

        Returns:
            ROS Odometry message
        """

        ros_msg = ROSOdometry()  # type: ignore[no-untyped-call]

        # Set header
        ros_msg.header.frame_id = self.frame_id
        ros_msg.header.stamp.sec = int(self.ts)
        ros_msg.header.stamp.nanosec = int((self.ts - int(self.ts)) * 1_000_000_000)

        # Set child frame ID
        ros_msg.child_frame_id = self.child_frame_id

        # Set pose with covariance
        ros_msg.pose = self.pose.to_ros_msg()

        # Set twist with covariance
        ros_msg.twist = self.twist.to_ros_msg()

        return ros_msg
