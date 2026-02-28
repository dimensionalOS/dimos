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

import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype

from dimos_lcm.nav_msgs import Odometry as LCMOdometry
import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistWithCovariance import TwistWithCovariance
from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Vector3 import Vector3


class Odometry(Timestamped):
    """Odometry message with pose, twist, and frame information."""

    msg_name = "nav_msgs.Odometry"

    def __init__(
        self,
        ts: float = 0.0,
        frame_id: str = "",
        child_frame_id: str = "",
        pose: PoseWithCovariance | Pose | None = None,
        twist: TwistWithCovariance | Twist | None = None,
    ) -> None:
        self.ts = ts if ts != 0 else time.time()
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id

        if pose is None:
            self.pose = PoseWithCovariance()
        elif isinstance(pose, Pose):
            self.pose = PoseWithCovariance(pose)
        else:
            self.pose = pose

        if twist is None:
            self.twist = TwistWithCovariance()
        elif isinstance(twist, Twist):
            self.twist = TwistWithCovariance(twist)
        else:
            self.twist = twist

    # -- Convenience properties --

    @property
    def position(self) -> Vector3:
        return self.pose.position

    @property
    def orientation(self) -> Quaternion:
        return self.pose.orientation

    @property
    def linear_velocity(self) -> Vector3:
        return self.twist.linear

    @property
    def angular_velocity(self) -> Vector3:
        return self.twist.angular

    @property
    def x(self) -> float:
        return self.pose.x

    @property
    def y(self) -> float:
        return self.pose.y

    @property
    def z(self) -> float:
        return self.pose.z

    @property
    def vx(self) -> float:
        return self.twist.linear.x

    @property
    def vy(self) -> float:
        return self.twist.linear.y

    @property
    def vz(self) -> float:
        return self.twist.linear.z

    @property
    def wx(self) -> float:
        return self.twist.angular.x

    @property
    def wy(self) -> float:
        return self.twist.angular.y

    @property
    def wz(self) -> float:
        return self.twist.angular.z

    @property
    def roll(self) -> float:
        return self.pose.roll

    @property
    def pitch(self) -> float:
        return self.pose.pitch

    @property
    def yaw(self) -> float:
        return self.pose.yaw

    # -- Serialization --

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMOdometry()

        lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec = self.ros_timestamp()
        lcm_msg.header.frame_id = self.frame_id
        lcm_msg.child_frame_id = self.child_frame_id

        lcm_msg.pose.pose = self.pose.pose
        lcm_msg.pose.covariance = list(np.asarray(self.pose.covariance))

        lcm_msg.twist.twist = self.twist.twist
        lcm_msg.twist.covariance = list(np.asarray(self.twist.covariance))

        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> Odometry:
        lcm_msg = LCMOdometry.lcm_decode(data)

        ts = lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000)

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

        return cls(
            ts=ts,
            frame_id=lcm_msg.header.frame_id,
            child_frame_id=lcm_msg.child_frame_id,
            pose=PoseWithCovariance(pose, lcm_msg.pose.covariance),
            twist=TwistWithCovariance(twist, lcm_msg.twist.covariance),
        )

    # -- Comparison / display --

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Odometry):
            return False
        return (
            abs(self.ts - other.ts) < 1e-6
            and self.frame_id == other.frame_id
            and self.child_frame_id == other.child_frame_id
            and self.pose == other.pose
            and self.twist == other.twist
        )

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

    def to_rerun(self) -> Archetype:
        """Convert to rerun Transform3D for visualizing the pose."""
        import rerun as rr

        return rr.Transform3D(
            translation=[self.x, self.y, self.z],
            rotation=rr.Quaternion(
                xyzw=[
                    self.orientation.x,
                    self.orientation.y,
                    self.orientation.z,
                    self.orientation.w,
                ]
            ),
        )
