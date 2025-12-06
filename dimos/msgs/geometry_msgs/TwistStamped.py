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
from typing import BinaryIO, TypeAlias

from dimos_lcm.geometry_msgs import TwistStamped as LCMTwistStamped  # type: ignore[import-untyped]
from plum import dispatch

try:
    from geometry_msgs.msg import (  # type: ignore[import-untyped]
        TwistStamped as ROSTwistStamped,  # type: ignore[attr-defined]
    )
except ImportError:
    ROSTwistStamped = None  # type: ignore[assignment, misc]

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import VectorConvertable
from dimos.types.timestamped import Timestamped

# Types that can be converted to/from TwistStamped
TwistConvertable: TypeAlias = (
    tuple[VectorConvertable, VectorConvertable] | LCMTwistStamped | dict[str, VectorConvertable]
)


def sec_nsec(ts):  # type: ignore[no-untyped-def]
    s = int(ts)
    return [s, int((ts - s) * 1_000_000_000)]


class TwistStamped(Twist, Timestamped):
    msg_name = "geometry_msgs.TwistStamped"
    ts: float
    frame_id: str

    @dispatch
    def __init__(self, ts: float = 0.0, frame_id: str = "", **kwargs) -> None:  # type: ignore[no-untyped-def]
        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()
        super().__init__(**kwargs)

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMTwistStamped()
        lcm_msg.twist = self
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = sec_nsec(self.ts)  # type: ignore[no-untyped-call]
        lcm_msg.header.frame_id = self.frame_id
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> TwistStamped:
        lcm_msg = LCMTwistStamped.lcm_decode(data)
        return cls(
            ts=lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000),
            frame_id=lcm_msg.header.frame_id,
            linear=[lcm_msg.twist.linear.x, lcm_msg.twist.linear.y, lcm_msg.twist.linear.z],
            angular=[lcm_msg.twist.angular.x, lcm_msg.twist.angular.y, lcm_msg.twist.angular.z],
        )

    def __str__(self) -> str:
        return (
            f"TwistStamped(linear=[{self.linear.x:.3f}, {self.linear.y:.3f}, {self.linear.z:.3f}], "
            f"angular=[{self.angular.x:.3f}, {self.angular.y:.3f}, {self.angular.z:.3f}])"
        )

    @classmethod
    def from_ros_msg(cls, ros_msg: ROSTwistStamped) -> TwistStamped:  # type: ignore[override]
        """Create a TwistStamped from a ROS geometry_msgs/TwistStamped message.

        Args:
            ros_msg: ROS TwistStamped message

        Returns:
            TwistStamped instance
        """

        # Convert timestamp from ROS header
        ts = ros_msg.header.stamp.sec + (ros_msg.header.stamp.nanosec / 1_000_000_000)

        # Convert twist
        twist = Twist.from_ros_msg(ros_msg.twist)

        return cls(
            ts=ts,
            frame_id=ros_msg.header.frame_id,
            linear=twist.linear,
            angular=twist.angular,
        )

    def to_ros_msg(self) -> ROSTwistStamped:  # type: ignore[override]
        """Convert to a ROS geometry_msgs/TwistStamped message.

        Returns:
            ROS TwistStamped message
        """

        ros_msg = ROSTwistStamped()  # type: ignore[no-untyped-call]

        # Set header
        ros_msg.header.frame_id = self.frame_id
        ros_msg.header.stamp.sec = int(self.ts)
        ros_msg.header.stamp.nanosec = int((self.ts - int(self.ts)) * 1_000_000_000)

        # Set twist
        ros_msg.twist = Twist.to_ros_msg(self)

        return ros_msg
