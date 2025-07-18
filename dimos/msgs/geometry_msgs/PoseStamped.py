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

import struct
import time
from io import BytesIO
from typing import BinaryIO, TypeAlias

from dimos_lcm.geometry_msgs import PoseStamped as LCMPoseStamped
from dimos_lcm.std_msgs import Header as LCMHeader
from dimos_lcm.std_msgs import Time as LCMTime
from plum import dispatch

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion, QuaternionConvertable
from dimos.msgs.geometry_msgs.Vector3 import Vector3, VectorConvertable
from dimos.types.timestamped import Timestamped

# Types that can be converted to/from Pose
PoseConvertable: TypeAlias = (
    tuple[VectorConvertable, QuaternionConvertable]
    | LCMPoseStamped
    | dict[str, VectorConvertable | QuaternionConvertable]
)


def sec_nsec(ts):
    s = int(ts)
    return [s, int((ts - s) * 1_000_000_000)]


class PoseStamped(Pose, Timestamped):
    msg_name = "geometry_msgs.PoseStamped"
    ts: float
    frame_id: str

    @dispatch
    def __init__(self, ts: float = 0.0, frame_id: str = "", **kwargs) -> None:
        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()
        super().__init__(**kwargs)

    def lcm_encode(self) -> bytes:
        lcm_mgs = LCMPoseStamped()
        lcm_mgs.pose = self
        [lcm_mgs.header.stamp.sec, lcm_mgs.header.stamp.nsec] = sec_nsec(self.ts)
        lcm_mgs.header.frame_id = self.frame_id
        return lcm_mgs.lcm_encode()

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> PoseStamped:
        lcm_msg = LCMPoseStamped.lcm_decode(data)
        return cls(
            ts=lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000),
            frame_id=lcm_msg.header.frame_id,
            position=[lcm_msg.pose.position.x, lcm_msg.pose.position.y, lcm_msg.pose.position.z],
            orientation=[
                lcm_msg.pose.orientation.x,
                lcm_msg.pose.orientation.y,
                lcm_msg.pose.orientation.z,
                lcm_msg.pose.orientation.w,
            ],  # noqa: E501,
        )

    def __str__(self) -> str:
        return (
            f"PoseStamped(pos=[{self.x:.3f}, {self.y:.3f}, {self.z:.3f}], "
            f"euler=[{self.roll:.3f}, {self.pitch:.3f}, {self.yaw:.3f}])"
        )
