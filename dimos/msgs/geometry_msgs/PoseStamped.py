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

import struct
import time
from io import BytesIO
from typing import BinaryIO, TypeAlias

from lcm_msgs.geometry_msgs import PoseStamped as LCMPoseStamped
from lcm_msgs.std_msgs import Header as LCMHeader
from lcm_msgs.std_msgs import Time as LCMTime
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
    name = "geometry_msgs.PoseStamped"
    ts: float
    frame_id: str

    @dispatch
    def __init__(self, *args, ts: float = 0, frame_id: str = "", **kwargs) -> None:
        self.frame_id = frame_id
        self.ts = ts if ts is not 0 else time.time()
        super().__init__(*args, **kwargs)

    def lcm_encode(self) -> bytes:
        lcm_mgs = LCMPoseStamped()
        lcm_mgs.pose = self
        [lcm_mgs.header.stamp.sec, lcm_mgs.header.stamp.sec] = sec_nsec(self.ts)
        lcm_mgs.header.frame_id = self.frame_id

        return lcm_mgs.encode()

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO):
        lcm_msg = LCMPoseStamped.decode(data)
        return cls(
            pose=Pose(lcm_msg.pose),
            ts=lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000),
            frame_id=lcm_msg.header.frame_id,
        )
