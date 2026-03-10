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
from typing import TYPE_CHECKING, BinaryIO

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype

    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

from dimos_lcm.geometry_msgs import PointStamped as LCMPointStamped

from dimos.msgs.geometry_msgs.Point import Point
from dimos.types.timestamped import Timestamped


class PointStamped(Point, Timestamped):
    """A 3D point with timestamp and frame_id.

    Follows the same pattern as PoseStamped(Pose, Timestamped) and
    TwistStamped(Twist, Timestamped). Inherits x/y/z from Point
    (which inherits from LCMPoint).
    """

    msg_name = "geometry_msgs.PointStamped"
    ts: float
    frame_id: str

    def __init__(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        ts: float = 0.0,
        frame_id: str = "",
    ) -> None:
        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()
        super().__init__(float(x), float(y), float(z))

    # -- LCM encode / decode --

    def lcm_encode(self) -> bytes:
        """Encode to LCM binary format."""
        lcm_msg = LCMPointStamped()
        lcm_msg.point = self  # Works because Point inherits from LCMPoint
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = self.ros_timestamp()
        lcm_msg.header.frame_id = self.frame_id
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> PointStamped:
        """Decode from LCM binary format."""
        lcm_msg = LCMPointStamped.lcm_decode(data)
        return cls(
            x=lcm_msg.point.x,
            y=lcm_msg.point.y,
            z=lcm_msg.point.z,
            ts=lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000),
            frame_id=lcm_msg.header.frame_id,
        )

    # -- Conversion methods --

    def to_rerun(self) -> Archetype:
        """Convert to rerun Points3D archetype for visualization."""
        import rerun as rr

        return rr.Points3D(positions=[[self.x, self.y, self.z]])

    def to_pose_stamped(self) -> PoseStamped:
        """Convert to PoseStamped with identity quaternion orientation."""
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        return PoseStamped(
            ts=self.ts,
            frame_id=self.frame_id,
            position=[self.x, self.y, self.z],
            orientation=[0.0, 0.0, 0.0, 1.0],
        )

    # -- String representations --

    def __str__(self) -> str:
        return f"PointStamped(point=[{self.x:.3f}, {self.y:.3f}, {self.z:.3f}], frame_id={self.frame_id!r})"

    def __repr__(self) -> str:
        return f"PointStamped(x={self.x}, y={self.y}, z={self.z}, ts={self.ts}, frame_id={self.frame_id!r})"
