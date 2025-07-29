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

from dimos_lcm.geometry_msgs import Point as LCMPoint
from dimos_lcm.geometry_msgs import Pose as LCMPose
from dimos_lcm.geometry_msgs import PoseStamped as LCMPoseStamped
from dimos_lcm.geometry_msgs import Quaternion as LCMQuaternion
from dimos_lcm.nav_msgs import Path as LCMPath
from dimos_lcm.std_msgs import Header as LCMHeader
from dimos_lcm.std_msgs import Time as LCMTime

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion, QuaternionConvertable
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3, VectorConvertable
from dimos.types.timestamped import Timestamped


def sec_nsec(ts):
    s = int(ts)
    return [s, int((ts - s) * 1_000_000_000)]


class Path(Timestamped):
    msg_name = "nav_msgs.Path"
    ts: float
    frame_id: str
    poses: list[PoseStamped]

    def __init__(
        self,
        ts: float = 0.0,
        frame_id: str = "world",
        poses: list[PoseStamped] | None = None,
        **kwargs,
    ) -> None:
        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()
        self.poses = poses if poses is not None else []

    def __len__(self) -> int:
        """Return the number of poses in the path."""
        return len(self.poses)

    def __bool__(self) -> bool:
        """Return True if path has poses."""
        return len(self.poses) > 0

    def head(self) -> PoseStamped | None:
        """Return the first pose in the path, or None if empty."""
        return self.poses[0] if self.poses else None

    def last(self) -> PoseStamped | None:
        """Return the last pose in the path, or None if empty."""
        return self.poses[-1] if self.poses else None

    def tail(self) -> "Path":
        """Return a new Path with all poses except the first."""
        return Path(ts=self.ts, frame_id=self.frame_id, poses=self.poses[1:] if self.poses else [])

    def push(self, pose: PoseStamped) -> "Path":
        """Return a new Path with the pose appended (immutable)."""
        return Path(ts=self.ts, frame_id=self.frame_id, poses=self.poses + [pose])

    def push_mut(self, pose: PoseStamped) -> None:
        """Append a pose to this path (mutable)."""
        self.poses.append(pose)

    def lcm_encode(self) -> bytes:
        """Encode Path to LCM bytes."""
        lcm_msg = LCMPath()

        # Set poses
        lcm_msg.poses_length = len(self.poses)
        lcm_poses = []  # Build list separately to avoid LCM library reuse issues
        for pose in self.poses:
            lcm_pose = LCMPoseStamped()
            # Create new pose objects to avoid LCM library reuse bug
            lcm_pose.pose = LCMPose()
            lcm_pose.pose.position = LCMPoint()
            lcm_pose.pose.orientation = LCMQuaternion()

            # Set the pose geometry data
            lcm_pose.pose.position.x = pose.x
            lcm_pose.pose.position.y = pose.y
            lcm_pose.pose.position.z = pose.z
            lcm_pose.pose.orientation.x = pose.orientation.x
            lcm_pose.pose.orientation.y = pose.orientation.y
            lcm_pose.pose.orientation.z = pose.orientation.z
            lcm_pose.pose.orientation.w = pose.orientation.w

            # Create new header to avoid reuse
            lcm_pose.header = LCMHeader()
            lcm_pose.header.stamp = LCMTime()

            # Set the header with pose timestamp but path's frame_id
            [lcm_pose.header.stamp.sec, lcm_pose.header.stamp.nsec] = sec_nsec(pose.ts)
            lcm_pose.header.frame_id = self.frame_id  # All poses use path's frame_id
            lcm_poses.append(lcm_pose)
        lcm_msg.poses = lcm_poses

        # Set header with path's own timestamp
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = sec_nsec(self.ts)
        lcm_msg.header.frame_id = self.frame_id

        return lcm_msg.lcm_encode()

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> "Path":
        """Decode LCM bytes to Path."""
        lcm_msg = LCMPath.lcm_decode(data)

        # Decode header
        header_ts = lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000)
        frame_id = lcm_msg.header.frame_id

        # Decode poses - all use the path's frame_id
        poses = []
        for lcm_pose in lcm_msg.poses:
            pose = PoseStamped(
                ts=lcm_pose.header.stamp.sec + (lcm_pose.header.stamp.nsec / 1_000_000_000),
                frame_id=frame_id,  # Use path's frame_id for all poses
                position=[
                    lcm_pose.pose.position.x,
                    lcm_pose.pose.position.y,
                    lcm_pose.pose.position.z,
                ],
                orientation=[
                    lcm_pose.pose.orientation.x,
                    lcm_pose.pose.orientation.y,
                    lcm_pose.pose.orientation.z,
                    lcm_pose.pose.orientation.w,
                ],
            )
            poses.append(pose)

        # Use header timestamp for the path
        return cls(ts=header_ts, frame_id=frame_id, poses=poses)

    def __str__(self) -> str:
        """String representation of Path."""
        return f"Path(frame_id='{self.frame_id}', poses={len(self.poses)})"

    def __getitem__(self, index: int | slice) -> PoseStamped | list[PoseStamped]:
        """Allow indexing and slicing of poses."""
        return self.poses[index]

    def __iter__(self):
        """Allow iteration over poses."""
        return iter(self.poses)

    def slice(self, start: int, end: int | None = None) -> "Path":
        """Return a new Path with a slice of poses."""
        return Path(ts=self.ts, frame_id=self.frame_id, poses=self.poses[start:end])

    def extend(self, other: "Path") -> "Path":
        """Return a new Path with poses from both paths (immutable)."""
        return Path(ts=self.ts, frame_id=self.frame_id, poses=self.poses + other.poses)

    def extend_mut(self, other: "Path") -> None:
        """Extend this path with poses from another path (mutable)."""
        self.poses.extend(other.poses)

    def reverse(self) -> "Path":
        """Return a new Path with poses in reverse order."""
        return Path(ts=self.ts, frame_id=self.frame_id, poses=list(reversed(self.poses)))

    def clear(self) -> None:
        """Clear all poses from this path (mutable)."""
        self.poses.clear()
