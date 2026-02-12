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

"""PoseArray message type for Dimos."""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.msgs.std_msgs.Header import Header

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.msgs.geometry_msgs.Pose import Pose


class PoseArray:
    """
    An array of poses with a header for reference frame and timestamp.

    This is commonly used for representing multiple candidate positions,
    such as grasp poses, particle filter samples, or waypoints.
    """

    msg_name = "geometry_msgs.PoseArray"

    def __init__(self, header: Header | None = None, poses: list[Pose] | None = None) -> None:
        """
        Initialize a PoseArray.

        Args:
            header: Header with frame_id and timestamp
            poses: List of Pose objects
        """
        self.header = header if header is not None else Header()
        self.poses = poses if poses is not None else []

    def __repr__(self) -> str:
        return f"PoseArray(header={self.header!r}, poses={len(self.poses)} poses)"

    def __str__(self) -> str:
        return f"PoseArray(frame_id={self.header.frame_id}, num_poses={len(self.poses)})"

    def __len__(self) -> int:
        """Return the number of poses in the array."""
        return len(self.poses)

    def __getitem__(self, index: int) -> Pose:
        """Get pose at index."""
        return self.poses[index]

    def __iter__(self) -> Iterator[Pose]:
        """Iterate over poses."""
        return iter(self.poses)

    def append(self, pose: Pose) -> None:
        """Add a pose to the array."""
        self.poses.append(pose)

    def encode(self) -> bytes:
        """
        Encode to bytes for LCM transmission.

        Note: This is a simple implementation. For production use,
        consider using proper LCM encoding.
        """
        import pickle

        return pickle.dumps({"header": self.header, "poses": self.poses})

    @classmethod
    def decode(cls, data: bytes) -> PoseArray:
        """
        Decode from bytes.

        Args:
            data: Pickled PoseArray data

        Returns:
            Decoded PoseArray
        """
        import pickle

        decoded = pickle.loads(data)
        return cls(header=decoded["header"], poses=decoded["poses"])
