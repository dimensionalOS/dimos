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

from typing import TYPE_CHECKING, BinaryIO

from dimos_lcm.geometry_msgs import PoseArray as LCMPoseArray
import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.std_msgs.Header import Header

if TYPE_CHECKING:
    from collections.abc import Iterator

    from rerun._baseclasses import Archetype


class PoseArray(LCMPoseArray):  # type: ignore[misc]
    """
    An array of poses with a header for reference frame and timestamp.

    This is commonly used for representing multiple candidate positions,
    such as grasp poses, particle filter samples, or waypoints.
    """

    msg_name = "geometry_msgs.PoseArray"
    header: Header
    poses: list[Pose]

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

    @property
    def frame_id(self) -> str:
        return str(self.header.frame_id)

    @property
    def ts(self) -> float:
        return float(self.header.stamp.sec + self.header.stamp.nsec / 1e9)

    # The LCM wire count, always derived; the base decoder assigns it.
    @property
    def poses_length(self) -> int:
        return len(self.poses)

    @poses_length.setter
    def poses_length(self, _: int) -> None:
        pass

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> PoseArray:
        msg = LCMPoseArray.lcm_decode(data)
        return cls(Header(msg.header), [Pose(pose) for pose in msg.poses])

    def positions(self) -> np.ndarray:
        """Pose positions, (N, 3)."""
        return np.array([[p.x, p.y, p.z] for p in self.poses], dtype=np.float64).reshape(-1, 3)

    def to_rerun(self, color: tuple[int, int, int] = (255, 0, 0), length: float = 0.1) -> Archetype:
        """Render as ``rr.Arrows3D``, one arrow along each pose's x axis."""
        import rerun as rr
        from scipy.spatial.transform import Rotation

        if not self.poses:
            return rr.Arrows3D(origins=[], vectors=[])
        quats = np.array([p.orientation.to_numpy() for p in self.poses])
        vectors = Rotation.from_quat(quats).apply([length, 0.0, 0.0])
        return rr.Arrows3D(origins=self.positions(), vectors=vectors, colors=[color])
