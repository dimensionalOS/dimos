# Copyright 2026 Dimensional Inc.
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

"""GraphNodes3D: visibility-graph nodes for debug visualization.

On the wire this reuses ``nav_msgs/Path``.  Each pose is a node; the
``orientation.w`` field encodes the node type:

    0 = normal nav node
    1 = odom (robot) node
    2 = goal node
    3 = frontier node
    4 = navpoint (trajectory) node

Rerun visualization renders as ``rr.Points3D`` with type-based coloring.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, BinaryIO

from dimos_lcm.geometry_msgs import (
    Point as LCMPoint,
    Pose as LCMPose,
    PoseStamped as LCMPoseStamped,
    Quaternion as LCMQuaternion,
)
from dimos_lcm.nav_msgs import Path as LCMPath
from dimos_lcm.std_msgs import Header as LCMHeader, Time as LCMTime

from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype


# Node type → RGBA color
TYPE_COLORS: dict[int, tuple[int, int, int, int]] = {
    0: (180, 180, 180, 200),  # normal — grey
    1: (0, 255, 0, 255),  # odom — green
    2: (255, 0, 0, 255),  # goal — red
    3: (255, 165, 0, 200),  # frontier — orange
    4: (0, 200, 255, 200),  # navpoint — cyan
}
DEFAULT_COLOR = (200, 200, 200, 180)


class GraphNode:
    """A single graph node with position and type."""

    __slots__ = ("node_type", "x", "y", "z")

    def __init__(self, x: float, y: float, z: float, node_type: int = 0) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.node_type = node_type


def _sec_nsec(ts: float) -> list[int]:
    s = int(ts)
    return [s, int((ts - s) * 1_000_000_000)]


class GraphNodes3D(Timestamped):
    """Visibility-graph node positions for debug visualization."""

    msg_name = "nav_msgs.GraphNodes3D"
    ts: float
    frame_id: str
    nodes: list[GraphNode]

    def __init__(
        self,
        ts: float = 0.0,
        frame_id: str = "map",
        nodes: list[GraphNode] | None = None,
    ) -> None:
        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()
        self.nodes = nodes if nodes is not None else []

    def lcm_encode(self) -> bytes:
        lcm_msg = LCMPath()
        lcm_msg.poses_length = len(self.nodes)
        lcm_msg.poses = []

        for node in self.nodes:
            pose = LCMPoseStamped()
            pose.header = LCMHeader()
            pose.header.stamp = LCMTime()
            [pose.header.stamp.sec, pose.header.stamp.nsec] = _sec_nsec(self.ts)
            pose.header.frame_id = self.frame_id
            pose.pose = LCMPose()
            pose.pose.position = LCMPoint()
            pose.pose.position.x = node.x
            pose.pose.position.y = node.y
            pose.pose.position.z = node.z
            pose.pose.orientation = LCMQuaternion()
            pose.pose.orientation.w = float(node.node_type)
            lcm_msg.poses.append(pose)

        lcm_msg.header = LCMHeader()
        lcm_msg.header.stamp = LCMTime()
        [lcm_msg.header.stamp.sec, lcm_msg.header.stamp.nsec] = _sec_nsec(self.ts)
        lcm_msg.header.frame_id = self.frame_id
        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> GraphNodes3D:
        lcm_msg = LCMPath.lcm_decode(data)
        header_ts = lcm_msg.header.stamp.sec + lcm_msg.header.stamp.nsec / 1e9
        frame_id = lcm_msg.header.frame_id

        nodes: list[GraphNode] = []
        for pose in lcm_msg.poses:
            nodes.append(
                GraphNode(
                    x=pose.pose.position.x,
                    y=pose.pose.position.y,
                    z=pose.pose.position.z,
                    node_type=int(pose.pose.orientation.w),
                )
            )
        return cls(ts=header_ts, frame_id=frame_id, nodes=nodes)

    def to_rerun(
        self,
        z_offset: float = 1.7,
        radii: float = 0.12,
    ) -> Archetype:
        """Render as ``rr.Points3D`` with type-based coloring."""
        import rerun as rr

        if not self.nodes:
            return rr.Points3D([])

        positions = [[n.x, n.y, n.z + z_offset] for n in self.nodes]
        colors = [TYPE_COLORS.get(n.node_type, DEFAULT_COLOR) for n in self.nodes]
        node_radii = [radii * 2.0 if n.node_type in (1, 2) else radii for n in self.nodes]

        return rr.Points3D(positions, colors=colors, radii=node_radii)

    def __len__(self) -> int:
        return len(self.nodes)

    def __str__(self) -> str:
        return f"GraphNodes3D(frame_id='{self.frame_id}', nodes={len(self.nodes)})"
