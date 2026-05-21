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

"""LineSegments3D: collection of 3D line segments for graph edge visualization.

On the wire uses ``nav_msgs/Path`` — consecutive pose pairs form segments.
Renders as ``rr.LineStrips3D`` with each segment as a separate strip.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, BinaryIO

from dimos_lcm.nav_msgs import Path as LCMPath

from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype


Segment = tuple[tuple[float, float, float], tuple[float, float, float]]


class LineSegments3D(Timestamped):
    """Line segments for graph edge visualization.

    Wire format: ``nav_msgs/Path`` — consecutive pose pairs are segments.
    ``orientation.w`` encodes traversability: 1.0=traversable, 0.5=partial, 0.0=unreachable.
    Each endpoint's ``header.stamp`` is preserved into ``segment_timestamps`` so
    consumers can correlate endpoints back to source events (e.g. keyframe
    creation time for a pose-graph SLAM producer).
    """

    msg_name = "nav_msgs.LineSegments3D"
    ts: float
    frame_id: str
    segments: list[Segment]
    traversability: list[float]
    segment_timestamps: list[tuple[float, float]]

    def __init__(
        self,
        ts: float = 0.0,
        frame_id: str = "map",
        segments: list[Segment] | None = None,
        traversability: list[float] | None = None,
        segment_timestamps: list[tuple[float, float]] | None = None,
    ) -> None:
        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()
        self.segments = segments or []
        self.traversability = traversability or [1.0] * len(self.segments)
        self.segment_timestamps = segment_timestamps or [(self.ts, self.ts)] * len(self.segments)

    def lcm_encode(self) -> bytes:
        raise NotImplementedError("Encoded on C++ side")

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> LineSegments3D:
        lcm_msg = LCMPath.lcm_decode(data)
        header_ts = lcm_msg.header.stamp.sec + lcm_msg.header.stamp.nsec / 1e9
        frame_id = lcm_msg.header.frame_id

        segments: list[Segment] = []
        traversability: list[float] = []
        segment_timestamps: list[tuple[float, float]] = []
        poses = lcm_msg.poses
        for i in range(0, len(poses) - 1, 2):
            p1, p2 = poses[i], poses[i + 1]
            segments.append(
                (
                    (p1.pose.position.x, p1.pose.position.y, p1.pose.position.z),
                    (p2.pose.position.x, p2.pose.position.y, p2.pose.position.z),
                )
            )
            traversability.append(p1.pose.orientation.w)
            start_ts = p1.header.stamp.sec + p1.header.stamp.nsec / 1e9
            end_ts = p2.header.stamp.sec + p2.header.stamp.nsec / 1e9
            segment_timestamps.append((start_ts, end_ts))
        return cls(
            ts=header_ts,
            frame_id=frame_id,
            segments=segments,
            traversability=traversability,
            segment_timestamps=segment_timestamps,
        )

    def to_rerun(
        self,
        z_offset: float = 1.7,
        color: tuple[int, int, int, int] = (0, 255, 150, 255),
        radii: float = 0.04,
    ) -> Archetype:
        """Render as ``rr.LineStrips3D`` — color-coded by traversability."""
        import rerun as rr

        if not self.segments:
            return rr.LineStrips3D([])

        strips = []
        colors = []
        for idx, (p1, p2) in enumerate(self.segments):
            strips.append(
                [
                    [p1[0], p1[1], p1[2] + z_offset],
                    [p2[0], p2[1], p2[2] + z_offset],
                ]
            )
            trav = self.traversability[idx] if idx < len(self.traversability) else 1.0
            if trav >= 0.9:
                colors.append((0, 220, 100, 200))
            elif trav >= 0.4:
                colors.append((255, 180, 0, 200))
            else:
                colors.append((255, 50, 50, 150))

        return rr.LineStrips3D(strips, colors=colors, radii=[radii] * len(strips))

    def __len__(self) -> int:
        return len(self.segments)

    def __str__(self) -> str:
        return f"LineSegments3D(frame_id='{self.frame_id}', segments={len(self.segments)})"
