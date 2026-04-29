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

"""ContourPolygons3D: filled 2D contour polygons in 3D space.

On the wire this uses ``sensor_msgs/PointCloud2``.  Each point's
``intensity`` field encodes its polygon id.  The Python side groups
points by id, ear-clips each polygon into triangles, and renders via
``rr.Mesh3D``.
"""

from __future__ import annotations

from collections import defaultdict
import struct
from typing import TYPE_CHECKING, BinaryIO

from dimos_lcm.sensor_msgs import PointCloud2 as LCMPointCloud2

from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype


class ContourPolygons3D(Timestamped):
    """Filled contour polygons for debug visualization.

    Wire format: ``sensor_msgs/PointCloud2`` where each point's
    intensity encodes its polygon id.
    """

    msg_name = "nav_msgs.ContourPolygons3D"
    ts: float
    frame_id: str
    _raw_bytes: bytes | None  # store raw LCM bytes to preserve intensity

    def __init__(
        self,
        ts: float = 0.0,
        frame_id: str = "map",
        raw_bytes: bytes | None = None,
    ) -> None:
        self.frame_id = frame_id
        self.ts = ts
        self._raw_bytes = raw_bytes

    def lcm_encode(self) -> bytes:
        if self._raw_bytes is None:
            raise ValueError("No data to encode")
        return self._raw_bytes

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> ContourPolygons3D:
        raw = data if isinstance(data, bytes) else data.read()
        lcm_msg = LCMPointCloud2.lcm_decode(raw)
        header_ts = lcm_msg.header.stamp.sec + lcm_msg.header.stamp.nsec / 1e9
        frame_id = lcm_msg.header.frame_id
        return cls(ts=header_ts, frame_id=frame_id, raw_bytes=raw)

    def _parse_xyzi(self) -> list[tuple[float, float, float, float]]:
        """Extract (x, y, z, intensity) from raw PointCloud2 bytes."""
        if self._raw_bytes is None:
            return []

        lcm_msg = LCMPointCloud2.lcm_decode(self._raw_bytes)

        offsets: dict[str, int] = {}
        for f in lcm_msg.fields:
            offsets[f.name] = f.offset
        if "x" not in offsets or "y" not in offsets or "z" not in offsets:
            return []

        data = bytes(lcm_msg.data)
        step = lcm_msg.point_step
        n = lcm_msg.width * lcm_msg.height
        result: list[tuple[float, float, float, float]] = []
        for i in range(n):
            base = i * step
            if base + step > len(data):
                break
            x = struct.unpack_from("<f", data, base + offsets["x"])[0]
            y = struct.unpack_from("<f", data, base + offsets["y"])[0]
            z = struct.unpack_from("<f", data, base + offsets["z"])[0]
            intensity = 0.0
            if "intensity" in offsets:
                intensity = struct.unpack_from("<f", data, base + offsets["intensity"])[0]
            result.append((x, y, z, intensity))
        return result

    def to_rerun(
        self,
        z_offset: float = 1.5,
    ) -> Archetype:
        """Render polygon outlines as ``rr.LineStrips3D`` — pink closed loops."""
        import rerun as rr

        pts = self._parse_xyzi()
        if not pts:
            return rr.LineStrips3D([])

        # Group points by polygon_id (intensity)
        polys: dict[int, list[tuple[float, float, float]]] = defaultdict(list)
        for x, y, z, intensity in pts:
            polys[int(intensity)].append((x, y, z))

        strips: list[list[list[float]]] = []
        for _poly_id, verts in polys.items():
            if len(verts) < 3:
                continue
            # Close the polygon by appending first vertex at the end
            ring = [[v[0], v[1], v[2] + z_offset] for v in verts]
            ring.append(ring[0])
            strips.append(ring)

        if not strips:
            return rr.LineStrips3D([])

        return rr.LineStrips3D(
            strips,
            colors=[(255, 50, 200, 255)] * len(strips),  # bright pink
            radii=[0.15] * len(strips),  # thick lines
        )

    def __str__(self) -> str:
        n = len(self._parse_xyzi())
        return f"ContourPolygons3D(frame_id='{self.frame_id}', points={n})"
