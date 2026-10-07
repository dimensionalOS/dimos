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

"""A triangle mesh with per-vertex normals, one chunk of a larger mesh keyed by ``key``."""

from __future__ import annotations

import struct
import time
from typing import TYPE_CHECKING

import numpy as np

from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from numpy.typing import NDArray
    from rerun._baseclasses import Archetype

_HEADER = struct.Struct("<d3iII")


class TriangleMesh(Timestamped):
    msg_name = "shape_msgs.TriangleMesh"

    def __init__(
        self,
        vertices: NDArray[np.float32],
        faces: NDArray[np.uint32],
        normals: NDArray[np.float32] | None = None,
        key: tuple[int, int, int] = (0, 0, 0),
        ts: float | None = None,
    ) -> None:
        self.vertices = np.ascontiguousarray(vertices, dtype=np.float32).reshape(-1, 3)
        self.faces = np.ascontiguousarray(faces, dtype=np.uint32).reshape(-1, 3)
        self.normals = (
            np.zeros_like(self.vertices)
            if normals is None
            else np.ascontiguousarray(normals, dtype=np.float32).reshape(-1, 3)
        )
        self.key = key
        self.ts = ts or time.time()

    def lcm_encode(self) -> bytes:
        header = _HEADER.pack(self.ts, *self.key, len(self.vertices), len(self.faces))
        return header + self.vertices.tobytes() + self.normals.tobytes() + self.faces.tobytes()

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs: object) -> TriangleMesh:
        ts, i, j, k, nv, nf = _HEADER.unpack_from(data)
        o = _HEADER.size
        v = np.frombuffer(data, np.float32, nv * 3, o)
        n = np.frombuffer(data, np.float32, nv * 3, o + nv * 12)
        f = np.frombuffer(data, np.uint32, nf * 3, o + nv * 24)
        return cls(v, f, n, (i, j, k), ts)

    def to_rerun(self) -> Archetype:
        import rerun as rr

        # an emptied chunk clears its entity; rerun rejects a mesh without triangles
        if len(self.faces) == 0:
            return rr.Clear(recursive=False)
        return rr.Mesh3D(
            vertex_positions=self.vertices,
            vertex_normals=self.normals,
            triangle_indices=self.faces,
        )
