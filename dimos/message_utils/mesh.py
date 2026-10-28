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

"""Generated chunked triangle meshes with array-shaped numerical views."""

from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.messages.dimos_msgs.msg import TriangleMesh
from dimos.messages.std_msgs.msg import Header


def from_arrays(
    vertices: NDArray[Any],
    faces: NDArray[Any],
    normals: NDArray[Any] | None = None,
    *,
    key: tuple[int, int, int] = (0, 0, 0),
    header: Header,
) -> TriangleMesh:
    vertices = np.asarray(vertices, dtype=np.float32).reshape(-1, 3)
    return TriangleMesh(
        header=header,
        key=np.asarray(key, dtype=np.int32),
        vertices=np.ascontiguousarray(vertices).reshape(-1),
        normals=np.ascontiguousarray(
            np.zeros_like(vertices) if normals is None else normals, dtype=np.float32
        )
        .reshape(-1, 3)
        .reshape(-1),
        faces=np.ascontiguousarray(faces, dtype=np.uint32).reshape(-1, 3).reshape(-1),
    )


def to_rerun(message: TriangleMesh) -> Any:
    import rerun as rr

    if len(message.faces) == 0:
        return rr.Clear(recursive=False)
    return rr.Mesh3D(
        vertex_positions=message.vertices.reshape(-1, 3),
        vertex_normals=message.normals.reshape(-1, 3),
        triangle_indices=message.faces.reshape(-1, 3),
    )
