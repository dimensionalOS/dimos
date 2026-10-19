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

"""Array construction and rendering for generated keyed mesh chunks."""

from typing import Any

from dimos_generated.dimos_msgs.msg import TriangleMesh
from dimos_generated.std_msgs.msg import Header
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.time import header_now


def triangle_mesh_from_arrays(
    vertices: NDArray[Any],
    faces: NDArray[Any],
    normals: NDArray[Any] | None = None,
    key: tuple[int, int, int] = (0, 0, 0),
    *,
    header: Header | None = None,
) -> TriangleMesh:
    vertices = np.array(vertices, dtype=np.float32, copy=True).reshape(-1, 3)
    normals = (
        np.zeros_like(vertices)
        if normals is None
        else np.array(normals, dtype=np.float32, copy=True).reshape(-1, 3)
    )
    faces = np.array(faces, dtype=np.uint32, copy=True).reshape(-1, 3)
    if normals.shape != vertices.shape:
        raise ValueError("mesh normals must match vertices")
    return TriangleMesh(
        header=header_now() if header is None else header,
        key=np.array(key, dtype=np.int32),
        vertices=vertices.ravel(),
        normals=normals.ravel(),
        faces=faces.ravel(),
    )


def triangle_mesh_to_rerun(message: TriangleMesh) -> Any:
    import rerun as rr

    if len(message.faces) == 0:
        return rr.Clear(recursive=False)
    return rr.Mesh3D(
        vertex_positions=message.vertices.reshape(-1, 3),
        vertex_normals=message.normals.reshape(-1, 3),
        triangle_indices=message.faces.reshape(-1, 3),
    )
