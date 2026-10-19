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

"""Keyed mesh chunks retain geometry and source metadata over CDR."""

from dimos_cdrgen.registry import decode, encode
import numpy as np

from dimos.msgs.mesh import triangle_mesh_from_arrays
from dimos.msgs.time import header_now, to_nanoseconds


def test_mesh_cdr_preserves_arrays_key_and_source_stamp() -> None:
    header = header_now("map")
    vertices = np.array([[0, 1, 2], [3, 4, 5], [6, 7, 8]], dtype=np.float32)
    message = triangle_mesh_from_arrays(
        vertices, np.array([[0, 1, 2]], dtype=np.uint32), key=(-1, 2, 3), header=header
    )
    vertices[:] = 99
    restored = decode(encode(message), message.__msgtype__)
    assert restored.vertices.tolist() == [0, 1, 2, 3, 4, 5, 6, 7, 8]
    assert restored.normals.tolist() == [0] * 9
    assert restored.faces.tolist() == [0, 1, 2]
    assert restored.key.tolist() == [-1, 2, 3]
    assert restored.header.frame_id == "map"
    assert to_nanoseconds(restored.header.stamp) == to_nanoseconds(header.stamp)
