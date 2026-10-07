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

import numpy as np

from dimos.mapping.experimental.mesh import ChunkQueue, _code


def _region(lo: int, hi: int) -> np.ndarray:
    """Voxels lo..hi-1 along x, mid-chunk on y and z so they count toward one chunk row."""
    i = np.arange(lo, hi)
    return ((np.stack([i, i * 0 + 16, i * 0 + 16], 1) + 0.5) * 0.05).astype(np.float32)


def _queue(min_change: int = 20) -> ChunkQueue:
    return ChunkQueue(voxel_size=0.05, min_change=min_change, min_fraction=0.1)


def test_jiggle_waits_until_a_chunk_changed_enough() -> None:
    q = _queue()
    for hi in range(131, 150):  # one voxel at a time inside chunk 4: 19 changes
        assert not q.update(7, _region(130, hi), 0.0)
    assert q.update(7, _region(130, 150), 0.0)


def test_flipping_voxels_cancel_out() -> None:
    q = _queue()
    q.update(7, _region(130, 150), 0.0)
    q.take()
    for _ in range(20):  # one voxel off and back on, 40 changes, no net change
        assert not q.update(7, _region(130, 149), 0.0)
        assert not q.update(7, _region(130, 150), 0.0)


def test_most_recently_changed_chunk_goes_first() -> None:
    q = _queue(min_change=1)
    q.update(1, _region(130, 140), 1.0)  # chunk 4
    q.update(2, _region(200, 210), 2.0)  # chunk 6, later
    assert q.take()[0][0] == _code(np.array([[6, 0, 0]]))[0]


def test_colours_recolour_every_chunk_once_the_range_moves() -> None:
    from dimos.mapping.experimental.mesh import MeshColours
    from dimos.msgs.shape_msgs.TriangleMesh import TriangleMesh

    def chunk(key: tuple[int, int, int], z: float) -> TriangleMesh:
        v = np.array([[0, 0, z], [1, 0, z], [0, 1, z + 1]], np.float32)
        return TriangleMesh(v, np.array([[0, 1, 2]], np.uint32), key=key)

    colours = MeshColours()
    assert len(colours(chunk((0, 0, 0), 0.0))) == 2  # geometry + its colours
    assert len(colours(chunk((1, 0, 0), 0.1))) == 2  # range barely moved
    assert len(colours(chunk((2, 0, 0), 5.0))) == 4  # range moved: all three recoloured
