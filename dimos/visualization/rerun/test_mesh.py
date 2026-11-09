# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
"""Shared mesh rendering works without importing the mapping implementation."""

import numpy as np

from dimos.message_utils import mesh as mesh_utils, time as message_time
from dimos.messages.dimos_msgs.msg import TriangleMesh


def test_colours_recolour_every_chunk_once_the_range_moves() -> None:
    from dimos.visualization.rerun.mesh import MeshColours

    def chunk(key: tuple[int, int, int], z: float) -> TriangleMesh:
        v = np.array([[0, 0, z], [1, 0, z], [0, 1, z + 1]], np.float32)
        return mesh_utils.from_arrays(
            v, np.array([[0, 1, 2]], np.uint32), key=key, header=message_time.header_now()
        )

    colours = MeshColours()
    assert len(colours(chunk((0, 0, 0), 0.0))) == 2  # geometry + its colours
    assert len(colours(chunk((1, 0, 0), 0.1))) == 2  # range barely moved
    assert len(colours(chunk((2, 0, 0), 5.0))) == 4  # range moved: all three recoloured
