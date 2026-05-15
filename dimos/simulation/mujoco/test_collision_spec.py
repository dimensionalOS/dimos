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

import numpy as np

from dimos.simulation.mujoco.collision_spec import CollisionSpec, decide_for_prim


def test_rectangular_flat_sheet_stays_single_horizontal_box() -> None:
    vertices = np.asarray(
        [
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 2.0, 0.0],
            [0.0, 2.0, 0.0],
        ],
        dtype=np.float64,
    )
    triangles = np.asarray([[0, 1, 2], [0, 2, 3]], dtype=np.int32)

    decision = decide_for_prim(vertices, triangles, "flat_rect", CollisionSpec())

    assert decision.mode == "primitive"
    assert decision.reason == "aspect-ratio:horizontal-slab"
    assert decision.primitive is not None
    assert decision.primitive["quat"] == (1.0, 0.0, 0.0, 0.0)


def test_large_non_rectangular_flat_sheet_uses_triangle_prisms() -> None:
    vertices = np.asarray(
        [
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.0, 2.0, 0.0],
            [0.0, 2.0, 0.0],
        ],
        dtype=np.float64,
    )
    triangles = np.asarray(
        [
            [0, 1, 2],
            [0, 2, 3],
            [3, 4, 5],
            [3, 5, 6],
        ],
        dtype=np.int32,
    )

    decision = decide_for_prim(vertices, triangles, "flat_l_shape", CollisionSpec())

    assert decision.mode == "hulls"
    assert decision.reason == "thin-sheet:triangle-prisms(4)"
    assert len(decision.hulls) == 4


def test_sheet_prisms_can_be_disabled() -> None:
    vertices = np.asarray(
        [
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.0, 2.0, 0.0],
            [0.0, 2.0, 0.0],
        ],
        dtype=np.float64,
    )
    triangles = np.asarray(
        [
            [0, 1, 2],
            [0, 2, 3],
            [3, 4, 5],
            [3, 5, 6],
        ],
        dtype=np.int32,
    )

    decision = decide_for_prim(
        vertices,
        triangles,
        "flat_l_shape",
        CollisionSpec(enable_sheet_prisms=False),
    )

    assert decision.mode == "primitive"
    assert decision.reason == "aspect-ratio:horizontal-slab"


def test_small_non_rectangular_flat_sheet_stays_single_horizontal_box() -> None:
    vertices = np.asarray(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.5, 0.0],
            [0.0, 0.5, 0.0],
            [0.5, 0.5, 0.0],
            [0.5, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )
    triangles = np.asarray(
        [
            [0, 1, 2],
            [0, 2, 3],
            [3, 4, 5],
            [3, 5, 6],
        ],
        dtype=np.int32,
    )

    decision = decide_for_prim(vertices, triangles, "small_flat_l_shape", CollisionSpec())

    assert decision.mode == "primitive"
    assert decision.reason == "aspect-ratio:horizontal-slab"
