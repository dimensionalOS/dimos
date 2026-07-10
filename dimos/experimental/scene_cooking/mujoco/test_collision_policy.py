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
import pytest

from dimos.experimental.scene_cooking.mujoco.collision_policy import CollisionSpec, decide_for_prim


def _flat_square_floor() -> tuple[np.ndarray, np.ndarray]:
    vertices = np.asarray(
        [
            [-1.0, -1.0, 0.0],
            [1.0, -1.0, 0.0],
            [1.0, 1.0, 0.0],
            [-1.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )
    triangles = np.asarray([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
    return vertices, triangles


def test_box_override_min_thickness_preserves_floor_top() -> None:
    vertices, triangles = _flat_square_floor()
    spec = CollisionSpec(
        prim_overrides={
            "Floor*": {
                "type": "box",
                "min_thickness": 0.04,
                "preserve": "top",
            }
        }
    )

    decision = decide_for_prim(vertices, triangles, "Floor_Plane.002", spec)

    assert decision.mode == "primitive"
    assert decision.reason == "sidecar:box"
    assert decision.primitive is not None
    assert decision.primitive["size"] == pytest.approx((1.0, 1.0, 0.02))
    assert decision.primitive["pos"] == pytest.approx((0.0, 0.0, -0.02))


def test_box_override_without_min_thickness_keeps_default_box_fit() -> None:
    vertices, triangles = _flat_square_floor()
    spec = CollisionSpec(prim_overrides={"Floor*": {"type": "box"}})

    decision = decide_for_prim(vertices, triangles, "Floor_Plane.002", spec)

    assert decision.mode == "primitive"
    assert decision.primitive is not None
    assert min(decision.primitive["size"]) == pytest.approx(0.001)
    assert decision.primitive["pos"][2] == pytest.approx(0.0)
