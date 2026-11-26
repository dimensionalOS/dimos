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

"""Check frame conventions and geometry boundaries before using policy observations."""

from typing import Any

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.experimental.gta_sim2.prepare import clip_triangle, ground_height
from dimos.experimental.gta_sim2.renderer import (
    RAGE_TO_RENDER,
    object_visual,
    projection,
    renderer_transform,
)
from dimos.sim2.sensors.camera.config import CameraConfig


def test_asymmetric_intrinsics_and_reversed_depth() -> None:
    camera = CameraConfig("test", width=640, height=480, fx=410, fy=390, cx=287, cy=213)
    xyz = np.array([0.4, 0.2, 2.0])  # optical +X right, +Y down, +Z forward
    clip = projection(camera) @ np.r_[xyz * [1, -1, -1], 1]
    ndc = clip[:3] / clip[3]
    pixel = (ndc[:2] * [1, -1] + 1) * [camera.width / 2, camera.height / 2]
    np.testing.assert_allclose(pixel, [369, 252])
    assert clip[3] == 2.0  # optical distance, not Euclidean range
    near = projection(camera) @ [0, 0, -0.01, 1]
    far = projection(camera) @ [0, 0, -5000, 1]
    np.testing.assert_allclose(near[2] / near[3], 1)
    np.testing.assert_allclose(far[2] / far[3], 0, atol=1e-12)


def test_actor_and_optical_camera_share_origin_rotation_and_roll() -> None:
    rotation = Rotation.from_euler("xyz", [0.3, -0.2, 0.7]).as_matrix()
    position = np.array([3.0, 5.0, 1.0])
    origin = np.array([-458.0, -66.0, 1.3])
    local_point = np.array([0.1, 0.2, 0.3])
    transformed = renderer_transform(position, rotation, origin) @ np.r_[local_point, 1]
    np.testing.assert_allclose(
        transformed[:3], RAGE_TO_RENDER @ (rotation @ local_point + position + origin)
    )
    forward, up = RAGE_TO_RENDER @ rotation[:, 2], -RAGE_TO_RENDER @ rotation[:, 1]
    np.testing.assert_allclose(forward @ up, 0, atol=1e-12)
    np.testing.assert_allclose(np.cross(forward, up), RAGE_TO_RENDER @ rotation[:, 0])


def test_patch_clipping_and_overpass_layer_selection() -> None:
    triangle = np.array([[-2.0, -2.0, 1.0], [2.0, -2.0, 1.0], [0.0, 2.0, 1.0]])
    clipped = np.asarray(
        clip_triangle(triangle, np.array([-1.0, -1.0, 0.0]), np.array([1.0, 1.0, 2.0]))
    )
    assert np.all(clipped[:, :, :2] >= -1) and np.all(clipped[:, :, :2] <= 1)
    surfaces = np.stack([triangle, triangle + np.array([0, 0, 10])])
    assert ground_height(surfaces, (0, 0), 2) == 1
    assert ground_height(surfaces, (0, 0), 12) == 11


def test_moving_prop_rotates_about_physics_pivot_in_gltf_coordinates() -> None:
    rotation = Rotation.from_euler("xyz", [0.2, -0.4, 1.1])
    pivot = np.array([0.5, 0.1, 0.0])
    scale = 1.3
    position = np.array([12.0, 4.0, 1.0])
    origin = np.array([-458.0, -66.0, 1.3])
    state: dict[str, Any] = {
        "identity": "map/entity/7",
        "position": position,
        "quaternion_wxyz": rotation.as_quat(scalar_first=True),
        "visual": {
            "source_map": "map",
            "source_model": "asset",
            "source_position": [0, 0, 0],
            "pivot": pivot,
            "scale": scale,
        },
    }
    matrix = np.array(object_visual(state, origin)["transform"]).reshape(4, 4).T
    np.testing.assert_allclose(
        (matrix @ np.r_[RAGE_TO_RENDER @ pivot, 1])[:3], RAGE_TO_RENDER @ (position + origin)
    )
    point = np.array([1.0, 0.3, 0.8])
    expected = RAGE_TO_RENDER @ (rotation.apply((point - pivot) * scale) + position + origin)
    np.testing.assert_allclose((matrix @ np.r_[RAGE_TO_RENDER @ point, 1])[:3], expected)
    state["quaternion_wxyz"] = [0, 0, 0, 0]
    with pytest.raises(ValueError, match="Invalid dynamic"):
        object_visual(state, origin)
