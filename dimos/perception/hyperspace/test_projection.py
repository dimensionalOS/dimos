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

from dimos.models.embedding.base import PatchEmbeddings
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.hyperspace.projection import depth_to_meters, project_patches_to_world

W, H = 64, 48
FX = FY = 50.0
CX, CY = W / 2, H / 2


def _camera_info() -> CameraInfo:
    return CameraInfo(height=H, width=W, K=[FX, 0, CX, 0, FY, CY, 0, 0, 1])


def _patches(grid: int = 4, dim: int = 8) -> PatchEmbeddings:
    vectors = np.arange(grid * grid, dtype=np.float32).reshape(grid, grid, 1)
    return PatchEmbeddings(
        vector=np.repeat(vectors, dim, axis=2),
        frame_id="cam",
        ts=1.0,
        source_width=W,
        source_height=H,
    )


def test_depth_to_meters_uint16_is_mm() -> None:
    depth = Image(
        data=np.full((2, 2), 1500, dtype=np.uint16), format=ImageFormat.DEPTH16, frame_id="d"
    )
    np.testing.assert_allclose(depth_to_meters(depth), 1.5)


def test_flat_wall_projects_to_depth() -> None:
    depth_m = 2.0
    depth = Image(
        data=np.full((H, W), int(depth_m * 1000), dtype=np.uint16),
        format=ImageFormat.DEPTH16,
        frame_id="depth",
    )
    identity = np.eye(4)
    points, patch_idx = project_patches_to_world(
        _patches(), depth, _camera_info(), _camera_info(), identity, identity, stride=2
    )
    assert points.shape[0] > 0
    assert points.shape[0] == patch_idx.shape[0]
    np.testing.assert_allclose(points[:, 2], depth_m, atol=1e-5)

    # Center pixel unprojects to the optical axis.
    center = np.argmin(np.abs(points[:, 0]) + np.abs(points[:, 1]))
    np.testing.assert_allclose(points[center, :2], 0.0, atol=depth_m / FX * 2)

    # Points left of center get left-column patches, right of center right-column.
    grid = 4
    cols = patch_idx % grid
    assert cols[points[:, 0] < -0.5].max() < cols[points[:, 0] > 0.5].min()


def test_world_transform_applied() -> None:
    depth = Image(
        data=np.full((H, W), 1000, dtype=np.uint16), format=ImageFormat.DEPTH16, frame_id="depth"
    )
    world_from_depth = np.eye(4)
    world_from_depth[:3, 3] = [10.0, 20.0, 30.0]
    points, _ = project_patches_to_world(
        _patches(), depth, _camera_info(), _camera_info(), np.eye(4), world_from_depth, stride=8
    )
    assert (points[:, 0] > 9.0).all()
    np.testing.assert_allclose(points[:, 2], 31.0, atol=1e-5)


def test_depth_range_filter() -> None:
    data = np.zeros((H, W), dtype=np.uint16)
    data[:, : W // 2] = 200  # 0.2 m: below min_depth
    data[:, W // 2 :] = 9000  # 9 m: beyond max_depth
    depth = Image(data=data, format=ImageFormat.DEPTH16, frame_id="depth")
    points, patch_idx = project_patches_to_world(
        _patches(), depth, _camera_info(), _camera_info(), np.eye(4), np.eye(4)
    )
    assert points.shape == (0, 3)
    assert patch_idx.shape == (0,)


def test_offset_color_camera_shifts_patch_assignment() -> None:
    # color_from_depth with +x translation moves points right in the color
    # frame, so patch columns shift right vs the aligned case.
    depth = Image(
        data=np.full((H, W), 2000, dtype=np.uint16), format=ImageFormat.DEPTH16, frame_id="depth"
    )
    color_from_depth = np.eye(4)
    color_from_depth[0, 3] = 0.5
    _, aligned_idx = project_patches_to_world(
        _patches(), depth, _camera_info(), _camera_info(), np.eye(4), np.eye(4), stride=2
    )
    points, shifted_idx = project_patches_to_world(
        _patches(), depth, _camera_info(), _camera_info(), color_from_depth, np.eye(4), stride=2
    )
    assert points.shape[0] > 0
    assert (shifted_idx % 4).mean() > (aligned_idx % 4).mean()
