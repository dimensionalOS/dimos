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

"""LidarColorModule v1 — frustum rendering smoke test."""

from __future__ import annotations

from unittest.mock import patch

import numpy as np
import pytest
import rerun as rr

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.lidar_color_module import LidarColorModule


def _info(width: int = 640, height: int = 480, fx: float = 500.0, fy: float = 500.0) -> CameraInfo:
    return CameraInfo.from_intrinsics(
        fx=fx, fy=fy, cx=width / 2, cy=height / 2, width=width, height=height
    )


@pytest.fixture
def mod():
    m = LidarColorModule(ray_length=1.0)
    yield m
    m._close_module()


def test_corner_endpoints_unset_until_camera_info(mod: LidarColorModule):
    assert mod._corner_endpoints is None


def test_corner_rays_have_expected_signs_after_camera_info(mod: LidarColorModule):
    mod._on_camera_info(_info(640, 480))
    endpoints = mod._corner_endpoints
    assert endpoints is not None
    # Image corners are (0,0) top-left, (W,0) top-right, (W,H) bottom-right, (0,H) bottom-left.
    # Optical frame: x right, y down, z forward.
    assert endpoints[0, 0] < 0 and endpoints[0, 1] < 0  # top-left:    -x, -y, +z
    assert endpoints[1, 0] > 0 and endpoints[1, 1] < 0  # top-right:   +x, -y, +z
    assert endpoints[2, 0] > 0 and endpoints[2, 1] > 0  # bot-right:   +x, +y, +z
    assert endpoints[3, 0] < 0 and endpoints[3, 1] > 0  # bot-left:    -x, +y, +z
    assert (endpoints[:, 2] > 0).all()
    np.testing.assert_allclose(np.linalg.norm(endpoints, axis=1), 1.0, atol=1e-9)


def test_render_before_camera_info_is_noop(mod: LidarColorModule):
    img = Image(data=np.zeros((480, 640, 3), dtype=np.uint8), format=ImageFormat.RGB)
    with patch.object(rr, "log") as log_mock:
        mod._render_frustum(img)
    assert log_mock.call_count == 0


def test_render_after_camera_info_logs_two_archetypes(mod: LidarColorModule):
    """Once camera_info has arrived, each image logs LineStrips3D + Transform3D."""
    mod._on_camera_info(_info(640, 480))
    img = Image(data=np.zeros((480, 640, 3), dtype=np.uint8), format=ImageFormat.RGB)

    with patch.object(rr, "log") as log_mock:
        mod._render_frustum(img)

    assert log_mock.call_count == 2
    paths = [call.args[0] for call in log_mock.call_args_list]
    assert paths == [mod.config.entity_path, mod.config.entity_path]
    assert isinstance(log_mock.call_args_list[0].args[1], rr.LineStrips3D)
    assert isinstance(log_mock.call_args_list[1].args[1], rr.Transform3D)


# --- color_pointcloud pure helper -------------------------------------------


from dimos.perception.lidar_color_module import color_pointcloud


def _solid_image(w: int, h: int, r: int, g: int, b: int) -> Image:
    """Image of solid RGB colour (uses RGB format so byte order is unambiguous)."""
    data = np.empty((h, w, 3), dtype=np.uint8)
    data[..., 0] = r
    data[..., 1] = g
    data[..., 2] = b
    return Image(data=data, format=ImageFormat.RGB)


def _gradient_image(w: int, h: int) -> Image:
    """RGB image where R encodes u and G encodes v — easy to verify sampled colors."""
    u = np.arange(w, dtype=np.uint8)
    v = np.arange(h, dtype=np.uint8)
    r = np.broadcast_to(u, (h, w))
    g = np.broadcast_to(v[:, None], (h, w))
    b = np.zeros((h, w), dtype=np.uint8)
    data = np.stack([r, g, b], axis=2)
    return Image(data=data, format=ImageFormat.RGB)


def test_color_pointcloud_picks_correct_pixels():
    """A point at (X, Y, Z) projects to (cx + fx*X/Z, cy + fy*Y/Z); we sample that pixel."""
    info = _info(width=200, height=100)  # fx=500, fy=500, cx=100, cy=50
    img = _gradient_image(200, 100)
    # Two points at known camera-frame positions; identity extrinsic.
    # X/Z and Y/Z chosen so the projection lands on integer pixels (no
    # half-pixel rounding ambiguity).
    pts = np.array([[0.0, 0.0, 2.0], [0.2, 0.04, 2.0]], dtype=np.float32)
    positions, colors = color_pointcloud(pts, img, info, T_camera_lidar=np.eye(4))
    assert positions.shape == (2, 3)
    assert colors.shape == (2, 3)
    # Pinhole: u = 500*X/Z + 100, v = 500*Y/Z + 50
    expected_uv = [(100, 50), (100 + 50, 50 + 10)]  # second pt: X/Z=0.1, Y/Z=0.02
    for (u, v), c in zip(expected_uv, colors, strict=False):
        # Gradient encodes (u, v, 0).
        assert int(c[0]) == u
        assert int(c[1]) == v
        assert int(c[2]) == 0


def test_color_pointcloud_drops_behind_camera():
    info = _info(640, 480)
    img = _solid_image(640, 480, 255, 0, 0)
    pts = np.array([[0.0, 0.0, -1.0], [0.0, 0.0, -5.0]], dtype=np.float32)
    positions, colors = color_pointcloud(pts, img, info, T_camera_lidar=np.eye(4))
    assert positions.shape == (0, 3)
    assert colors.shape == (0, 3)


def test_color_pointcloud_drops_out_of_frame():
    info = _info(640, 480)
    img = _solid_image(640, 480, 255, 0, 0)
    # In front of camera but way off-axis -> projects outside image.
    pts = np.array([[100.0, 0.0, 1.0]], dtype=np.float32)
    positions, _ = color_pointcloud(pts, img, info, T_camera_lidar=np.eye(4))
    assert positions.shape == (0, 3)


def test_color_pointcloud_applies_extrinsic():
    """A point at lidar origin should map to camera origin if T is identity translation."""
    info = _info(640, 480)
    img = _solid_image(640, 480, 50, 150, 200)
    # T translates lidar +1m along camera +Z — so a lidar-frame point at origin
    # appears at z=1 in camera, which is in front and on-axis.
    T = np.eye(4)
    T[2, 3] = 1.0
    pts = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    positions, colors = color_pointcloud(pts, img, info, T_camera_lidar=T)
    assert positions.shape == (1, 3)
    # Position is in *lidar* frame — unchanged from input.
    np.testing.assert_allclose(positions[0], [0.0, 0.0, 0.0])
    # Color from the solid image: (50, 150, 200).
    np.testing.assert_array_equal(colors[0], [50, 150, 200])


def test_color_pointcloud_handles_bgr_format():
    """BGR images get reversed to RGB before sampling."""
    info = _info(640, 480)
    # Solid blue in RGB == (0, 0, 255) — but stored as BGR is (255, 0, 0).
    data = np.empty((480, 640, 3), dtype=np.uint8)
    data[..., 0] = 255  # B channel
    data[..., 1] = 0
    data[..., 2] = 0
    bgr = Image(data=data, format=ImageFormat.BGR)
    pts = np.array([[0.0, 0.0, 2.0]], dtype=np.float32)
    _, colors = color_pointcloud(pts, bgr, info, T_camera_lidar=np.eye(4))
    # After BGR->RGB swap, sampled color should be RGB (0, 0, 255).
    np.testing.assert_array_equal(colors[0], [0, 0, 255])


def test_color_pointcloud_empty_input():
    info = _info(640, 480)
    img = _solid_image(640, 480, 0, 0, 0)
    pts = np.zeros((0, 3), dtype=np.float32)
    positions, colors = color_pointcloud(pts, img, info, T_camera_lidar=np.eye(4))
    assert positions.shape == (0, 3)
    assert colors.shape == (0, 3)


def test_camera_info_change_updates_frustum(mod: LidarColorModule):
    """A second CameraInfo with different fx should replace the cached endpoints."""
    mod._on_camera_info(_info(640, 480, fx=500.0))
    narrow = mod._corner_endpoints.copy()
    mod._on_camera_info(_info(640, 480, fx=250.0))
    wide = mod._corner_endpoints
    # Wider FoV (smaller fx) -> larger |x| at the corners.
    assert np.abs(wide[:, 0]).mean() > np.abs(narrow[:, 0]).mean()
