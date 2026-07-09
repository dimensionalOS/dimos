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

import os

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def _turbojpeg_available() -> bool:
    try:
        from turbojpeg import TurboJPEG

        TurboJPEG()
    except Exception:
        return False
    return True


# Skip locally when the native lib is missing; in CI run anyway so a missing
# dep fails loudly instead of silently skipping (aiortc guard pattern). An
# import-time assert would break jobs that collect-but-deselect these tests.
pytestmark = pytest.mark.skipif(
    not _turbojpeg_available() and not os.environ.get("CI"),
    reason="native libturbojpeg unavailable",
)


@pytest.fixture()
def rgb_image():
    rng = np.random.RandomState(42)
    gradient = np.linspace(0, 255, 640, dtype=np.uint8)
    data = np.broadcast_to(gradient, (480, 640)).copy()
    data = np.stack([data, data // 2, rng.randint(0, 50, (480, 640), dtype=np.uint8)], axis=-1)
    return Image(data=data, format=ImageFormat.RGB, frame_id="cam", ts=1234.5678)


def test_jpeg_roundtrip_preserves_meta_and_pixels(rgb_image) -> None:
    ci = CompressedImage.from_image(rgb_image, quality=90)
    assert ci.format == "jpeg"
    assert ci.frame_id == "cam"
    assert ci.ts == rgb_image.ts
    assert 0 < len(ci.data) < rgb_image.data.nbytes // 4

    img = ci.decode()
    assert img.format == ImageFormat.RGB
    assert img.frame_id == "cam"
    assert img.ts == rgb_image.ts
    assert img.shape == rgb_image.shape
    diff = np.abs(img.data.astype(int) - rgb_image.data.astype(int)).mean()
    assert diff < 5, f"JPEG q90 mean pixel error too high: {diff}"


def test_png_roundtrip_is_lossless_bgr(rgb_image) -> None:
    bgr = rgb_image.to_bgr()
    ci = CompressedImage.from_image(bgr, format="png")
    img = ci.decode()
    assert img.format == ImageFormat.BGR
    assert np.array_equal(img.data, bgr.data)
    assert img.ts == bgr.ts


def test_png_roundtrip_is_lossless_gray16() -> None:
    data = np.arange(100 * 80, dtype=np.uint16).reshape(100, 80)
    src = Image(data=data, format=ImageFormat.GRAY16, frame_id="d", ts=1.0)
    img = CompressedImage.from_image(src, format="png").decode()
    assert img.format == ImageFormat.GRAY16
    assert np.array_equal(img.data, data)


def test_lcm_roundtrip(rgb_image) -> None:
    ci = CompressedImage.from_image(rgb_image)
    wire = ci.lcm_encode()
    out = CompressedImage.lcm_decode(wire)
    assert out.data == ci.data
    assert out.format == "jpeg"
    assert out.frame_id == "cam"
    assert abs(out.ts - ci.ts) < 1e-6


def test_jpeg_rejects_depth_formats() -> None:
    depth = Image(data=np.zeros((10, 10), dtype=np.uint16), format=ImageFormat.GRAY16)
    with pytest.raises(ValueError, match="use format='png'"):
        CompressedImage.from_image(depth)


def test_png_rejects_float_depth() -> None:
    depth = Image(data=np.zeros((10, 10), dtype=np.float32), format=ImageFormat.DEPTH)
    with pytest.raises(ValueError, match="PNG cannot encode"):
        CompressedImage.from_image(depth, format="png")


def test_from_image_rejects_non_image() -> None:
    with pytest.raises(TypeError):
        CompressedImage.from_image(b"not an image")  # type: ignore[arg-type]


def test_max_width_resizes(rgb_image) -> None:
    img = CompressedImage.from_image(rgb_image, max_width=320).decode()
    assert img.width <= 320
    assert img.height <= 320


def test_to_rerun_is_encoded_image(rgb_image) -> None:
    import rerun as rr

    assert isinstance(CompressedImage.from_image(rgb_image).to_rerun(), rr.EncodedImage)
