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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import CompressedImage
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.msgs.image import (
    compressed_image_from_image,
    image_from_array,
    image_from_compressed,
    image_to_bgr,
    image_to_rgb,
    image_view,
)
from dimos.msgs.time import time_from_seconds, to_nanoseconds
from dimos.visualization.rerun.message_helpers import image_archetype


@pytest.fixture()
def rgb_image():
    rng = np.random.RandomState(42)
    gradient = np.linspace(0, 255, 640, dtype=np.uint8)
    data = np.broadcast_to(gradient, (480, 640)).copy()
    data = np.stack([data, data // 2, rng.randint(0, 50, (480, 640), dtype=np.uint8)], axis=-1)
    return image_from_array(
        data, encoding="rgb8", header=Header(frame_id="cam", stamp=time_from_seconds(1234.5678))
    )


def test_jpeg_roundtrip_preserves_meta_and_pixels(rgb_image) -> None:
    ci = compressed_image_from_image(rgb_image, quality=90)
    assert "; jpeg compressed " in ci.format
    assert ci.header.frame_id == "cam"
    assert to_nanoseconds(ci.header.stamp) == to_nanoseconds(rgb_image.header.stamp)
    assert 0 < len(ci.data) < image_view(rgb_image).nbytes // 4
    img = image_from_compressed(ci)
    assert img.encoding in ("rgb8", "bgr8")
    assert img.header.frame_id == "cam"
    assert to_nanoseconds(img.header.stamp) == to_nanoseconds(rgb_image.header.stamp)
    assert image_view(img).shape == image_view(rgb_image).shape
    diff = np.abs(image_to_rgb(img).astype(int) - image_view(rgb_image).astype(int)).mean()
    assert diff < 5, f"JPEG q90 mean pixel error too high: {diff}"


def test_png_roundtrip_is_lossless_bgr(rgb_image) -> None:
    bgr = image_from_array(image_to_bgr(rgb_image), encoding="bgr8", header=rgb_image.header)
    ci = compressed_image_from_image(bgr, format="png")
    img = image_from_compressed(ci)
    assert img.encoding == "bgr8"
    assert np.array_equal(image_view(img), image_view(bgr))
    assert to_nanoseconds(img.header.stamp) == to_nanoseconds(bgr.header.stamp)


def test_png_roundtrip_is_lossless_gray16() -> None:
    data = np.arange(100 * 80, dtype=np.uint16).reshape(100, 80)
    src = image_from_array(
        data, encoding="mono16", header=Header(frame_id="d", stamp=time_from_seconds(1.0))
    )
    img = image_from_compressed(compressed_image_from_image(src, format="png"))
    assert img.encoding == "mono16"
    assert np.array_equal(image_view(img), data)


def test_png_decode_alpha_is_bgra() -> None:
    import cv2

    arr = np.zeros((10, 10, 4), dtype=np.uint8)
    arr[..., 3] = 128
    ok, buf = cv2.imencode(".png", arr)
    assert ok
    img = image_from_compressed(
        CompressedImage(
            data=np.frombuffer(buf.tobytes(), dtype=np.uint8),
            format="png",
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        )
    )
    assert img.encoding == "bgra8"
    assert np.array_equal(image_view(img), arr)


def test_lcm_roundtrip(rgb_image) -> None:
    ci = compressed_image_from_image(rgb_image)
    wire = cdr_encode(ci)
    out = cdr_decode(wire, CompressedImage)
    np.testing.assert_array_equal(out.data, ci.data)
    assert "; jpeg compressed " in out.format
    assert out.header.frame_id == "cam"
    assert abs(to_nanoseconds(out.header.stamp) - to_nanoseconds(ci.header.stamp)) < 1e-06


def test_jpeg_rejects_depth_formats() -> None:
    depth = image_from_array(
        np.zeros((10, 10), dtype=np.uint16),
        encoding="mono16",
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    with pytest.raises(ValueError, match="JPEG requires 8-bit"):
        compressed_image_from_image(depth)


def test_png_rejects_float_depth() -> None:
    depth = image_from_array(
        np.zeros((10, 10), dtype=np.float32),
        encoding="32FC1",
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    with pytest.raises(ValueError, match="PNG cannot encode"):
        compressed_image_from_image(depth, format="png")


def test_from_image_rejects_non_image() -> None:
    with pytest.raises(TypeError):
        compressed_image_from_image(b"not an image")


def test_max_width_resizes(rgb_image) -> None:
    img = image_from_compressed(compressed_image_from_image(rgb_image, max_width=320))
    assert img.width <= 320
    assert img.height <= 320


def test_to_rerun_is_encoded_image(rgb_image) -> None:
    import rerun as rr

    assert isinstance(image_archetype(compressed_image_from_image(rgb_image)), rr.EncodedImage)


def test_jxl_roundtrip_is_lossless_gray16() -> None:
    data = np.arange(100 * 80, dtype=np.uint16).reshape(100, 80)
    src = image_from_array(
        data, encoding="mono16", header=Header(frame_id="d", stamp=time_from_seconds(1.0))
    )
    ci = compressed_image_from_image(src, format="jxl")
    assert "; jxl compressed " in ci.format
    img = image_from_compressed(ci)
    assert img.encoding == "mono16"
    assert np.array_equal(image_view(img), data)


def test_jxl_roundtrip_is_lossless_float_depth() -> None:
    data = np.linspace(0.1, 10.0, 100 * 80, dtype=np.float32).reshape(100, 80)
    src = image_from_array(
        data, encoding="32FC1", header=Header(frame_id="d", stamp=time_from_seconds(2.0))
    )
    ci = compressed_image_from_image(src, format="jxl")
    img = image_from_compressed(ci)
    assert img.encoding == "32FC1"
    assert np.array_equal(image_view(img), data)
    assert to_nanoseconds(img.header.stamp) == to_nanoseconds(src.header.stamp)
    assert img.header.frame_id == "d"


def test_jxl_lossy_rgb_honors_quality(rgb_image) -> None:
    ci = compressed_image_from_image(rgb_image, format="jxl", quality=90)
    assert 0 < len(ci.data) < image_view(rgb_image).nbytes // 4
    img = image_from_compressed(ci)
    assert img.encoding in ("rgb8", "bgr8")
    diff = np.abs(image_to_rgb(img).astype(int) - image_view(rgb_image).astype(int)).mean()
    assert diff < 5, f"JXL q90 mean pixel error too high: {diff}"


def test_jxl_effort_trades_cpu_for_size(rgb_image) -> None:
    fast = compressed_image_from_image(rgb_image, format="jxl", effort=1)
    thorough = compressed_image_from_image(rgb_image, format="jxl", effort=7)
    assert len(thorough.data) <= len(fast.data)


def test_jxl_rejects_float64() -> None:
    depth = image_from_array(np.zeros((10, 10), dtype=np.float64), encoding="64FC1")
    with pytest.raises(ValueError, match="JXL cannot encode"):
        compressed_image_from_image(depth, format="jxl")


@pytest.mark.parametrize("codec", ["png", "jxl"])
def test_compressed_uint16_depth_keeps_units_and_rerun_depth(codec: str) -> None:
    import rerun as rr

    pixels = np.arange(80, dtype=np.uint16).reshape(10, 8)
    source = image_from_array(
        pixels, encoding="16UC1", header=Header(frame_id="depth", stamp=Time(sec=0, nanosec=0))
    )
    compressed = compressed_image_from_image(source, format=codec)
    decoded = image_from_compressed(compressed)
    assert decoded.encoding == "16UC1"
    assert decoded.header == source.header
    np.testing.assert_array_equal(image_view(decoded), pixels)
    if codec == "jxl":
        assert isinstance(image_archetype(compressed), rr.DepthImage)


@pytest.mark.parametrize("codec", ["png", "jxl"])
def test_compressed_big_endian_depth(codec: str) -> None:
    pixels = np.arange(80, dtype=">u2").reshape(10, 8) * 257
    source = image_from_array(pixels.astype(">u2"), encoding="16UC1")
    decoded = image_from_compressed(compressed_image_from_image(source, format=codec))
    np.testing.assert_array_equal(image_view(decoded), pixels)
