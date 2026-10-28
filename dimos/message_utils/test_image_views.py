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

"""Borrowed buffers preserve padding, byte order, lifetime, and ROS value semantics."""

import gc

import cv2
from dimos_cdrgen.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
from PIL import Image as PILImage
import pytest

from dimos.message_utils import image, time as message_time
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.sensor_msgs.msg import Image
from dimos.messages.std_msgs.msg import Header


def test_padded_image_view_retains_owner_and_is_readonly() -> None:
    msg = Image(
        width=2,
        height=2,
        step=4,
        encoding="mono8",
        data=np.array([1, 2, 99, 99, 3, 4, 99, 99], dtype=np.uint8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        is_bigendian=0,
    )
    pixels = image.view(msg)
    np.testing.assert_array_equal(pixels, [[1, 2], [3, 4]])
    assert pixels.strides == (4, 1)
    assert np.shares_memory(pixels, msg.data)
    msg.data[0] = 5
    assert pixels[0, 0] == 5
    msg.data[0] = 1
    with pytest.raises(ValueError):
        pixels[0, 0] = 9
    del msg
    gc.collect()
    np.testing.assert_array_equal(pixels, [[1, 2], [3, 4]])
    copied = pixels.copy()
    copied[0, 0] = 9
    assert pixels[0, 0] == 1


def test_big_endian_depth_view() -> None:
    msg = Image(
        width=2,
        height=1,
        step=4,
        encoding="16UC1",
        is_bigendian=1,
        data=np.array([0x01, 0x02, 0x03, 0x04], dtype=np.uint8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    np.testing.assert_array_equal(image.view(msg), [[258, 772]])


def test_image_array_copy_preserves_endian_and_handles_strides() -> None:
    pixels = np.arange(24, dtype=">u2").reshape(4, 6)[::2, ::2]
    msg = image.from_array(pixels, encoding="16UC1")
    assert (msg.width, msg.height, msg.step, msg.is_bigendian) == (3, 2, 6, 1)
    expected = pixels.copy()
    pixels[:] = 0
    np.testing.assert_array_equal(image.view(msg), expected)


@pytest.mark.parametrize(
    "pixels,encoding",
    [(np.zeros((2, 2), dtype=np.float32), "mono8"), (np.zeros((2, 2), dtype=np.uint8), "rgb8")],
)
def test_image_array_requires_matching_encoding(pixels: np.ndarray, encoding: str) -> None:
    with pytest.raises(ValueError, match="does not match"):
        image.from_array(pixels, encoding=encoding)


@pytest.mark.parametrize("step,data", [(1, [1]), (2, [1]), (2, [1, 2, 3])])
def test_invalid_image_layout(step: int, data: list[int]) -> None:
    with pytest.raises(ValueError, match="dimensions"):
        image.view(
            Image(
                width=2,
                height=1,
                step=step,
                data=np.asarray(data, dtype=np.uint8),
                encoding="mono8",
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
                is_bigendian=0,
            )
        )


def test_jpeg_preserves_rgb_color() -> None:
    msg = Image(
        width=16,
        height=16,
        step=48,
        encoding="rgb8",
        data=np.asarray([255, 0, 0] * 256, dtype=np.uint8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        is_bigendian=0,
    )
    jpeg = image.to_jpeg(msg, quality=95)
    assert isinstance(jpeg, bytes)
    bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert bgr.shape == (16, 16, 3)
    assert bgr[8, 8, 2] > 250 and bgr[8, 8, 0] < 5


@pytest.mark.parametrize("encoding", ["rgb8", "bgr8", "rgba8", "bgra8", "mono8"])
def test_sharpness_prefers_edges_to_blurred_pixels(encoding):
    mono = ((np.indices((64, 64)).sum(axis=0) % 2) * 255).astype(np.uint8)
    channels = 1 if encoding == "mono8" else (4 if "a" in encoding else 3)
    pixels = mono if channels == 1 else np.repeat(mono[:, :, None], channels, axis=2)
    blurred = cv2.GaussianBlur(pixels, (5, 5), 0)
    sharp = image.from_array(pixels, encoding=encoding)
    soft = image.from_array(blurred, encoding=encoding)
    assert image.sharpness(sharp) > image.sharpness(soft)


def test_sharpness_rejects_unsupported_depth_and_empty_images():
    depth = image.from_array(np.zeros((2, 2, 3), dtype=np.float32), encoding="32FC3")
    with pytest.raises(ValueError, match="Sharpness requires"):
        image.sharpness(depth)
    with pytest.raises(ValueError, match="nonempty"):
        image.sharpness(
            Image(
                encoding="mono8",
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
                height=0,
                width=0,
                is_bigendian=0,
                step=0,
                data=np.array([], dtype=np.uint8),
            )
        )


@pytest.mark.parametrize(
    "encoding,pixel",
    [
        ("rgb8", [255, 0, 17]),
        ("bgr8", [17, 0, 255]),
        ("rgba8", [255, 0, 17, 23]),
        ("bgra8", [17, 0, 255, 23]),
    ],
)
def test_rgb_model_input_is_color_correct_and_independent(encoding: str, pixel: list[int]) -> None:
    source = image.from_array(np.array([[pixel]], dtype=np.uint8), encoding=encoding)
    restored = cdr_decode(cdr_encode(source), Image)
    rgb = image.to_rgb(restored)
    np.testing.assert_array_equal(rgb, [[[255, 0, 17]]])
    rgb[0, 0] = 0
    np.testing.assert_array_equal(image.view(restored), [[pixel]])


def test_rgb_model_input_respects_big_endian_padded_grayscale() -> None:
    msg = Image(
        width=2,
        height=2,
        step=6,
        encoding="mono16",
        is_bigendian=1,
        data=np.array([0x12, 0x34, 0xFF, 0xFF, 99, 99, 0x01, 0x00, 0, 0, 99, 99], dtype=np.uint8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    np.testing.assert_array_equal(
        image.to_rgb(msg), [[[18, 18, 18], [255, 255, 255]], [[1, 1, 1], [0, 0, 0]]]
    )


def test_rgb_model_input_requires_explicit_float_depth_scale() -> None:
    msg = image.from_array(np.array([[2.5]], dtype=np.float32), encoding="32FC1")
    with pytest.raises(ValueError, match="RGB8"):
        image.to_rgb(msg)


def test_brightness_ignores_padding_and_normalizes_big_endian_u16() -> None:
    message = Image(
        width=2,
        height=1,
        step=6,
        encoding="mono16",
        is_bigendian=1,
        data=np.array([0, 0, 255, 255, 255, 255], dtype=np.uint8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    assert image.brightness(message) == pytest.approx(0.5)
    depth = image.from_array(np.array([[1.0]], dtype=np.float32), encoding="32FC1")
    assert image.brightness(depth) == pytest.approx(1.0 / 255.0)


@pytest.mark.parametrize("mode", ["L", "RGBA"])
def test_image_file_decodes_rgb_and_preserves_source_header(tmp_path, mode):
    path = tmp_path / "frame.png"
    pixels = np.array([[7, 19]], dtype=np.uint8)
    source = PILImage.fromarray(pixels).convert(mode)
    source.save(path)
    header = Header(stamp=message_time.time_from_nanoseconds(1234567890), frame_id="camera")
    message = image.from_file(path, header=header)
    assert message.encoding == "rgb8"
    assert cdr_encode(message.header) == cdr_encode(header)
    np.testing.assert_array_equal(image.view(message), [[[7, 7, 7], [19, 19, 19]]])


@pytest.mark.parametrize(
    "encoding,dtype,shape",
    [("64FC1", ">f8", (3, 4)), ("16SC1", ">i2", (3, 4)), ("32FC3", ">f4", (3, 4, 3))],
)
def test_remaining_raw_encodings_keep_signed_float_and_endian_values(encoding, dtype, shape):
    pixels = (np.arange(np.prod(shape)).reshape(shape) - 5).astype(dtype)
    source = image.from_array(pixels, encoding=encoding)
    decoded = cdr_decode(cdr_encode(source), Image)
    assert decoded.encoding == encoding
    assert decoded.is_bigendian == 1
    np.testing.assert_array_equal(image.view(decoded), pixels)


@pytest.mark.parametrize("encoding", ["rgb8", "bgr8"])
def test_bgr_array_is_writable_and_independent(encoding: str) -> None:
    message = image.from_array(np.full((2, 3, 3), 17, dtype=np.uint8), encoding=encoding)
    pixels = image.to_bgr(message)
    pixels[:] = 0
    assert np.all(image.view(message) == 17)


def test_resize_reuses_fitting_message_and_copies_downscaled_pixels() -> None:
    message = image.from_array(np.full((4, 8, 3), 17, dtype=np.uint8), encoding="rgb8")
    unchanged, scale = image.resize_to_fit(message, 8, 4)
    assert unchanged is message
    assert scale == 1.0
    resized, scale = image.resize_to_fit(message, 4, 4)
    assert resized is not message
    assert (resized.width, resized.height, scale) == (4, 2, 0.5)
    assert resized.encoding == message.encoding
    assert resized.header == message.header
    resized.data[:] = 0
    assert np.all(image.view(message) == 17)


@pytest.mark.parametrize(
    "encoding, channels", [("mono8", 1), ("rgb8", 3), ("bgr8", 3), ("rgba8", 4), ("bgra8", 4)]
)
def test_sharpness_preserves_normalized_score(encoding, channels):
    pixels = np.zeros((8, 8), dtype=np.uint8)
    pixels[:, 4:] = 32
    if channels > 1:
        pixels = np.repeat(pixels[:, :, None], channels, axis=2)
    assert image.sharpness(image.from_array(pixels, encoding=encoding)) == pytest.approx(
        0.4699777662754059
    )


def test_sharpness_preserves_floor_resize_and_grayscale_support():
    rng = np.random.default_rng(17)
    pixels = rng.integers(0, 256, (173, 321, 3), dtype=np.uint8)
    assert image.sharpness(image.from_array(pixels, encoding="rgb8")) == pytest.approx(
        0.9509032368659973
    )
    for encoding, dtype in [("mono16", np.uint16), ("32FC1", np.float32)]:
        assert (
            image.sharpness(image.from_array(np.zeros((8, 8), dtype=dtype), encoding=encoding))
            == 0.0
        )
