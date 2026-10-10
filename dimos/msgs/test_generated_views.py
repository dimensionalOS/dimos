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
import json
import math

import cv2
from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
from PIL import Image as PILImage
import pytest

from dimos.msgs.geometry import yaw
from dimos.msgs.image import (
    image_brightness,
    image_from_array,
    image_from_file,
    image_sharpness,
    image_to_jpeg,
    image_to_rgb,
    image_view,
)
from dimos.msgs.occupancy import block_max_reduce, occupancy_view
from dimos.msgs.time import time_from_nanoseconds
from dimos.web.relay_bridge.builtin_codecs import decode_point, encode_path, encode_pose


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
    pixels = image_view(msg)
    np.testing.assert_array_equal(pixels, [[1, 2], [3, 4]])
    assert pixels.strides == (4, 1)
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
    np.testing.assert_array_equal(image_view(msg), [[258, 772]])


def test_image_array_copy_preserves_endian_and_handles_strides() -> None:
    pixels = np.arange(24, dtype=">u2").reshape(4, 6)[::2, ::2]
    msg = image_from_array(pixels, encoding="16UC1")
    assert (msg.width, msg.height, msg.step, msg.is_bigendian) == (3, 2, 6, 1)
    expected = pixels.copy()
    pixels[:] = 0
    np.testing.assert_array_equal(image_view(msg), expected)


@pytest.mark.parametrize(
    "pixels,encoding",
    [(np.zeros((2, 2), dtype=np.float32), "mono8"), (np.zeros((2, 2), dtype=np.uint8), "rgb8")],
)
def test_image_array_requires_matching_encoding(pixels: np.ndarray, encoding: str) -> None:
    with pytest.raises(ValueError, match="does not match"):
        image_from_array(pixels, encoding=encoding)


@pytest.mark.parametrize("step,data", [(1, [1]), (2, [1]), (2, [1, 2, 3])])
def test_invalid_image_layout(step: int, data: list[int]) -> None:
    with pytest.raises(ValueError, match="dimensions"):
        image_view(
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
    jpeg = image_to_jpeg(msg, quality=95)
    bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert bgr.shape == (16, 16, 3)
    assert bgr[8, 8, 2] > 250 and bgr[8, 8, 0] < 5


def test_occupancy_view_and_obstacle_preserving_reduction() -> None:
    msg = OccupancyGrid(
        info=MapMetaData(
            width=4,
            height=2,
            map_load_time=Time(sec=0, nanosec=0),
            resolution=0.0,
            origin=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=np.array([-1, -1, 0, 100, -1, -1, 10, 20], dtype=np.int8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    cells = occupancy_view(msg)
    assert not cells.flags.writeable
    np.testing.assert_array_equal(block_max_reduce(cells, 2), [[-1, 100]])
    msg.info.width = 3
    with pytest.raises(ValueError, match="dimensions"):
        occupancy_view(msg)


def test_web_pose_path_and_goal_use_nested_ros_fields() -> None:
    pose = PoseStamped(
        header=Header(stamp=time_from_nanoseconds(1500000000), frame_id=""),
        pose=Pose(
            position=Point(x=2, y=3, z=0.0),
            orientation=Quaternion(z=math.sqrt(0.5), w=math.sqrt(0.5), x=0.0, y=0.0),
        ),
    )
    assert yaw(pose.pose.orientation) == pytest.approx(math.pi / 2)
    value = json.loads(encode_pose(pose))
    assert value == {"x": 2, "y": 3, "z": 0, "yaw": pytest.approx(math.pi / 2), "ts": 1.5}
    assert json.loads(
        encode_path(Path(poses=[pose], header=Header(stamp=Time(sec=0, nanosec=0), frame_id="")))
    ) == [[2, 3]]
    goal = decode_point({"x": 4, "y": 5})
    assert goal.point == Point(x=4, y=5, z=0.0) and goal.header.frame_id == "world"


@pytest.mark.parametrize("encoding", ["rgb8", "bgr8", "rgba8", "bgra8", "mono8"])
def test_sharpness_prefers_edges_to_blurred_pixels(encoding):
    mono = ((np.indices((64, 64)).sum(axis=0) % 2) * 255).astype(np.uint8)
    channels = 1 if encoding == "mono8" else (4 if "a" in encoding else 3)
    pixels = mono if channels == 1 else np.repeat(mono[:, :, None], channels, axis=2)
    blurred = cv2.GaussianBlur(pixels, (5, 5), 0)
    sharp = image_from_array(pixels, encoding=encoding)
    soft = image_from_array(blurred, encoding=encoding)
    assert image_sharpness(sharp) > image_sharpness(soft)


def test_sharpness_rejects_depth_and_empty_images():
    depth = image_from_array(np.zeros((2, 2), dtype=np.float32), encoding="32FC1")
    with pytest.raises(ValueError, match="8-bit visual"):
        image_sharpness(depth)
    with pytest.raises(ValueError, match="nonempty"):
        image_sharpness(
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
    source = image_from_array(np.array([[pixel]], dtype=np.uint8), encoding=encoding)
    restored = cdr_decode(cdr_encode(source), Image)
    rgb = image_to_rgb(restored)
    np.testing.assert_array_equal(rgb, [[[255, 0, 17]]])
    rgb[0, 0] = 0
    np.testing.assert_array_equal(image_view(restored), [[pixel]])


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
        image_to_rgb(msg), [[[18, 18, 18], [255, 255, 255]], [[1, 1, 1], [0, 0, 0]]]
    )


def test_rgb_model_input_requires_explicit_float_depth_scale() -> None:
    msg = image_from_array(np.array([[2.5]], dtype=np.float32), encoding="32FC1")
    with pytest.raises(ValueError, match="RGB8"):
        image_to_rgb(msg)


def test_mosaic_uses_generated_pixels_and_preserves_rgb_colors() -> None:
    from dimos.memory.vis.utils import mosaic

    red = image_from_array(np.array([[[255, 0, 0]]], dtype=np.uint8), encoding="rgb8")
    blue = image_from_array(np.array([[[255, 0, 0]]], dtype=np.uint8), encoding="bgr8")
    observation = mosaic([red, blue], cols=2, cell_height=1)
    result = cdr_decode(cdr_encode(observation.data), Image)
    assert result.encoding == "bgr8"
    np.testing.assert_array_equal(image_to_rgb(result), [[[255, 0, 0], [0, 0, 255]]])
    assert observation.tags == {"mosaic": True}
    assert observation.pose is None


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
    assert image_brightness(message) == pytest.approx(0.5)
    depth = image_from_array(np.array([[1.0]], dtype=np.float32), encoding="32FC1")
    with pytest.raises(ValueError, match="unsigned integer"):
        image_brightness(depth)


@pytest.mark.parametrize("mode", ["L", "RGBA"])
def test_image_file_decodes_rgb_and_preserves_source_header(tmp_path, mode):
    path = tmp_path / "frame.png"
    pixels = np.array([[7, 19]], dtype=np.uint8)
    source = PILImage.fromarray(pixels).convert(mode)
    source.save(path)
    header = Header(stamp=time_from_nanoseconds(1234567890), frame_id="camera")
    message = image_from_file(path, header=header)
    assert message.encoding == "rgb8"
    assert cdr_encode(message.header) == cdr_encode(header)
    np.testing.assert_array_equal(image_view(message), [[[7, 7, 7], [19, 19, 19]]])


@pytest.mark.parametrize(
    "encoding,dtype,shape",
    [("64FC1", ">f8", (3, 4)), ("16SC1", ">i2", (3, 4)), ("32FC3", ">f4", (3, 4, 3))],
)
def test_remaining_raw_encodings_keep_signed_float_and_endian_values(encoding, dtype, shape):
    pixels = (np.arange(np.prod(shape)).reshape(shape) - 5).astype(dtype)
    source = image_from_array(pixels, encoding=encoding)
    decoded = cdr_decode(cdr_encode(source), Image)
    assert decoded.encoding == encoding
    assert decoded.is_bigendian == 1
    np.testing.assert_array_equal(image_view(decoded), pixels)
