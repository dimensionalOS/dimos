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
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.msgs.geometry import yaw
from dimos.msgs.image import image_from_array, image_to_jpeg, image_view
from dimos.msgs.occupancy import block_max_reduce, occupancy_view
from dimos.msgs.time import time_from_nanoseconds
from dimos.web.relay_bridge.builtin_codecs import decode_point, encode_path, encode_pose


def test_padded_image_view_retains_owner_and_is_readonly() -> None:
    msg = Image(width=2, height=2, step=4, encoding="mono8", data=[1, 2, 99, 99, 3, 4, 99, 99])
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
        width=2, height=1, step=4, encoding="16UC1", is_bigendian=1, data=[0x01, 0x02, 0x03, 0x04]
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
        image_view(Image(width=2, height=1, step=step, data=data, encoding="mono8"))


def test_jpeg_preserves_rgb_color() -> None:
    msg = Image(width=16, height=16, step=48, encoding="rgb8", data=[255, 0, 0] * 256)
    jpeg = image_to_jpeg(msg, quality=95)
    bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert bgr.shape == (16, 16, 3)
    assert bgr[8, 8, 2] > 250 and bgr[8, 8, 0] < 5


def test_occupancy_view_and_obstacle_preserving_reduction() -> None:
    msg = OccupancyGrid(info=MapMetaData(width=4, height=2), data=[-1, -1, 0, 100, -1, -1, 10, 20])
    cells = occupancy_view(msg)
    assert not cells.flags.writeable
    np.testing.assert_array_equal(block_max_reduce(cells, 2), [[-1, 100]])
    msg.info.width = 3
    with pytest.raises(ValueError, match="dimensions"):
        occupancy_view(msg)


def test_web_pose_path_and_goal_use_nested_ros_fields() -> None:
    pose = PoseStamped(
        header=Header(stamp=time_from_nanoseconds(1500000000)),
        pose=Pose(
            position=Point(x=2, y=3), orientation=Quaternion(z=math.sqrt(0.5), w=math.sqrt(0.5))
        ),
    )
    assert yaw(pose.pose.orientation) == pytest.approx(math.pi / 2)
    value = json.loads(encode_pose(pose))
    assert value == {"x": 2, "y": 3, "z": 0, "yaw": pytest.approx(math.pi / 2), "ts": 1.5}
    assert json.loads(encode_path(Path(poses=[pose]))) == [[2, 3]]
    goal = decode_point({"x": 4, "y": 5})
    assert goal.point == Point(x=4, y=5) and goal.header.frame_id == "world"
