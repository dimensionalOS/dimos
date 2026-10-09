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

import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import cv2
from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.experimental.robot.bosdyn.spot.utils import (
    camera_info_from_response,
    camera_mount_transforms,
    decode_image,
    roll_optical_frame,
    rotate_camera_info_quarter_turns,
    rotate_image_quarter_turns,
)
from dimos.msgs.camera_info import camera_info_from_intrinsics
from dimos.msgs.geometry import yaw
from dimos.msgs.image import image_from_array, image_view


def test_camera_mount_transforms_uses_loaded_robot_topology(tmp_path: Path) -> None:
    urdf = tmp_path / "spot.urdf"
    urdf.write_text(
        """
        <robot name="spot">
          <link name="base"/>
          <link name="camera_mount"/>
          <link name="camera_optical"/>
          <joint name="mount" type="fixed">
            <origin xyz="1 0 0"/>
            <parent link="base"/>
            <child link="camera_mount"/>
          </joint>
          <joint name="optical" type="fixed">
            <origin xyz="0 2 0"/>
            <parent link="camera_mount"/>
            <child link="camera_optical"/>
          </joint>
        </robot>
        """
    )

    transforms = camera_mount_transforms(urdf, "body", ["camera_optical"])

    assert len(transforms) == 1
    assert transforms[0].header.frame_id == "body"
    assert transforms[0].child_frame_id == "camera_optical"
    translation = transforms[0].transform.translation
    assert (translation.x, translation.y, translation.z) == (1.0, 2.0, 0.0)


@pytest.mark.parametrize("turns", [-1, 0, 1])
def test_optical_roll_preserves_mount_and_exact_stamp(turns):
    edge = TransformStamped(
        header=Header(frame_id="body", stamp=Time(sec=1700000000, nanosec=123456789)),
        child_frame_id="optical",
        transform=Transform(
            translation=Vector3(x=1, y=2, z=3), rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        ),
    )
    rolled = roll_optical_frame(edge, turns)
    assert rolled.header == edge.header
    assert rolled.child_frame_id == edge.child_frame_id
    assert rolled.transform.translation == edge.transform.translation
    assert yaw(rolled.transform.rotation) == pytest.approx(turns * math.pi / 2)
    assert edge.transform.rotation.w == 1


@pytest.fixture
def image_sdk(monkeypatch):
    constants = SimpleNamespace(
        FORMAT_JPEG=1,
        FORMAT_RAW=2,
        PIXEL_FORMAT_GREYSCALE_U8=1,
        PIXEL_FORMAT_RGB_U8=3,
        PIXEL_FORMAT_RGBA_U8=4,
        PIXEL_FORMAT_DEPTH_U16=5,
        PIXEL_FORMAT_GREYSCALE_U16=6,
    )
    root = ModuleType("bosdyn")
    root.__path__ = []
    api = ModuleType("bosdyn.api")
    api.image_pb2 = SimpleNamespace(Image=constants)
    monkeypatch.setitem(sys.modules, "bosdyn", root)
    monkeypatch.setitem(sys.modules, "bosdyn.api", api)
    return constants


@pytest.mark.parametrize(
    "encoding,pixel_format,dtype,channels",
    [
        ("mono8", 1, np.uint8, 1),
        ("rgb8", 3, np.uint8, 3),
        ("rgba8", 4, np.uint8, 4),
        ("16UC1", 5, np.uint16, 1),
        ("mono16", 6, np.uint16, 1),
    ],
)
def test_sdk_raw_camera_boundary_emits_generated_pixels(
    image_sdk, encoding, pixel_format, dtype, channels
):
    shape = (2, 3) if channels == 1 else (2, 3, channels)
    pixels = np.arange(np.prod(shape), dtype=dtype).reshape(shape)
    shot = SimpleNamespace(
        format=image_sdk.FORMAT_RAW,
        pixel_format=pixel_format,
        rows=2,
        cols=3,
        data=pixels.tobytes(),
    )
    response = SimpleNamespace(shot=SimpleNamespace(image=shot, acquisition_time="robot-clock"))
    clock = SimpleNamespace(local_seconds_from_robot_timestamp=lambda stamp: 1700000000.25)
    value = decode_image(response, "camera-optical", clock)
    assert type(value) is Image and value.encoding == encoding
    decoded = cdr_decode(value.encode(), Image)
    assert decoded.header.frame_id == "camera-optical"
    assert decoded.header.stamp == Time(sec=1700000000, nanosec=250000000)
    np.testing.assert_array_equal(image_view(decoded), pixels)


def test_sdk_jpeg_camera_boundary_decodes_generated_image(image_sdk):
    pixels = np.full((8, 8, 3), [20, 80, 140], dtype=np.uint8)
    ok, jpeg = cv2.imencode(".jpg", pixels)
    assert ok
    shot = SimpleNamespace(format=image_sdk.FORMAT_JPEG, pixel_format=3, data=jpeg.tobytes())
    response = SimpleNamespace(shot=SimpleNamespace(image=shot, acquisition_time="robot-clock"))
    clock = SimpleNamespace(local_seconds_from_robot_timestamp=lambda stamp: 1700000000.25)
    value = decode_image(response, "camera-optical", clock)
    assert type(value) is Image and value.encoding == "bgr8"
    np.testing.assert_allclose(image_view(cdr_decode(value.encode(), Image)), pixels, atol=2)


@pytest.mark.parametrize("turns", [-1, 0, 1, 2, 4])
def test_generated_camera_rotation_preserves_depth_header_and_intrinsics(turns):
    header = Header(frame_id="optical", stamp=Time(sec=-1, nanosec=123456789))
    pixels = np.arange(6, dtype=np.uint16).reshape(2, 3) * 1000
    image = image_from_array(pixels, encoding="16UC1", header=header)
    info = camera_info_from_intrinsics(10, 20, 1, 0.5, 3, 2, header=header)
    before_image, before_info = cdr_encode(image), cdr_encode(info)
    rotated = rotate_image_quarter_turns(image, turns)
    calibration = rotate_camera_info_quarter_turns(info, turns)
    np.testing.assert_array_equal(image_view(rotated), np.rot90(pixels, k=turns))
    assert rotated.header == calibration.header == header
    assert (calibration.height, calibration.width) == image_view(rotated).shape
    expected_fx, expected_fy, cx, cy, width, height = 10, 20, 1, 0.5, 3, 2
    for _ in range(turns % 4):
        expected_fx, expected_fy = expected_fy, expected_fx
        cx, cy = cy, width - 1 - cx
        width, height = height, width
    assert list(calibration.k) == [expected_fx, 0, cx, 0, expected_fy, cy, 0, 0, 1]
    assert cdr_encode(image) == before_image and cdr_encode(info) == before_info


def test_sdk_calibration_boundary_emits_plain_generated_value():
    source = SimpleNamespace(
        HasField=lambda name: name == "pinhole",
        cols=6,
        rows=4,
        pinhole=SimpleNamespace(
            intrinsics=SimpleNamespace(
                focal_length=SimpleNamespace(x=10, y=20), principal_point=SimpleNamespace(x=3, y=2)
            )
        ),
    )
    value = camera_info_from_response(SimpleNamespace(source=source), "optical", 1700000000.25)
    assert list(value.k) == [10, 0, 3, 0, 20, 2, 0, 0, 1]
    assert value.header.stamp == Time(sec=1700000000, nanosec=250000000)
