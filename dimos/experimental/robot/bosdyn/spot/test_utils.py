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

from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from dimos.experimental.robot.bosdyn.spot.effectors.high_level import SpotHighLevel
from dimos.experimental.robot.bosdyn.spot.utils import (
    camera_info_from_response,
    camera_mount_transforms,
    decode_image,
    roll_optical_frame,
    rotate_camera_info_quarter_turns,
    rotate_image_quarter_turns,
)
from dimos.message_utils import camera_info, geometry, image
from dimos.message_utils.time import time_from_seconds, to_seconds
from dimos.messages.geometry_msgs.msg import Point, Quaternion, Transform, TransformStamped, Vector3
from dimos.messages.std_msgs.msg import Header


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
    assert [
        transforms[0].transform.translation.x,
        transforms[0].transform.translation.y,
        transforms[0].transform.translation.z,
    ] == [1.0, 2.0, 0.0]


def test_upright_rotation_preserves_pixels_calibration_and_stamp() -> None:
    header = Header(stamp=time_from_seconds(12.25), frame_id="camera")
    pixels = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.uint16)
    source = image.from_array(pixels, encoding="16UC1", header=header)
    rotated = rotate_image_quarter_turns(source, 1)
    np.testing.assert_array_equal(image.view(rotated), [[3, 6], [2, 5], [1, 4]])
    assert rotated.header == header and rotated.encoding == "16UC1"
    info = camera_info.from_intrinsics(10.0, 20.0, 0.75, 0.25, 3, 2, header=header)
    upright = rotate_camera_info_quarter_turns(info, 1)
    assert (upright.width, upright.height) == (2, 3)
    np.testing.assert_allclose(upright.k, [20.0, 0.0, 0.25, 0.0, 10.0, 1.25, 0.0, 0.0, 1.0])
    assert upright.header == header
    transform = TransformStamped(
        header=header,
        child_frame_id="optical",
        transform=Transform(
            translation=Vector3(1.0, 2.0, 3.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
        ),
    )
    rolled = roll_optical_frame(transform, 1)
    assert rolled.header == header and rolled.child_frame_id == "optical"
    assert rolled.transform.translation == Vector3(1.0, 2.0, 3.0)
    np.testing.assert_allclose(
        geometry.vector_array(
            geometry.rotate_vector(rolled.transform.rotation, Vector3(1.0, 0.0, 0.0))
        ),
        [0.0, 1.0, 0.0],
        atol=1e-15,
    )


@pytest.fixture
def bosdyn_image_types(monkeypatch):
    """Only SDK enum constants are needed to exercise the image byte boundary."""
    constants = SimpleNamespace(
        FORMAT_JPEG=1,
        FORMAT_RAW=2,
        PIXEL_FORMAT_GREYSCALE_U8=3,
        PIXEL_FORMAT_GREYSCALE_U16=4,
        PIXEL_FORMAT_DEPTH_U16=5,
        PIXEL_FORMAT_RGB_U8=6,
        PIXEL_FORMAT_RGBA_U8=7,
    )
    for name in ["bosdyn", "bosdyn.api", "bosdyn.api.image_pb2"]:
        module = ModuleType(name)
        if name.endswith("image_pb2"):
            module.Image = constants
        monkeypatch.setitem(sys.modules, name, module)
    return constants


@pytest.mark.parametrize(
    ("pixel_format", "encoding", "dtype", "pixels"),
    [
        ("PIXEL_FORMAT_GREYSCALE_U8", "mono8", np.uint8, [[1, 2], [3, 4]]),
        ("PIXEL_FORMAT_GREYSCALE_U16", "mono16", np.uint16, [[1200, 2200], [3200, 4200]]),
        ("PIXEL_FORMAT_DEPTH_U16", "16UC1", np.uint16, [[1200, 2200], [3200, 4200]]),
        ("PIXEL_FORMAT_RGB_U8", "rgb8", np.uint8, [[[3, 15, 200]]]),
        ("PIXEL_FORMAT_RGBA_U8", "rgba8", np.uint8, [[[3, 15, 200, 180]]]),
    ],
)
def test_decode_raw_sdk_image_keeps_pixels_and_capture_time(
    bosdyn_image_types, pixel_format, encoding, dtype, pixels
):
    values = np.array(pixels, dtype=dtype)
    response = SimpleNamespace(
        shot=SimpleNamespace(
            acquisition_time="robot-capture",
            image=SimpleNamespace(
                pixel_format=getattr(bosdyn_image_types, pixel_format),
                format=bosdyn_image_types.FORMAT_RAW,
                rows=values.shape[0],
                cols=values.shape[1],
                data=values.tobytes(),
            ),
        )
    )
    clock = Mock()
    clock.local_seconds_from_robot_timestamp.return_value = 42.125
    result = decode_image(response, "frontleft_optical", clock)
    assert result is not None
    assert result.encoding == encoding and result.header.frame_id == "frontleft_optical"
    assert to_seconds(result.header.stamp) == 42.125
    np.testing.assert_array_equal(image.view(result), values)
    clock.local_seconds_from_robot_timestamp.assert_called_once_with("robot-capture")


def test_odometry_boundary_preserves_sdk_pose_velocity_and_tf():
    state = SimpleNamespace(x=1.25, y=-2.5, z=0.75, rot=SimpleNamespace(x=0.1, y=0.2, z=0.3, w=0.9))
    velocity = SimpleNamespace(
        linear=SimpleNamespace(x=0.3, y=-0.2, z=0.1), angular=SimpleNamespace(x=-0.4, y=0.5, z=-0.6)
    )
    publisher = SimpleNamespace(
        config=SimpleNamespace(odom_frame_id="vision", base_frame_id="body"),
        odometry=Mock(),
        tf=Mock(),
    )
    SpotHighLevel._publish_odom(publisher, state, velocity, 42.125)
    (odom,) = publisher.odometry.publish.call_args.args
    assert odom.header.frame_id == "vision" and odom.child_frame_id == "body"
    assert to_seconds(odom.header.stamp) == 42.125
    assert odom.pose.pose.position == Point(1.25, -2.5, 0.75)
    assert odom.pose.pose.orientation == Quaternion(0.1, 0.2, 0.3, 0.9)
    assert odom.twist.twist.linear == Vector3(0.3, -0.2, 0.1)
    assert odom.twist.twist.angular == Vector3(-0.4, 0.5, -0.6)
    np.testing.assert_array_equal(odom.pose.covariance, np.zeros(36))
    (tf,) = publisher.tf.publish.call_args.args
    assert tf.transforms[0].header == odom.header
    assert tf.transforms[0].child_frame_id == "body"
    assert tf.transforms[0].transform.translation == Vector3(1.25, -2.5, 0.75)
    assert tf.transforms[0].transform.rotation == odom.pose.pose.orientation


def test_decode_jpeg_sdk_image_keeps_bgr_pixels_and_capture_stamp(bosdyn_image_types):
    pixels = np.full((8, 8, 3), [17, 35, 210], dtype=np.uint8)
    ok, jpeg = cv2.imencode(".jpg", pixels)
    assert ok
    response = SimpleNamespace(
        shot=SimpleNamespace(
            acquisition_time="capture",
            image=SimpleNamespace(
                pixel_format=bosdyn_image_types.PIXEL_FORMAT_RGB_U8,
                format=bosdyn_image_types.FORMAT_JPEG,
                data=jpeg.tobytes(),
            ),
        )
    )
    clock = Mock()
    clock.local_seconds_from_robot_timestamp.return_value = 17.75
    result = decode_image(response, "optical", clock)
    assert result is not None
    assert result.encoding == "bgr8" and result.header.frame_id == "optical"
    assert to_seconds(result.header.stamp) == 17.75
    np.testing.assert_allclose(image.view(result)[4, 4], [17, 35, 210], atol=3)


def test_sdk_pinhole_uses_source_intrinsics_and_capture_stamp():
    response = SimpleNamespace(
        source=SimpleNamespace(
            HasField=lambda field: field == "pinhole",
            cols=640,
            rows=480,
            pinhole=SimpleNamespace(
                intrinsics=SimpleNamespace(
                    focal_length=SimpleNamespace(x=410.0, y=420.0),
                    principal_point=SimpleNamespace(x=315.0, y=237.0),
                )
            ),
        )
    )
    info = camera_info_from_response(response, "optical", 23.625)
    assert info is not None
    assert info.header.frame_id == "optical" and to_seconds(info.header.stamp) == 23.625
    assert (info.width, info.height) == (640, 480)
    np.testing.assert_array_equal(info.k, [410.0, 0.0, 315.0, 0.0, 420.0, 237.0, 0.0, 0.0, 1.0])
