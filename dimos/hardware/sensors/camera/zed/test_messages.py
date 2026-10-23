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

import math

from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest

from dimos.hardware.sensors.camera.zed.compat import CameraInfo as ZedCalibration
from dimos.hardware.sensors.camera.zed.messages import (
    frame_message,
    tracking_to_base,
    transform_from_arrays,
)
from dimos.message_utils.geometry import quaternion_from_euler, transform_matrix
from dimos.message_utils.time import time_from_nanoseconds, to_nanoseconds
from dimos.messages.geometry_msgs.msg import Transform, Vector3
from dimos.messages.tf2_msgs.msg import TFMessage


@pytest.mark.parametrize("ns", [0, -1, 1700000000123456789])
def test_frames_roundtrip_cdr_preserves_exact_stamp_and_extrinsics(ns):
    stamp = time_from_nanoseconds(ns)
    extrinsics = Transform(
        translation=Vector3(x=1, y=2, z=3), rotation=quaternion_from_euler(0, 0, math.pi / 2)
    )
    mount = Transform(translation=Vector3(x=4, y=5, z=6), rotation=quaternion_from_euler(0.2, 0, 0))

    message = frame_message(
        camera_name="zed",
        base_frame="base",
        base_transform=mount,
        sdk_extrinsics=extrinsics,
        stamp=stamp,
    )
    actual = decode(encode(message), TFMessage)

    assert [(edge.header.frame_id, edge.child_frame_id) for edge in actual.transforms] == [
        ("base", "zed_link"),
        ("zed_link", "zed_depth_frame"),
        ("zed_depth_frame", "zed_depth_optical_frame"),
        ("zed_link", "zed_color_frame"),
        ("zed_color_frame", "zed_color_optical_frame"),
    ]
    assert [to_nanoseconds(edge.header.stamp) for edge in actual.transforms] == [ns] * 5
    np.testing.assert_allclose(
        transform_matrix(actual.transforms[0].transform), transform_matrix(mount)
    )
    for edge in [actual.transforms[1], actual.transforms[3]]:
        np.testing.assert_allclose(
            transform_matrix(edge.transform),
            np.linalg.inv(transform_matrix(extrinsics)),
            atol=1e-12,
        )
    for edge in [actual.transforms[2], actual.transforms[4]]:
        optical_to_frame = transform_matrix(edge.transform)
        # Optical right/down/forward maps to camera forward/left/up.
        np.testing.assert_allclose(optical_to_frame @ [1, 2, 3, 1], [3, -1, -2, 1], atol=1e-12)


def test_tracking_removes_rotated_mount_and_is_included_without_mount_edge():
    stamp = time_from_nanoseconds(1700000000123456789)
    world_camera = transform_from_arrays(
        [5, 6, 7],
        [0, 0, math.sqrt(0.5), math.sqrt(0.5)],
        parent="world",
        child="zed_link",
        stamp=stamp,
    )
    mount = Transform(
        translation=Vector3(x=1, y=2, z=3), rotation=quaternion_from_euler(0, 0, -math.pi / 2)
    )

    tracking = tracking_to_base(world_camera, mount, "base")
    message = frame_message(
        camera_name="zed",
        base_frame="base",
        base_transform=None,
        sdk_extrinsics=mount,
        stamp=stamp,
        tracking=tracking,
    )
    actual = decode(encode(message), TFMessage)

    assert tracking.header.frame_id == "world"
    assert tracking.child_frame_id == "base"
    assert to_nanoseconds(tracking.header.stamp) == 1700000000123456789
    np.testing.assert_allclose(
        transform_matrix(tracking.transform),
        transform_matrix(world_camera.transform) @ np.linalg.inv(transform_matrix(mount)),
        atol=1e-12,
    )
    assert len(actual.transforms) == 5
    assert actual.transforms[-1] == tracking
    assert actual.transforms[0].header.frame_id == "zed_link"
    assert tracking_to_base(world_camera, None, "base") == world_camera


@pytest.mark.parametrize("translation,rotation", [([1, 2], [0, 0, 0, 1]), ([1, 2, 3], [0, 0, 1])])
def test_incomplete_sdk_pose_is_rejected(translation, rotation):
    with pytest.raises(ValueError, match="three translation and four quaternion"):
        transform_from_arrays(
            translation, rotation, parent="world", child="camera", stamp=time_from_nanoseconds(0)
        )


def test_shipped_zed_calibration_is_generated_and_roundtrips_cdr():
    calibration = ZedCalibration.SingleWebcam

    actual = decode(encode(calibration), type(calibration))

    assert (actual.width, actual.height) == (640, 376)
    assert actual.header.frame_id == "camera_optical"
    assert to_nanoseconds(actual.header.stamp) == 0
    assert actual.k[0] == pytest.approx(379.45267)
    assert actual.d[0] == pytest.approx(-0.309435)
