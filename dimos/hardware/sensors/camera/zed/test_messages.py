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

from enum import IntEnum
import importlib
import math
import sys
from types import ModuleType

import numpy as np
import pytest

from dimos.message_utils import geometry, time as message_time
from dimos.messages.geometry_msgs.msg import Transform, Vector3


@pytest.fixture
def camera(monkeypatch, mocker):
    # Mock the unavailable hardware SDK; exercise the real camera methods.
    sdk = ModuleType("pyzed.sl")
    sdk.DEPTH_MODE = IntEnum("DEPTH_MODE", {"NEURAL": 0})
    sdk.REFERENCE_FRAME = IntEnum("REFERENCE_FRAME", {"WORLD": 0})
    sdk.POSITIONAL_TRACKING_STATE = IntEnum("POSITIONAL_TRACKING_STATE", {"OK": 0})
    package = ModuleType("pyzed")
    package.sl = sdk
    name = "dimos.hardware.sensors.camera.zed.camera"
    with monkeypatch.context() as context:
        context.setitem(sys.modules, "pyzed", package)
        context.setitem(sys.modules, "pyzed.sl", sdk)
        context.delitem(sys.modules, name, raising=False)
        module = importlib.import_module(name)
        instance = module.ZEDCamera(camera_name="zed", base_frame_id="base")
        mocker.patch.object(instance.tf, "publish")
        try:
            yield instance
        finally:
            instance.stop()
            sys.modules.pop(name, None)


def sdk_pose(mocker, translation, rotation):
    value = mocker.Mock()
    value.get_translation.return_value.get.return_value = translation
    value.get_orientation.return_value.get.return_value = rotation
    return value


@pytest.mark.parametrize("ns", [0, -1, 1700000000123456789])
def test_frames_preserve_source_stamp_and_extrinsics(ns, camera, mocker):
    stamp = message_time.time_from_nanoseconds(ns)
    extrinsics = Transform(
        translation=Vector3(x=1, y=2, z=3),
        rotation=geometry.quaternion_from_euler(0, 0, math.pi / 2),
    )
    mount = Transform(
        translation=Vector3(x=4, y=5, z=6), rotation=geometry.quaternion_from_euler(0.2, 0, 0)
    )

    camera.config.base_transform = mount
    camera._camera_link_to_color_extrinsics = sdk_pose(
        mocker,
        geometry.vector_array(extrinsics.translation),
        geometry.quaternion_array(extrinsics.rotation),
    )
    camera._publish_tf(stamp)
    message = camera.tf.publish.call_args.args[0]
    actual = message

    assert [(edge.header.frame_id, edge.child_frame_id) for edge in actual.transforms] == [
        ("base", "zed_link"),
        ("zed_link", "zed_depth_frame"),
        ("zed_depth_frame", "zed_depth_optical_frame"),
        ("zed_link", "zed_color_frame"),
        ("zed_color_frame", "zed_color_optical_frame"),
    ]
    assert [message_time.to_nanoseconds(edge.header.stamp) for edge in actual.transforms] == [
        ns
    ] * 5
    np.testing.assert_allclose(
        geometry.transform_matrix(actual.transforms[0].transform), geometry.transform_matrix(mount)
    )
    for edge in [actual.transforms[1], actual.transforms[3]]:
        np.testing.assert_allclose(
            geometry.transform_matrix(edge.transform),
            np.linalg.inv(geometry.transform_matrix(extrinsics)),
            atol=1e-12,
        )
    for edge in [actual.transforms[2], actual.transforms[4]]:
        optical_to_frame = geometry.transform_matrix(edge.transform)
        # Optical right/down/forward maps to camera forward/left/up.
        np.testing.assert_allclose(optical_to_frame @ [1, 2, 3, 1], [3, -1, -2, 1], atol=1e-12)


def test_tracking_removes_rotated_mount_and_is_included_without_mount_edge(camera, mocker):
    stamp = message_time.time_from_nanoseconds(1700000000123456789)
    camera._pose = sdk_pose(mocker, [5, 6, 7], [0, 0, math.sqrt(0.5), math.sqrt(0.5)])
    camera._tracking_enabled = True
    camera._zed = mocker.Mock()
    camera._zed.get_position.return_value = 0
    world_camera = camera._extrinsics_to_transform(camera._pose, "world", "zed_link", stamp)
    mount = Transform(
        translation=Vector3(x=1, y=2, z=3),
        rotation=geometry.quaternion_from_euler(0, 0, -math.pi / 2),
    )

    camera.config.base_transform = mount
    tracking = camera._tracking_transform(stamp)
    camera.config.base_transform = None
    camera._camera_link_to_color_extrinsics = sdk_pose(
        mocker, geometry.vector_array(mount.translation), geometry.quaternion_array(mount.rotation)
    )
    assert camera._tracking_transform(stamp) == world_camera
    mocker.patch.object(camera, "_tracking_transform", return_value=tracking)
    camera._publish_tf(stamp)
    message = camera.tf.publish.call_args.args[0]
    actual = message

    assert tracking.header.frame_id == "world"
    assert tracking.child_frame_id == "base"
    assert message_time.to_nanoseconds(tracking.header.stamp) == 1700000000123456789
    np.testing.assert_allclose(
        geometry.transform_matrix(tracking.transform),
        geometry.transform_matrix(world_camera.transform)
        @ np.linalg.inv(geometry.transform_matrix(mount)),
        atol=1e-12,
    )
    assert len(actual.transforms) == 5
    assert actual.transforms[-1] == tracking
    assert actual.transforms[0].header.frame_id == "zed_link"
