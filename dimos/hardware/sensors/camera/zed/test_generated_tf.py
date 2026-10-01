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

from enum import Enum
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

from dimos_generated.geometry_msgs.msg import Transform, Vector3
from dimos_generated.tf2_msgs.msg import TFMessage
import numpy as np
import pytest

from dimos.msgs.geometry import transform_matrix
from dimos.msgs.time import time_from_seconds


@pytest.fixture
def camera_module(monkeypatch):
    # Load the real camera class with only the optional SDK boundary mocked.
    # Neither Camera() nor start() is invoked.
    sl = ModuleType("pyzed.sl")
    sl.DEPTH_MODE = Enum("DEPTH_MODE", {"NEURAL": 1})
    sl.REFERENCE_FRAME = SimpleNamespace(WORLD=1)
    sl.POSITIONAL_TRACKING_STATE = SimpleNamespace(OK=1)
    package = ModuleType("pyzed")
    package.sl = sl
    monkeypatch.setitem(sys.modules, "pyzed", package)
    monkeypatch.setitem(sys.modules, "pyzed.sl", sl)
    spec = importlib.util.spec_from_file_location(
        "_zed_generated_tf_test", Path(__file__).with_name("camera.py")
    )
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def sdk_pose(translation):
    return SimpleNamespace(
        get_translation=lambda: SimpleNamespace(get=lambda: np.asarray(translation)),
        get_orientation=lambda: SimpleNamespace(get=lambda: np.asarray([0.0, 0.0, 0.0, 1.0])),
    )


@pytest.mark.parametrize("mount", [None, Transform(translation=Vector3(x=0.25))])
def test_tracking_uses_generated_frame_chain(camera_module, mount):
    camera = camera_module.ZEDCamera(base_transform=mount)
    camera._zed = Mock()
    camera._zed.get_position.return_value = 1
    camera._pose = sdk_pose([1.0, 2.0, 3.0])
    camera._tracking_enabled = True
    try:
        result = camera._tracking_transform(1700000000.125)
        assert result.header.frame_id == "world"
        assert result.header.stamp == time_from_seconds(1700000000.125)
        assert result.child_frame_id == ("camera_link" if mount is None else "base_link")
        assert result.transform.translation.x == (1.0 if mount is None else 0.75)
        assert result.transform.translation.y == 2.0
        camera._zed.get_position.return_value = 0
        assert camera._tracking_transform(2.0) is None
    finally:
        camera._zed = None
        camera.stop()


def test_generated_tf_preserves_mount_extrinsics_and_optical_frames(camera_module, monkeypatch):
    camera = camera_module.ZEDCamera()
    camera._camera_link_to_color_extrinsics = sdk_pose([0.1, 0.2, 0.3])
    received = []
    monkeypatch.setattr(camera.tf, "publish", received.append)
    try:
        camera._publish_tf(1700000000.125)
        message = TFMessage.decode(received[0].encode())
        assert len(message.transforms) == 5
        frames = [(t.header.frame_id, t.child_frame_id) for t in message.transforms]
        assert frames == [
            ("base_link", "camera_link"),
            ("camera_link", "camera_depth_frame"),
            ("camera_depth_frame", "camera_depth_optical_frame"),
            ("camera_link", "camera_color_frame"),
            ("camera_color_frame", "camera_color_optical_frame"),
        ]
        assert all(t.header.stamp == time_from_seconds(1700000000.125) for t in message.transforms)
        np.testing.assert_array_equal(transform_matrix(message.transforms[0].transform), np.eye(4))
        for index in (1, 3):
            np.testing.assert_allclose(
                transform_matrix(message.transforms[index].transform)[:3, 3], [-0.1, -0.2, -0.3]
            )
        for index in (2, 4):
            np.testing.assert_allclose(
                transform_matrix(message.transforms[index].transform)[:3, :3],
                [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
                atol=1e-15,
            )
    finally:
        camera.stop()
