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

import numpy as np
import pytest

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.assets.model import RobotModel
from dimos.teleop.spatial.module import SpatialTeleopModule


@pytest.fixture
def viewer():
    module = SpatialTeleopModule(
        robot_model=RobotModelConfig(
            model=RobotModel.from_file(Path("/unused.urdf")), joint_names=[]
        ),
        cameras={"workspace": "Head", "left": "Left", "right": "Right"},
        camera_depth_ranges={"left": (0.07, 0.5), "right": (0.07, 0.5)},
    )
    try:
        yield module
    finally:
        module.stop()


def pair(frame, z, ts=100):
    return (
        Image(np.zeros((2, 2, 3), dtype=np.uint8), ImageFormat.RGB, frame, ts),
        Image(np.full((2, 2), z, dtype=np.float32), ImageFormat.DEPTH, frame, ts),
    )


def test_each_camera_uses_its_own_calibration_pose_and_empty_observation(viewer):
    for slot, focal, x in (("workspace", 100, 0), ("left", 200, 1), ("right", 300, 2)):
        viewer._on_camera_info(
            slot, CameraInfo.from_intrinsics(focal, focal, 0, 0, 2, 2, frame_id=slot)
        )
        viewer._transforms.receive_transform(
            Transform(translation=Vector3(x, 0, 0), frame_id="world", child_frame_id=slot, ts=100)
        )
        viewer._on_rgbd(slot, pair(slot, 0.3))
        np.testing.assert_allclose(viewer._surfaces[slot].points[1], [x + 0.3 / focal, 0, 0.3])
    original = viewer._surfaces["workspace"]
    viewer._on_rgbd("left", pair("right", 0.3))
    assert "registered optical" in viewer._observation_errors["left"]
    viewer._on_rgbd("right", pair("right", 1, 100.01))
    assert len(viewer._surfaces["right"].points) == 0
    assert len(viewer._surfaces["left"].points) == 4
    assert viewer._surfaces["workspace"] is original


def test_remote_workspace_uses_configured_https_listener(viewer):
    viewer.config.server_port = 9443
    view = viewer.get_web_views()[0]
    assert view["network"] == {"scheme": "https", "port": 9443}
    assert view["path"].startswith("/workspace?operator_session=")
