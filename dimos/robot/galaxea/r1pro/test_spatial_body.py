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

"""Body targets cross the measured torso frame before reaching bimanual IK."""

from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.spatial import R1ProSpatialTeleop
from dimos.teleop.spatial.model import SpatialGroup, SpatialRobot
from dimos.teleop.webxr.controller_types import Hand


@pytest.fixture
def body_viewer(mocker):
    module = R1ProSpatialTeleop(
        absolute_body_targets=True,
        ik_base_frame="upper_base",
        robot_model=RobotModelConfig(
            model=RobotModel.from_file(Path("unused.urdf")), joint_names=[], base_link="base"
        ),
        groups={
            "left": SpatialGroup(
                label="Left",
                joint_names=(),
                tip_frame="tip",
                base_frame="shoulder",
                body_anchor_frame="anchor",
                body_shoulder_frame="shoulder",
                body_forward_orientation=tuple(Rotation.from_euler("y", -np.pi / 2).as_quat()),
            )
        },
    )
    module._robot = mocker.Mock(spec=SpatialRobot)
    module._robot.geometry = {"body_reference": {"left": {"shoulder": [0, 0.2, 0.4], "reach": 0.7}}}
    published = mocker.patch.object(module.left_controller_output, "publish")
    try:
        yield module, published
    finally:
        module.stop()


def test_absolute_target_uses_measured_height_frame_and_reach_cap(body_viewer, mocker):
    module, published = body_viewer
    anchor = np.eye(4)
    anchor[:3, 3] = [0, 0, 0.8]
    upper_from_base = np.eye(4)
    upper_from_base[:3, :3] = Rotation.from_euler("y", 0.4).as_matrix()
    upper_from_base[:3, 3] = [0.1, 0, -0.8]
    transforms = {("base", "anchor"): anchor, ("upper_base", "base"): upper_from_base}
    mocker.patch.object(
        module._transforms,
        "get",
        side_effect=lambda parent, child, *args, **kwargs: Transform.from_matrix(
            transforms[parent, child]
        ),
    )
    module._publish_msg(
        Hand.LEFT, PoseStamped(position=[1, 0, 0], orientation=[0, 0, 0, 1], frame_id="body")
    )
    target = published.call_args.args[0]
    expected_root = np.array([0.7 * 0.98, 0.2, 1.2])
    np.testing.assert_allclose(
        target.position.to_numpy(), upper_from_base[:3, :3] @ expected_root + upper_from_base[:3, 3]
    )
    assert target.frame_id == "upper_base"
    np.testing.assert_allclose(
        Rotation.from_quat(target.orientation.to_list()).as_matrix(),
        upper_from_base[:3, :3] @ Rotation.from_euler("y", -np.pi / 2).as_matrix(),
        atol=1e-12,
    )
    # There is no grip-time reference, so repeating the pose cannot add displacement.
    module._publish_msg(
        Hand.LEFT, PoseStamped(position=[1, 0, 0], orientation=[0, 0, 0, 1], frame_id="body")
    )
    np.testing.assert_allclose(
        published.call_args.args[0].position.to_numpy(), target.position.to_numpy()
    )


def test_missing_height_transform_does_not_emit_a_target(body_viewer, mocker):
    module, published = body_viewer
    mocker.patch.object(module._transforms, "get", return_value=None)
    module._publish_msg(Hand.LEFT, PoseStamped(position=[1, 0, 0], frame_id="body"))
    published.assert_not_called()
