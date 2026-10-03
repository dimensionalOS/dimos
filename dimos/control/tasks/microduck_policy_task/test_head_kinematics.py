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

import mujoco
import numpy as np
import pytest

from dimos.control.tasks.microduck_policy_task.head_kinematics import (
    HEAD_COMMAND_LOWER,
    HEAD_COMMAND_UPPER,
    HEAD_JOINT_NAMES,
    HeadKinematics,
)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion


@pytest.fixture
def head_model(tmp_path):
    path = tmp_path / "head.xml"
    path.write_text(
        """
<mujoco>
  <compiler angle="radian"/>
  <default><joint limited="true" range="-3 3"/><geom size=".01" mass=".1"/></default>
  <worldbody>
    <body name="trunk_base" pos=".1 .2 .3">
      <freejoint/>
      <geom/>
      <body name="neck" pos="0 0 .04">
        <joint name="neck_pitch" axis="0 1 0"/><geom/>
        <body name="pitch_link" pos="0 0 .05">
          <joint name="head_pitch" axis="0 1 0"/><geom/>
          <body name="yaw_link">
            <joint name="head_yaw" axis="0 0 1"/><geom/>
            <body name="head">
              <joint name="head_roll" axis="1 0 0"/><geom/>
              <site name="head_camera" pos=".03 0 0"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
""",
        encoding="utf-8",
    )
    return path


@pytest.mark.parametrize(
    "angles",
    [
        (0.0, 0.0, 0.0, 0.0),
        (0.349, 0.349, 0.0, 0.0),
        (0.2, -0.3, 0.6, 0.1),
        (-0.5, 0.8, -1.0, -0.2),
    ],
)
def test_pinocchio_camera_pose_matches_mujoco_in_trunk_frame(head_model, angles):
    model = mujoco.MjModel.from_xml_path(str(head_model))
    data = mujoco.MjData(model)
    for name, angle in zip(HEAD_JOINT_NAMES, angles, strict=True):
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = angle
    mujoco.mj_kinematics(model, data)
    trunk, camera = model.body("trunk_base").id, model.site("head_camera").id
    trunk_rotation = data.xmat[trunk].reshape(3, 3)
    optical_rotation = Quaternion(-0.5, 0.5, -0.5, 0.5).to_rotation_matrix()

    pose = HeadKinematics(head_model).camera_in_trunk_cv2(angles)

    np.testing.assert_allclose(
        pose.translation, trunk_rotation.T @ (data.site_xpos[camera] - data.xpos[trunk]), atol=1e-9
    )
    np.testing.assert_allclose(
        pose.rotation,
        trunk_rotation.T @ data.site_xmat[camera].reshape(3, 3) @ optical_rotation,
        atol=1e-9,
    )


def test_gaze_points_optical_axis_at_target_and_clamps_unreachable_target(head_model):
    head = HeadKinematics(head_model)
    target = np.asarray([1.0, 0.2, -0.1])

    reachable = head.look_at(tuple(target))
    unreachable = head.look_at((-1.0, 0.0, 0.0))

    assert reachable.clamped is False
    optical_target = head.camera_in_trunk_cv2(reachable.joints).inverse().act(target)
    assert optical_target[2] > 0
    np.testing.assert_allclose(optical_target[:2], [0.0, 0.0], atol=1e-4)
    assert unreachable.clamped is True
    assert np.all(np.asarray(unreachable.joints) >= HEAD_COMMAND_LOWER)
    assert np.all(np.asarray(unreachable.joints) <= HEAD_COMMAND_UPPER)


def test_gaze_follows_camera_mount_from_model(head_model):
    original = HeadKinematics(head_model).camera_in_trunk_cv2((0.0, 0.0, 0.0, 0.0))
    spec = mujoco.MjSpec.from_file(str(head_model))
    spec.site("head_camera").pos[0] += 0.02
    changed = Path(head_model).with_name("shifted.xml")
    changed.write_text(spec.to_xml(), encoding="utf-8")

    shifted = HeadKinematics(changed).camera_in_trunk_cv2((0.0, 0.0, 0.0, 0.0))

    np.testing.assert_allclose(
        shifted.translation - original.translation, [0.02, 0.0, 0.0], atol=1e-9
    )
