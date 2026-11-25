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

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.robot.assets.joint_transforms import JointTransforms, joint_position
from dimos.robot.assets.model import JointDescription, RobotModel

_LINKAGE_URDF = """<robot name="linkage">
  <link name="base"/><link name="a"/><link name="b"/><link name="c"/><link name="d"/>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="a"/>
    <origin xyz="0.1 -0.2 0.3" rpy="0.4 -0.5 0.6"/><axis xyz="1 2 -3"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="slide" type="prismatic">
    <parent link="a"/><child link="b"/>
    <origin xyz="0 0.2 0" rpy="-0.3 0.2 1.1"/><axis xyz="0 1 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="follower" type="revolute">
    <parent link="b"/><child link="c"/>
    <origin xyz="0 0 0.1" rpy="3.0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
    <mimic joint="hinge" multiplier="-2" offset="0.25"/>
  </joint>
  <joint name="weld" type="fixed">
    <parent link="c"/><child link="d"/><origin xyz="0.3 0 0" rpy="0 1.2 0"/>
  </joint>
</robot>"""

_POSITIONS = ({}, {"hinge": 2.9, "slide": -0.7}, {"hinge": -1.3, "slide": 0.4})


def _joints() -> tuple[JointDescription, ...]:
    return RobotModel.from_urdf_string(_LINKAGE_URDF).load().joints


def _matrices(translations: np.ndarray, rotations: np.ndarray) -> np.ndarray:
    matrices = np.tile(np.eye(4), (len(translations), 1, 1))
    matrices[:, :3, :3] = Rotation.from_quat(rotations).as_matrix()
    matrices[:, :3, 3] = translations
    return matrices


def test_every_edge_is_where_its_joint_puts_it_at_any_position() -> None:
    joints = _joints()
    transforms = JointTransforms(joints)
    for positions in _POSITIONS:
        matrices = _matrices(*transforms.at(positions))
        for joint, matrix in zip(joints, matrices, strict=True):
            expected = joint.parent_from_child(joint_position(joint, positions))
            np.testing.assert_allclose(matrix, expected, atol=1e-9)


def test_the_quaternion_product_is_each_joints_own_rotation_with_w_not_negative() -> None:
    joints = _joints()
    for positions in _POSITIONS:
        _, rotations = JointTransforms(joints).at(positions)
        np.testing.assert_allclose(np.linalg.norm(rotations, axis=1), 1.0, atol=1e-12)
        assert (rotations[:, 3] >= 0.0).all()
        for joint, rotation in zip(joints, rotations, strict=True):
            expected = joint.parent_from_child(joint_position(joint, positions))[:3, :3]
            np.testing.assert_allclose(
                Rotation.from_quat(rotation).as_matrix(), expected, atol=1e-9
            )


def test_a_mimic_joint_follows_its_leader() -> None:
    follower = next(joint for joint in _joints() if joint.name == "follower")
    assert joint_position(follower, {"hinge": 0.5, "follower": 9.0}) == pytest.approx(-0.75)
    assert joint_position(follower, {}) == pytest.approx(0.25)


def test_no_joints_gives_no_transforms() -> None:
    translations, rotations = JointTransforms(()).at({})
    assert translations.shape == (0, 3)
    assert rotations.shape == (0, 4)


def test_a_joint_with_no_single_position_is_refused() -> None:
    with pytest.raises(ValueError, match="floating"):
        JointTransforms((JointDescription(name="free", type="floating"),))
