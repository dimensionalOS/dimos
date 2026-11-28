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
from typing import Any

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

pytest.importorskip("pxr")

from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.usd_utils.from_urdf import urdf_to_usd
from dimos.robot.assets.usd_utils.joints import usd_joints, usd_root_link

_URDF = """<?xml version="1.0"?>
<robot name="probe">
  <link name="base"/><link name="arm"/><link name="tool"/><link name="tip"/>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="arm"/>
    <origin xyz="0.01 0.02 0.03" rpy="0.1 0.2 0.3"/>
    <axis xyz="0.57735 0.57735 0.57735"/>
    <limit lower="-1.2" upper="0.8" effort="5" velocity="1"/>
  </joint>
  <joint name="wrist" type="prismatic">
    <parent link="arm"/><child link="tool"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-0.05" upper="0.05" effort="5" velocity="1"/>
  </joint>
  <joint name="spin" type="continuous">
    <parent link="tool"/><child link="tip"/>
    <origin xyz="0 0 0.03" rpy="0 0 0"/><axis xyz="0 0 1"/>
  </joint>
</robot>
"""

_PATHS = {"base": "probe/base", "arm": "probe/base/arm", "tool": "probe/base/arm/tool"}
_PATHS["tip"] = "probe/base/arm/tool/tip"


@pytest.fixture
def probe(tmp_path: Path) -> Any:
    source = tmp_path / "probe.urdf"
    source.write_text(_URDF)
    return RobotModel.from_file(source).load()


def test_every_urdf_joint_reads_back_off_its_usd(probe: Any) -> None:
    joints = usd_joints(urdf_to_usd(probe))

    assert [joint.name for joint in joints] == [joint.name for joint in probe.joints]
    for got, original in zip(joints, probe.joints, strict=True):
        assert (got.type, got.parent_link, got.child_link) == (
            original.type,
            _PATHS[original.parent_link],
            _PATHS[original.child_link],
        )
        for position in (0.0, 0.3):
            np.testing.assert_allclose(
                got.parent_from_child(position), original.parent_from_child(position), atol=1e-6
            )
        assert (got.lower, got.upper) == pytest.approx((original.lower, original.upper), abs=1e-6)
    assert usd_root_link(urdf_to_usd(probe)) == "probe/base"


def _rig(joint: str, axis: str = "X", pos1: str = "(0, 0, 0.2)") -> str:
    return f"""#usda 1.0
(
    defaultPrim = "rig"
)

def Xform "rig"
{{
    def Xform "base" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    )
    {{
        def Xform "mast" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI"]
        )
        {{
            double3 xformOp:translate = (0, 0, 0.5)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }}
    }}

    def Xform "arm" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    )
    {{
        def {joint} "elbow"
        {{
            uniform token physics:axis = "{axis}"
            rel physics:body0 = </rig/base>
            rel physics:body1 = </rig/arm>
            point3f physics:localPos0 = (0, 0, 1)
            point3f physics:localPos1 = {pos1}
            quatf physics:localRot0 = (1, 0, 0, 0)
            quatf physics:localRot1 = (0.70710678, 0, 0, 0.70710678)
        }}
    }}
}}
"""


def _arm_pose(joints: Any, angle: float) -> Any:
    """The arm in the base, chaining whatever joints lead to it."""
    by_child = {joint.child_link: joint for joint in joints}
    pose, link = np.eye(4), "rig/arm"
    while link in by_child:
        joint = by_child[link]
        pose = joint.parent_from_child(angle if joint.type != "fixed" else 0.0) @ pose
        link = joint.parent_link
    return pose


@pytest.mark.parametrize("joint", ["PhysicsFixedJoint", "PhysicsRevoluteJoint"])
def test_a_joint_anchored_off_the_child_origin_turns_about_the_anchor(joint: str) -> None:
    """base_from_joint @ motion @ inverse(arm_from_joint), straight from UsdPhysics."""
    joints = usd_joints(_rig(joint))

    for angle in (0.0, 0.7, -1.3) if joint == "PhysicsRevoluteJoint" else (0.0,):
        expected = np.eye(4)
        expected[:3, 3] = [0, 0, 1]
        motion = np.eye(4)
        motion[:3, :3] = Rotation.from_rotvec([angle, 0, 0]).as_matrix()
        arm_from_joint = np.eye(4)
        arm_from_joint[:3, :3] = Rotation.from_euler("z", np.pi / 2).as_matrix()
        arm_from_joint[:3, 3] = [0, 0, 0.2]
        np.testing.assert_allclose(
            _arm_pose(joints, angle), expected @ motion @ np.linalg.inv(arm_from_joint), atol=1e-6
        )


def test_a_joint_turns_about_the_axis_its_token_names() -> None:
    """A converter states the axis as a token; localRot1 turns it into the child's frame."""
    elbow = next(joint for joint in usd_joints(_rig("PhysicsRevoluteJoint", "Y", "(0, 0, 0)")))

    np.testing.assert_allclose(
        elbow.axis, Rotation.from_euler("z", np.pi / 2).apply([0, 1, 0]), atol=1e-6
    )


def test_a_nested_body_no_joint_holds_is_welded_where_it_sits() -> None:
    weld = next(
        joint
        for joint in usd_joints(_rig("PhysicsFixedJoint"))
        if joint.child_link == "rig/base/mast"
    )

    assert (weld.type, weld.parent_link) == ("fixed", "rig/base")
    np.testing.assert_allclose(weld.origin_xyz, (0, 0, 0.5), atol=1e-9)
    assert usd_root_link(_rig("PhysicsFixedJoint")) == "rig/base"
