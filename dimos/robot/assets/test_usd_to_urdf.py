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
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.usd import urdf_to_usd
from dimos.robot.assets.usd_to_urdf import usd_to_urdf

# usd-core is an optional extra, unavailable on aarch64 before Python 3.11 (see assets/usd.py).
pytest.importorskip("pxr")

# Every joint type, an axis on none of the three USD axes, and one of each geometry kind.
_URDF = """<?xml version="1.0"?>
<robot name="probe">
  <link name="base">
    <collision>
      <origin xyz="0 0 0.01" rpy="0 0 0.5"/>
      <geometry><box size="0.1 0.2 0.3"/></geometry>
    </collision>
    <inertial><mass value="2.5"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
  </link>
  <link name="arm">
    <collision><geometry><cylinder radius="0.05" length="0.4"/></geometry></collision>
    <visual><geometry><mesh filename="meshes/arm.stl"/></geometry></visual>
  </link>
  <link name="tool">
    <collision><geometry><sphere radius="0.03"/></geometry></collision>
  </link>
  <link name="tip"/>
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
  <joint name="stud" type="fixed">
    <parent link="tool"/><child link="tip"/>
    <origin xyz="0 0 0.03" rpy="0 0 0"/>
  </joint>
</robot>
"""


@pytest.fixture
def probe(tmp_path: Path) -> Any:
    meshes = tmp_path / "meshes"
    meshes.mkdir()
    (meshes / "arm.stl").write_text("solid arm\nendsolid arm\n")
    source = tmp_path / "probe.urdf"
    source.write_text(_URDF)
    return RobotModel.from_file(source).load()


def _round_trip(loaded: Any, tmp_path: Path) -> Any:
    """The description after URDF -> USD -> URDF, loaded again for comparison."""
    recovered = tmp_path / "recovered.urdf"
    recovered.write_text(usd_to_urdf(urdf_to_usd(loaded)))
    return RobotModel.from_file(recovered).load()


def test_every_joint_survives_the_round_trip_exactly(probe: Any, tmp_path: Path) -> None:
    """Covers the conversions that are easy to get backwards: an axis folded into the local
    rotations, and a revolute limit that is degrees in USD and radians in URDF.
    """
    back = _round_trip(probe, tmp_path)

    assert [joint.name for joint in back.joints] == [joint.name for joint in probe.joints]
    for original in probe.joints:
        got = back.get_joint(original.name)
        assert got is not None, original.name
        assert (got.type, got.parent_link, got.child_link) == (
            original.type,
            original.parent_link,
            original.child_link,
        )
        # 1e-6, not exact: UsdPhysics declares localPos as point3f and localRot as quatf, so a joint's
        # placement goes through single precision (~2e-8 on a metre).
        np.testing.assert_allclose(got.origin_xyz, original.origin_xyz, atol=1e-6)
        np.testing.assert_allclose(got.origin_rpy, original.origin_rpy, atol=1e-6)
        np.testing.assert_allclose(got.axis, original.axis, atol=1e-6)
        assert (got.lower, got.upper) == pytest.approx((original.lower, original.upper), abs=1e-6)


def test_the_kinematic_tree_comes_back_the_same(probe: Any, tmp_path: Path) -> None:
    back = _round_trip(probe, tmp_path)

    assert back.root_link == probe.root_link
    assert {j.child_link for j in back.joints} == {j.child_link for j in probe.joints}


def test_the_forward_kinematics_agree(probe: Any, tmp_path: Path) -> None:
    back = _round_trip(probe, tmp_path)

    for original in probe.joints:
        got = back.get_joint(original.name)
        assert got is not None
        position = 0.0 if original.type in {"fixed"} else 0.3
        np.testing.assert_allclose(
            got.parent_from_child(position), original.parent_from_child(position), atol=1e-6
        )


def test_collision_geometry_comes_back_as_collision(probe: Any) -> None:
    root = ET.fromstring(usd_to_urdf(urdf_to_usd(probe)))
    links = {link.get("name"): link for link in root.findall("link")}

    assert links["base"].find("collision/geometry/box") is not None
    assert links["arm"].find("collision/geometry/cylinder") is not None
    assert links["tool"].find("collision/geometry/sphere") is not None


def test_each_shape_keeps_its_dimensions(probe: Any) -> None:
    root = ET.fromstring(usd_to_urdf(urdf_to_usd(probe)))
    links = {link.get("name"): link for link in root.findall("link")}

    box = links["base"].find("collision/geometry/box")
    cylinder = links["arm"].find("collision/geometry/cylinder")
    sphere = links["tool"].find("collision/geometry/sphere")
    assert box is not None and cylinder is not None and sphere is not None
    np.testing.assert_allclose([float(v) for v in (box.get("size") or "").split()], [0.1, 0.2, 0.3])
    assert float(cylinder.get("radius") or 0) == pytest.approx(0.05)
    assert float(cylinder.get("length") or 0) == pytest.approx(0.4)
    assert float(sphere.get("radius") or 0) == pytest.approx(0.03)


def test_a_geometry_origin_survives(probe: Any) -> None:
    # The box is the only one placed off its link's frame.
    root = ET.fromstring(usd_to_urdf(urdf_to_usd(probe)))
    base = next(link for link in root.findall("link") if link.get("name") == "base")

    origin = base.find("collision/origin")
    assert origin is not None
    np.testing.assert_allclose(
        [float(v) for v in (origin.get("xyz") or "").split()], [0, 0, 0.01], atol=1e-9
    )
    np.testing.assert_allclose(
        [float(v) for v in (origin.get("rpy") or "").split()], [0, 0, 0.5], atol=1e-9
    )


def test_a_visual_mesh_keeps_the_path_the_urdf_named(probe: Any) -> None:
    root = ET.fromstring(usd_to_urdf(urdf_to_usd(probe)))
    arm = next(link for link in root.findall("link") if link.get("name") == "arm")

    mesh = arm.find("visual/geometry/mesh")
    assert mesh is not None
    assert (mesh.get("filename") or "").endswith("meshes/arm.stl")


def test_what_the_conversion_could_not_express_comes_back(probe: Any) -> None:
    root = ET.fromstring(usd_to_urdf(urdf_to_usd(probe)))
    base = next(link for link in root.findall("link") if link.get("name") == "base")

    mass = base.find("inertial/mass")
    assert mass is not None
    assert float(mass.get("value") or 0) == pytest.approx(2.5)


def test_a_continuous_joint_round_trips_as_continuous(tmp_path: Path) -> None:
    source = tmp_path / "spin.urdf"
    source.write_text(
        '<robot name="spin"><link name="a"/><link name="b"/>'
        '<joint name="spin" type="continuous"><parent link="a"/><child link="b"/>'
        '<axis xyz="0 0 1"/></joint></robot>'
    )
    loaded = RobotModel.from_file(source).load()

    back = _round_trip(loaded, tmp_path)

    joint = back.get_joint("spin")
    assert joint is not None
    assert joint.type == "continuous"
    assert joint.lower is None and joint.upper is None


def test_the_robot_name_can_be_overridden(probe: Any) -> None:
    assert ET.fromstring(usd_to_urdf(urdf_to_usd(probe), name="other")).get("name") == "other"


def test_a_layer_with_no_joints_is_still_a_robot(tmp_path: Path) -> None:
    source = tmp_path / "world.urdf"
    source.write_text('<robot name="world"><link name="world"/></robot>')
    loaded = RobotModel.from_file(source).load()

    root = ET.fromstring(usd_to_urdf(urdf_to_usd(loaded)))

    assert [link.get("name") for link in root.findall("link")] == ["world"]
    assert root.findall("joint") == []


def test_the_root_link_is_found_in_the_layer_not_taken_from_the_default_prim(
    probe: Any, tmp_path: Path
) -> None:
    from dimos.robot.assets.usd_to_urdf import usd_root_link

    usd = urdf_to_usd(probe)

    assert usd_root_link(usd) == probe.root_link == "base"
    assert "probe" not in usd_root_link(usd)


_NESTED_USDA = """#usda 1.0
(
    defaultPrim = "rig"
)

def Xform "rig"
{
    def Xform "chassis" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    )
    {
        def Xform "mast" (
            prepend apiSchemas = ["PhysicsRigidBodyAPI"]
        )
        {
            matrix4d xformOp:transform = ( (1,0,0,0), (0,1,0,0), (0,0,1,0), (0,0,0.5,1) )
            uniform token[] xformOpOrder = ["xformOp:transform"]

            def Cube "collision_0" (
                prepend apiSchemas = ["PhysicsCollisionAPI"]
            )
            {
                double size = 1
                float3 xformOp:scale = (0.1, 0.1, 0.1)
                uniform token[] xformOpOrder = ["xformOp:scale"]
            }
        }
    }
}
"""


_CAPSULE_USDA = """#usda 1.0
(
    defaultPrim = "leg"
)

def Xform "leg"
{
    def Xform "thigh" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    )
    {
        def Capsule "collision_0" (
            prepend apiSchemas = ["PhysicsCollisionAPI"]
        )
        {
            double height = 0.3
            double radius = 0.04
        }
    }
}
"""


def test_a_capsule_becomes_a_cylinder_that_covers_it() -> None:
    """URDF has no capsule, and dropping one leaves the link with no collision volume at all: the
    G1's thighs, shins and hands are capsules. A cylinder spanning the full length, caps
    included, is the usual stand-in.
    """
    root = ET.fromstring(usd_to_urdf(_CAPSULE_USDA))
    (link,) = [link for link in root.findall("link") if link.get("name") == "thigh"]

    cylinder = link.find("collision/geometry/cylinder")
    assert cylinder is not None, "the capsule was dropped"
    assert float(cylinder.get("radius") or 0) == pytest.approx(0.04)
    # height + both hemispherical caps, so the stand-in is never shorter than what it replaces.
    assert float(cylinder.get("length") or 0) == pytest.approx(0.38)


_MJC_GROUPED_USDA = """#usda 1.0
(
    defaultPrim = "leg"
)

def Xform "leg"
{
    def Xform "thigh" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    )
    {
        def Capsule "left_thigh"
        {
            double height = 0.2
            double radius = 0.06
            uniform int mjc:group = 3
        }

        def Mesh "thigh_visual"
        {
            uniform int mjc:group = 2
        }
    }
}
"""


def test_a_layer_that_marks_no_collision_falls_back_to_mujocos_grouping() -> None:
    """The G1's default class is `contype="0" conaffinity="0"`, so the converted layer carries no
    CollisionAPI anywhere; such models still put their collision geoms in MuJoCo's group 3.
    """
    root = ET.fromstring(usd_to_urdf(_MJC_GROUPED_USDA))
    (link,) = [link for link in root.findall("link") if link.get("name") == "thigh"]

    assert link.find("collision/geometry/cylinder") is not None, "group 3 was not read as collision"
    assert link.find("visual/geometry/cylinder") is None


_MARKED_AND_GROUPED_USDA = """#usda 1.0
(
    defaultPrim = "leg"
)

def Xform "leg"
{
    def Xform "thigh" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    )
    {
        def Cube "hard" (
            prepend apiSchemas = ["PhysicsCollisionAPI"]
        )
        {
            double size = 1
        }

        def Capsule "soft"
        {
            double height = 0.2
            double radius = 0.06
            uniform int mjc:group = 3
        }
    }
}
"""


def test_a_layer_that_marks_collision_is_not_second_guessed() -> None:
    """The group number is per-model -- the M20's vendor MJCF collides on group 1, not 3 -- so the
    fallback must stay a fallback.
    """
    root = ET.fromstring(usd_to_urdf(_MARKED_AND_GROUPED_USDA))
    (link,) = [link for link in root.findall("link") if link.get("name") == "thigh"]

    assert link.find("collision/geometry/box") is not None
    assert link.find("visual/geometry/cylinder") is not None, (
        "the group was read despite CollisionAPI"
    )
    assert link.find("collision/geometry/cylinder") is None


_SCENE_USDA = """#usda 1.0
(
    defaultPrim = "room"
)

def Xform "room"
{
    def Xform "desk" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    )
    {
        def PhysicsFixedJoint "desk_joint"
        {
            rel physics:body0 = </room>
            rel physics:body1 = </room/desk>
        }
    }

    def Xform "chair" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    )
    {
        def PhysicsFixedJoint "chair_joint"
        {
            rel physics:body0 = </room>
            rel physics:body1 = </room/chair>
        }
    }
}
"""


def test_a_scene_is_rooted_at_the_prim_every_joint_hangs_off() -> None:
    """An MJCF worldbody is not a rigid body, so every item is some weld's child and "the link no
    joint claims" comes back empty: the fallback rooted office_555's 448 pieces of furniture at a
    cardboard box. The prim all the welds name is the root, link or not.
    """
    from dimos.robot.assets.usd_to_urdf import usd_root_link

    assert usd_root_link(_SCENE_USDA) == "room"


# The chair hangs off a second, real prim, so the layer has two candidate roots.
_TWO_PARENTS_USDA = _SCENE_USDA.replace(
    "rel physics:body0 = </room>\n            rel physics:body1 = </room/chair>",
    "rel physics:body0 = </room/alcove>\n            rel physics:body1 = </room/chair>",
).replace(
    '    def Xform "chair" (',
    '    def Xform "alcove"\n    {\n    }\n\n    def Xform "chair" (',
)


def test_two_candidate_roots_are_not_guessed_between() -> None:
    """Two different parents means the layer is not a flat scene, and picking either would be a
    guess dressed up as a root.
    """
    from dimos.robot.assets.usd_to_urdf import usd_root_link

    assert usd_root_link(_TWO_PARENTS_USDA) in {"desk", "chair"}


def test_a_nested_link_with_no_joint_becomes_a_fixed_joint() -> None:
    """USD conveys the tree by nesting and a URDF with two unconnected links will not load, so a
    hand-written layer that states only the hierarchy needs the weld inferred.
    """
    root = ET.fromstring(usd_to_urdf(_NESTED_USDA))

    assert {link.get("name") for link in root.findall("link")} == {"chassis", "mast"}
    joints = root.findall("joint")
    assert len(joints) == 1
    assert joints[0].get("type") == "fixed"
    parent = joints[0].find("parent")
    child = joints[0].find("child")
    assert parent is not None and child is not None
    assert (parent.get("link"), child.get("link")) == ("chassis", "mast")


def test_an_inferred_weld_keeps_the_nested_prim_s_pose() -> None:
    root = ET.fromstring(usd_to_urdf(_NESTED_USDA))

    origin = root.findall("joint")[0].find("origin")
    assert origin is not None
    np.testing.assert_allclose(
        [float(v) for v in (origin.get("xyz") or "").split()], [0, 0, 0.5], atol=1e-9
    )


def test_a_nested_layer_names_its_root_link() -> None:
    from dimos.robot.assets.usd_to_urdf import usd_root_link

    assert usd_root_link(_NESTED_USDA) == "chassis"


_FLOATING_URDF = """<?xml version="1.0"?>
<robot name="walker">
  <link name="world"/>
  <link name="pelvis">
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <link name="thigh"/>
  <joint name="floating_base" type="floating">
    <parent link="world"/><child link="pelvis"/>
    <origin xyz="0 0 0.8" rpy="0 0 0"/>
  </joint>
  <joint name="hip" type="revolute">
    <parent link="pelvis"/><child link="thigh"/>
    <axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""


@pytest.fixture
def walker(tmp_path: Path) -> Any:
    source = tmp_path / "walker.urdf"
    source.write_text(_FLOATING_URDF)
    return RobotModel.from_file(source).load()


def test_a_floating_base_does_not_come_back_welded(walker: Any, tmp_path: Path) -> None:
    """A floating joint is written as no joint prim, which is indistinguishable from a nested,
    welded body, so the joint left out is recorded on the child. Without that a walking robot's
    base comes back fixed and anything posing it from odometry is ignored.
    """
    back = _round_trip(walker, tmp_path)

    joint = back.get_joint("floating_base")
    assert joint is not None, [j.name for j in back.joints]
    assert joint.type == "floating"
    assert (joint.parent_link, joint.child_link) == ("world", "pelvis")
    np.testing.assert_allclose(joint.origin_xyz, [0, 0, 0.8], atol=1e-6)


def test_a_planar_joint_round_trips_too(tmp_path: Path) -> None:
    source = tmp_path / "slider.urdf"
    source.write_text(
        '<robot name="slider"><link name="world"/><link name="body"/>'
        '<joint name="planar_base" type="planar"><parent link="world"/>'
        '<child link="body"/><origin xyz="0 0 0.1"/></joint></robot>'
    )
    loaded = RobotModel.from_file(source).load()

    back = _round_trip(loaded, tmp_path)

    joint = back.get_joint("planar_base")
    assert joint is not None
    assert joint.type == "planar"


def test_a_nested_link_with_no_record_is_still_read_as_welded() -> None:
    root = ET.fromstring(usd_to_urdf(_NESTED_USDA))

    assert [joint.get("type") for joint in root.findall("joint")] == ["fixed"]
