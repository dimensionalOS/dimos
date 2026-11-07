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
    """The whole point: a URDF through USD and back describes the same kinematics.

    Covers the conversions that are easy to get backwards -- an axis folded into the local
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
        # 1e-6, not exact: UsdPhysics declares localPos as point3f and localRot as quatf, so
        # a joint's placement goes through single precision on the way out. The loss is
        # ~2e-8 on a metre, which is nothing next to the padding a self-filter uses.
        np.testing.assert_allclose(got.origin_xyz, original.origin_xyz, atol=1e-6)
        np.testing.assert_allclose(got.origin_rpy, original.origin_rpy, atol=1e-6)
        np.testing.assert_allclose(got.axis, original.axis, atol=1e-6)
        assert (got.lower, got.upper) == pytest.approx((original.lower, original.upper), abs=1e-6)


def test_the_kinematic_tree_comes_back_the_same(probe: Any, tmp_path: Path) -> None:
    back = _round_trip(probe, tmp_path)

    assert back.root_link == probe.root_link
    assert {j.child_link for j in back.joints} == {j.child_link for j in probe.joints}


def test_the_forward_kinematics_agree(probe: Any, tmp_path: Path) -> None:
    """Comparing the matrices, not just the fields they were written from."""
    back = _round_trip(probe, tmp_path)

    for original in probe.joints:
        got = back.get_joint(original.name)
        assert got is not None
        position = 0.0 if original.type in {"fixed"} else 0.3
        np.testing.assert_allclose(
            got.parent_from_child(position), original.parent_from_child(position), atol=1e-6
        )


def test_collision_geometry_comes_back_as_collision(probe: Any) -> None:
    """RoboPlan reads the collision half, so that is the half that has to survive."""
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
    """<inertial> is passed through to USD's customData, so it returns verbatim."""
    root = ET.fromstring(usd_to_urdf(urdf_to_usd(probe)))
    base = next(link for link in root.findall("link") if link.get("name") == "base")

    mass = base.find("inertial/mass")
    assert mass is not None
    assert float(mass.get("value") or 0) == pytest.approx(2.5)


def test_a_continuous_joint_round_trips_as_continuous(tmp_path: Path) -> None:
    """It is written as a revolute with no limits at all, so it has to be read back by that."""
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
    """The world root a manager stands on when it was given no model."""
    source = tmp_path / "world.urdf"
    source.write_text('<robot name="world"><link name="world"/></robot>')
    loaded = RobotModel.from_file(source).load()

    root = ET.fromstring(usd_to_urdf(urdf_to_usd(loaded)))

    assert [link.get("name") for link in root.findall("link")] == ["world"]
    assert root.findall("joint") == []
