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

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.usd import JOINT_ATTR, LINK_ATTR, graft_usd, prefix_usd_frames, urdf_to_usd

# usd-core is an optional extra with no linux-aarch64 wheel (see dimos.robot.assets.usd).
pytest.importorskip("pxr")

# A base with a fixed mount, a revolute joint about a non-axis-aligned axis, and a link name
# that is not a USD identifier.
_URDF = """<?xml version="1.0"?>
<robot name="toy">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.05"/>
      <geometry><box size="0.4 0.3 0.1"/></geometry>
      <material name="green"><color rgba="0.2 0.6 0.2 1"/></material>
    </visual>
    <collision><geometry><cylinder radius="0.2" length="0.1"/></geometry></collision>
  </link>
  <link name="mount"/>
  <link name="arm-1"><visual><geometry><sphere radius="0.05"/></geometry></visual></link>
  <link name="spin"/>
  <joint name="base_to_mount" type="fixed">
    <parent link="base_link"/><child link="mount"/>
    <origin xyz="0.1 0 0.1" rpy="0 0 0.3"/>
  </joint>
  <joint name="mount_to_arm" type="revolute">
    <parent link="mount"/><child link="arm-1"/>
    <origin xyz="0 0 0.02" rpy="0 0.2 0"/><axis xyz="0.6 0.8 0"/>
    <limit lower="-1.5707963267948966" upper="1.5707963267948966" effort="10" velocity="1"/>
  </joint>
  <joint name="arm_to_spin" type="continuous">
    <parent link="arm-1"/><child link="spin"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
"""


@pytest.fixture
def toy(tmp_path: Path) -> Any:
    urdf = tmp_path / "toy.urdf"
    urdf.write_text(_URDF)
    return RobotModel.from_file(urdf).load()


def _link_of(stage: Any, path: str) -> str:
    prim = stage.GetPrimAtPath(path)
    assert prim, f"no prim at {path}"
    return str(prim.GetAttribute(LINK_ATTR).Get())


def _stage(usd: str) -> Any:
    from pxr import Sdf, Usd

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd)
    return Usd.Stage.Open(layer)


def _prim_by_link(stage: Any, link: str) -> Any:
    for prim in stage.Traverse():
        attr = prim.GetAttribute(LINK_ATTR)
        if attr and attr.HasAuthoredValue() and str(attr.Get()) == link:
            return prim
    raise AssertionError(f"no prim for link {link!r}")


def test_the_default_prim_is_the_urdf_robot_name(toy: Any) -> None:
    stage = _stage(urdf_to_usd(toy))

    assert stage.GetDefaultPrim().GetName() == "toy"


def test_links_nest_down_the_kinematic_tree(toy: Any) -> None:
    stage = _stage(urdf_to_usd(toy))

    assert str(_prim_by_link(stage, "arm-1").GetPath()) == "/toy/base_link/mount/arm_1"


def test_a_link_name_that_is_not_a_usd_identifier_keeps_its_urdf_spelling(toy: Any) -> None:
    # "arm-1" cannot be a prim name, so the original is the only way back to the tf frame.
    stage = _stage(urdf_to_usd(toy))
    prim = _prim_by_link(stage, "arm-1")

    assert prim.GetName() == "arm_1"
    assert str(prim.GetAttribute(LINK_ATTR).Get()) == "arm-1"


def test_each_link_sits_at_its_urdf_zero_pose(toy: Any) -> None:
    from pxr import UsdGeom

    stage = _stage(urdf_to_usd(toy))
    joint = toy.get_joint("base_to_mount")
    assert joint is not None

    local = UsdGeom.Xformable(_prim_by_link(stage, "mount")).GetLocalTransformation()

    np.testing.assert_allclose(np.array(local).T, joint.parent_from_child(0.0), atol=1e-9)


def test_a_revolute_limit_is_radians_in_urdf_and_degrees_in_usd(toy: Any) -> None:
    stage = _stage(urdf_to_usd(toy))
    joint = stage.GetPrimAtPath("/toy/joints/mount_to_arm")

    assert joint.GetAttribute("physics:lowerLimit").Get() == pytest.approx(-90.0, abs=1e-3)
    assert joint.GetAttribute("physics:upperLimit").Get() == pytest.approx(90.0, abs=1e-3)


def test_a_continuous_joint_is_authored_without_limits(toy: Any) -> None:
    # No limit attributes at all is how USD says unlimited.
    stage = _stage(urdf_to_usd(toy))
    joint = stage.GetPrimAtPath("/toy/joints/arm_to_spin")

    assert joint.GetTypeName() == "PhysicsRevoluteJoint"
    assert not joint.GetAttribute("physics:lowerLimit").HasAuthoredValue()


def test_an_off_axis_joint_turns_about_the_urdf_axis(toy: Any) -> None:
    from scipy.spatial.transform import Rotation

    # physics:axis is only ever X/Y/Z, so the axis has to come out of the local rotations.
    stage = _stage(urdf_to_usd(toy))
    joint = stage.GetPrimAtPath("/toy/joints/mount_to_arm")
    assert str(joint.GetAttribute("physics:axis").Get()) == "Z"

    rot = joint.GetAttribute("physics:localRot0").Get()
    imaginary = rot.GetImaginary()
    rotation = Rotation.from_quat([imaginary[0], imaginary[1], imaginary[2], rot.GetReal()])
    description = toy.get_joint("mount_to_arm")
    assert description is not None
    expected = Rotation.from_euler("xyz", description.origin_rpy).apply(description.axis)

    np.testing.assert_allclose(rotation.apply([0.0, 0.0, 1.0]), expected, atol=1e-6)


def test_collision_geometry_is_carried_but_not_rendered(toy: Any) -> None:
    stage = _stage(urdf_to_usd(toy))
    collision = stage.GetPrimAtPath("/toy/base_link/collision_1")

    assert collision.GetTypeName() == "Cylinder"
    assert str(collision.GetAttribute("visibility").Get()) == "invisible"
    assert "PhysicsCollisionAPI" in collision.GetAppliedSchemas()


def test_collision_geometry_can_be_left_out(toy: Any) -> None:
    stage = _stage(urdf_to_usd(toy, include_collision=False))

    assert not stage.GetPrimAtPath("/toy/base_link/collision_1")


def test_a_box_is_a_unit_cube_scaled_to_its_extents(toy: Any) -> None:
    stage = _stage(urdf_to_usd(toy))
    cube = stage.GetPrimAtPath("/toy/base_link/visual_0")

    assert cube.GetAttribute("size").Get() == pytest.approx(1.0)
    np.testing.assert_allclose(cube.GetAttribute("xformOp:scale").Get(), (0.4, 0.3, 0.1), atol=1e-9)


def test_a_material_colour_becomes_the_display_colour(toy: Any) -> None:
    stage = _stage(urdf_to_usd(toy))
    colors = stage.GetPrimAtPath("/toy/base_link/visual_0").GetAttribute("primvars:displayColor")

    np.testing.assert_allclose(colors.Get()[0], (0.2, 0.6, 0.2), atol=1e-6)


def _mesh_urdf(mesh_path: Path) -> str:
    return f"""<?xml version="1.0"?>
<robot name="meshy">
  <link name="base_link">
    <visual><geometry><mesh filename="{mesh_path}" scale="2 2 2"/></geometry></visual>
  </link>
</robot>
"""


@pytest.fixture
def meshy(tmp_path: Path) -> Any:
    import trimesh

    mesh_path = tmp_path / "box.stl"
    trimesh.creation.box((0.1, 0.1, 0.1)).export(mesh_path)
    urdf = tmp_path / "meshy.urdf"
    urdf.write_text(_mesh_urdf(mesh_path))
    return RobotModel.from_file(urdf).load()


def test_inline_mesh_mode_bakes_the_triangles_into_the_layer(meshy: Any) -> None:
    # A scene leaves as one string, so an external mesh file would be unresolvable.
    stage = _stage(urdf_to_usd(meshy, mesh_mode="inline"))
    mesh = stage.GetPrimAtPath("/meshy/base_link/visual_0")

    assert len(mesh.GetAttribute("points").Get()) > 0
    assert set(mesh.GetAttribute("faceVertexCounts").Get()) == {3}


def test_reference_mesh_mode_keeps_the_path_the_urdf_named(meshy: Any) -> None:
    stage = _stage(urdf_to_usd(meshy, mesh_mode="reference"))
    mesh = stage.GetPrimAtPath("/meshy/base_link/visual_0")

    assert not mesh.GetAttribute("points").HasAuthoredValue()
    assert mesh.GetAttribute("dimos:meshFile").Get().path.endswith("box.stl")


def test_a_subtree_is_grafted_under_the_frame_it_names(toy: Any) -> None:
    root = urdf_to_usd(toy)

    stage = _stage(graft_usd(root, {"mount": urdf_to_usd(toy, name="arm_sub")}))

    assert stage.GetPrimAtPath("/toy/base_link/mount/arm_sub/base_link")


def test_a_grafted_subtree_keeps_its_joints_pointing_at_its_own_links(toy: Any) -> None:
    # Copied specs keep their authored paths, so the body rels have to be rebased.
    root = urdf_to_usd(toy)

    stage = _stage(graft_usd(root, {"mount": urdf_to_usd(toy, name="arm_sub")}))
    joint = stage.GetPrimAtPath("/toy/base_link/mount/arm_sub/joints/base_to_mount")

    assert joint
    targets = joint.GetRelationship("physics:body0").GetTargets()
    assert str(targets[0]) == "/toy/base_link/mount/arm_sub/base_link"


def test_grafting_at_a_frame_the_root_does_not_have_is_an_error(toy: Any) -> None:
    with pytest.raises(ValueError, match="no prim for that frame"):
        graft_usd(urdf_to_usd(toy), {"nope": urdf_to_usd(toy, name="arm_sub")})


def test_grafting_nothing_leaves_the_root_alone(toy: Any) -> None:
    root = urdf_to_usd(toy)

    assert graft_usd(root, {}) == root


def _attrs(stage: Any, name: str) -> set[str]:
    return {
        str(prim.GetAttribute(name).Get())
        for prim in stage.Traverse()
        if prim.GetAttribute(name) and prim.GetAttribute(name).HasAuthoredValue()
    }


def test_every_link_name_moves_under_the_prefix(toy: Any) -> None:
    stage = _stage(prefix_usd_frames(urdf_to_usd(toy), "left"))

    assert _attrs(stage, LINK_ATTR) == {
        "left/base_link",
        "left/mount",
        "left/arm-1",
        "left/spin",
    }


def test_joint_names_move_with_the_links(toy: Any) -> None:
    # A joint name is what a JointState keys on: two arms answering to one name is the same
    # collision the link names have, one layer down.
    stage = _stage(prefix_usd_frames(urdf_to_usd(toy), "left"))

    assert _attrs(stage, JOINT_ATTR) == {
        "left/base_to_mount",
        "left/mount_to_arm",
        "left/arm_to_spin",
    }


def test_the_original_urdf_spelling_survives_the_move(toy: Any) -> None:
    # "arm-1" is not a USD identifier, so the prim is "arm_1"; the attribute keeps the real name.
    stage = _stage(prefix_usd_frames(urdf_to_usd(toy), "left"))

    assert stage.GetPrimAtPath("/toy/base_link/mount/arm_1")
    assert _link_of(stage, "/toy/base_link/mount/arm_1") == "left/arm-1"


def test_a_nested_namespace_prefixes_whole(toy: Any) -> None:
    stage = _stage(prefix_usd_frames(urdf_to_usd(toy), "fleet/robot0"))

    assert "fleet/robot0/base_link" in _attrs(stage, LINK_ATTR)


def test_prim_paths_and_joint_bodies_are_untouched_by_the_move(toy: Any) -> None:
    # Prim names cannot hold a "/", and prim paths are already unique by nesting.
    prefixed = prefix_usd_frames(urdf_to_usd(toy), "left")

    stage = _stage(prefixed)
    joint = stage.GetPrimAtPath("/toy/joints/base_to_mount")

    assert joint
    targets = joint.GetRelationship("physics:body0").GetTargets()
    assert str(targets[0]) == "/toy/base_link"


def test_an_empty_prefix_leaves_the_layer_byte_identical(toy: Any) -> None:
    # An un-namespaced module must not even be re-exported: nothing about it is ambiguous.
    usd = urdf_to_usd(toy)

    assert prefix_usd_frames(usd, "") is usd


def test_a_prefixed_subtree_grafts_at_the_roots_own_unprefixed_frame(toy: Any) -> None:
    # The key is a link of the root, which is never renamed; only the subtree moves.
    root = urdf_to_usd(toy)
    subtree = prefix_usd_frames(urdf_to_usd(toy, name="arm_sub"), "left")

    stage = _stage(graft_usd(root, {"mount": subtree}))

    assert _link_of(stage, "/toy/base_link/mount") == "mount"
    assert _link_of(stage, "/toy/base_link/mount/arm_sub/base_link") == "left/base_link"
