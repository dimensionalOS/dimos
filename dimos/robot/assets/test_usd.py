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
from dimos.robot.assets.urdf_to_usd import PASSTHROUGH_KEY, urdf_to_usd
from dimos.robot.assets.usd import (
    graft_usd,
    prefix_usd_frames,
    rename_usd_frames,
    usd_drives,
    usd_frame_names,
    usd_with_drives,
    usd_with_frames,
)
from dimos.robot.assets.usd_assets import asset_hash, require_asset_hashes, unhashed_assets

pytest.importorskip("pxr")

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


def _stage(usd: str) -> Any:
    from pxr import Sdf, Usd

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd)
    return Usd.Stage.Open(layer)


def _prim_by_link(stage: Any, link: str) -> Any:
    from pxr import UsdPhysics

    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        if (prim.GetDisplayName() or prim.GetName()) == link:
            return prim
    raise AssertionError(f"no prim for link {link!r}")


def test_the_default_prim_is_the_urdf_robot_name(toy: Any) -> None:
    stage = _stage(urdf_to_usd(toy))

    assert stage.GetDefaultPrim().GetName() == "toy"


def test_links_nest_down_the_kinematic_tree(toy: Any) -> None:
    stage = _stage(urdf_to_usd(toy))

    assert str(_prim_by_link(stage, "arm-1").GetPath()) == "/toy/base_link/mount/arm_1"


def test_a_link_name_that_is_not_a_usd_identifier_keeps_its_urdf_spelling(toy: Any) -> None:
    stage = _stage(urdf_to_usd(toy))
    prim = _prim_by_link(stage, "arm-1")

    assert prim.GetName() == "arm_1"
    assert prim.GetDisplayName() == "arm-1"


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
    stage = _stage(urdf_to_usd(toy))
    joint = stage.GetPrimAtPath("/toy/joints/arm_to_spin")

    assert joint.GetTypeName() == "PhysicsRevoluteJoint"
    assert not joint.GetAttribute("physics:lowerLimit").HasAuthoredValue()


def test_an_off_axis_joint_turns_about_the_urdf_axis(toy: Any) -> None:
    from scipy.spatial.transform import Rotation

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


def _mesh_urdf(mesh_path: str) -> str:
    return f"""<?xml version="1.0"?>
<robot name="meshy">
  <link name="base_link">
    <visual><geometry><mesh filename="{mesh_path}" scale="2 2 2"/></geometry></visual>
  </link>
</robot>
"""


@pytest.fixture
def meshy(tmp_path: Path) -> Any:
    urdf = tmp_path / "meshy.urdf"
    urdf.write_text(_mesh_urdf("meshes/box.stl"))
    return RobotModel.from_file(urdf).load()


def _references(prim: Any) -> list[str]:
    from pxr import Usd

    items: list[str] = []
    for spec in prim.GetPrimStack():
        for ref in spec.referenceList.GetAddedOrExplicitItems():
            items.append(ref.assetPath)
    assert Usd
    return items


def test_a_mesh_names_the_path_the_urdf_named(meshy: Any) -> None:
    stage = _stage(urdf_to_usd(meshy))
    mesh = stage.GetPrimAtPath("/meshy/base_link/visual_0")

    assert str(mesh.GetAssetInfo()["identifier"].path) == "meshes/box.stl"


def test_a_mesh_file_is_not_referenced_as_though_it_were_a_usd_layer(meshy: Any) -> None:
    stage = _stage(urdf_to_usd(meshy))

    offenders = {
        str(prim.GetPath()): path
        for prim in stage.Traverse()
        for path in _references(prim)
        if not path.endswith((".usd", ".usda", ".usdc", ".usdz"))
    }

    assert offenders == {}


def test_a_mesh_carries_no_geometry_of_its_own(meshy: Any) -> None:
    stage = _stage(urdf_to_usd(meshy))
    mesh = stage.GetPrimAtPath("/meshy/base_link/visual_0")

    assert not mesh.GetAttribute("points").HasAuthoredValue()


def test_the_layer_authors_no_custom_attributes(toy: Any) -> None:
    usd = urdf_to_usd(toy)

    assert "custom " not in usd
    assert "dimos:" not in usd


def test_a_subtree_is_grafted_under_the_frame_it_names(toy: Any) -> None:
    root = urdf_to_usd(toy)

    stage = _stage(graft_usd(root, {"mount": urdf_to_usd(toy, name="arm_sub")}))

    assert stage.GetPrimAtPath("/toy/base_link/mount/arm_sub/base_link")


def test_a_grafted_subtree_keeps_its_joints_pointing_at_its_own_links(toy: Any) -> None:
    root = urdf_to_usd(toy)

    stage = _stage(graft_usd(root, {"mount": urdf_to_usd(toy, name="arm_sub")}))
    joint = stage.GetPrimAtPath("/toy/base_link/mount/arm_sub/joints/base_to_mount")

    assert joint
    targets = joint.GetRelationship("physics:body0").GetTargets()
    assert str(targets[0]) == "/toy/base_link/mount/arm_sub/base_link"


def test_a_graft_joins_the_subtree_to_its_mount_with_a_floating_joint(toy: Any) -> None:
    root = urdf_to_usd(toy)

    stage = _stage(graft_usd(root, {"mount": urdf_to_usd(toy, name="arm_sub")}))
    mounted = [prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsJoint"]

    assert len(mounted) == 1
    joint = mounted[0]
    assert joint.GetDisplayName() == "base_link_mount"
    assert str(joint.GetRelationship("physics:body0").GetTargets()[0]) == "/toy/base_link/mount"
    assert str(joint.GetRelationship("physics:body1").GetTargets()[0]) == (
        "/toy/base_link/mount/arm_sub/base_link"
    )


def test_the_mount_joint_adds_no_offset_of_its_own(toy: Any) -> None:
    from pxr import Gf

    stage = _stage(graft_usd(urdf_to_usd(toy), {"mount": urdf_to_usd(toy, name="arm_sub")}))
    joint = next(prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsJoint")

    for attribute in ("physics:localPos0", "physics:localPos1"):
        assert joint.GetAttribute(attribute).Get() == Gf.Vec3f(0.0, 0.0, 0.0)
    for attribute in ("physics:localRot0", "physics:localRot1"):
        assert joint.GetAttribute(attribute).Get() == Gf.Quatf(1.0, 0.0, 0.0, 0.0)


def test_a_frame_can_be_added_to_a_scene_and_mounted_at(toy: Any) -> None:
    root = urdf_to_usd(toy)

    grown = usd_with_frames(root, {"spot": ("base_link", (1.0, 2.0, 0.0), (0.0, 0.0, 0.0))})
    stage = _stage(graft_usd(grown, {"spot": urdf_to_usd(toy, name="arm_sub")}))

    assert _prim_by_link(stage, "spot"), "the frame is a link, so graft_usd can match it"
    assert stage.GetPrimAtPath("/toy/base_link/spot/arm_sub/base_link")


def test_a_frame_added_to_a_link_is_welded_to_it(toy: Any) -> None:
    from pxr import UsdPhysics

    grown = usd_with_frames(
        urdf_to_usd(toy), {"spot": ("base_link", (1.0, 2.0, 0.0), (0.0, 0.0, 0.0))}
    )
    stage = _stage(grown)

    frame = _prim_by_link(stage, "spot")
    assert frame, "the frame is still a link"
    welds = [
        prim
        for prim in stage.Traverse()
        if prim.IsA(UsdPhysics.FixedJoint)
        and prim.GetRelationship("physics:body1").GetTargets() == [frame.GetPath()]
    ]
    assert len(welds) == 1, "the frame is held by exactly one weld"
    base = _prim_by_link(stage, "base_link")
    assert welds[0].GetRelationship("physics:body0").GetTargets() == [base.GetPath()]


def test_a_frame_added_to_something_that_is_not_a_link_is_welded_to_the_world(toy: Any) -> None:
    from pxr import UsdPhysics

    stage = _stage(
        usd_with_frames(urdf_to_usd(toy), {"spot": ("toy", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))})
    )

    frame = _prim_by_link(stage, "spot")
    assert frame, "the frame is a link"
    welds = [
        prim
        for prim in stage.Traverse()
        if prim.IsA(UsdPhysics.FixedJoint)
        and prim.GetRelationship("physics:body1").GetTargets() == [frame.GetPath()]
    ]
    assert len(welds) == 1, "the frame is held by exactly one weld"
    assert welds[0].GetRelationship("physics:body0").GetTargets() == []


def test_a_driven_joint_says_its_gains_in_both_spellings(toy: Any) -> None:
    from pxr import UsdPhysics

    grown = usd_with_drives(urdf_to_usd(toy), {"base_to_mount": (40.0, 2.0, 0.25)})
    stage = _stage(grown)

    joint = next(
        prim
        for prim in stage.Traverse()
        if prim.IsA(UsdPhysics.Joint) and prim.GetName() == "base_to_mount"
    )
    drive = UsdPhysics.DriveAPI(joint, "angular")
    assert drive.GetStiffnessAttr().Get() == pytest.approx(40.0)
    assert drive.GetDampingAttr().Get() == pytest.approx(2.0)
    assert drive.GetTargetPositionAttr().Get() == pytest.approx(0.25)


def _with_actuator(usd: str, joint: str, *, biased: bool) -> str:
    from pxr import Sdf, Usd, UsdPhysics

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd)
    stage = Usd.Stage.Open(layer)
    prim = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint) and p.GetName() == joint)
    actuator = stage.DefinePrim(f"/toy/Physics/{joint}", "MjcActuator")
    actuator.CreateRelationship("mjc:target").SetTargets([prim.GetPath()])
    double = Sdf.ValueTypeNames.Double
    actuator.CreateAttribute("mjc:ctrlRange:min", double).Set(-23.7)
    actuator.CreateAttribute("mjc:ctrlRange:max", double).Set(23.7)
    if biased:
        actuator.CreateAttribute("mjc:biasType", Sdf.ValueTypeNames.Token).Set("affine")
    return str(layer.ExportToString())


def test_a_motors_control_range_becomes_its_torque_limit(toy: Any) -> None:
    usd = _with_actuator(urdf_to_usd(toy), "base_to_mount", biased=False)

    stage = _stage(usd_with_drives(usd, {"base_to_mount": (40.0, 2.0, 0.0)}))
    driven = stage.GetPrimAtPath("/toy/Physics/base_to_mount")

    assert str(driven.GetAttribute("mjc:biasType").Get()) == "affine"
    assert driven.GetAttribute("mjc:forceRange:max").Get() == pytest.approx(23.7)
    assert str(driven.GetAttribute("mjc:ctrlLimited").Get()) == "false"


def test_an_actuator_that_already_biases_keeps_its_control_range_alone(toy: Any) -> None:
    usd = _with_actuator(urdf_to_usd(toy), "base_to_mount", biased=True)

    stage = _stage(usd_with_drives(usd, {"base_to_mount": (40.0, 2.0, 0.0)}))
    driven = stage.GetPrimAtPath("/toy/Physics/base_to_mount")

    assert driven.GetAttribute("mjc:gainPrm").Get()[0] == pytest.approx(40.0)
    assert not driven.GetAttribute("mjc:forceRange:max").HasAuthoredValue()


def test_a_joint_the_layer_does_not_have_cannot_be_driven(toy: Any) -> None:
    with pytest.raises(ValueError, match="no joint for \\['nowhere'\\]"):
        usd_with_drives(urdf_to_usd(toy), {"nowhere": (1.0, 0.1, 0.0)})


def test_the_drives_read_back_as_they_were_written(toy: Any) -> None:
    grown = usd_with_drives(
        urdf_to_usd(toy), {"base_to_mount": (40.0, 2.0, 0.25), "mount_to_arm": (8.0, 0.4, -1.5)}
    )

    assert usd_drives(grown) == {
        "base_to_mount": pytest.approx((40.0, 2.0, 0.25)),
        "mount_to_arm": pytest.approx((8.0, 0.4, -1.5)),
    }
    assert "nowhere" not in usd_drives(urdf_to_usd(toy))


def test_a_frame_can_hang_off_a_frame_added_in_the_same_call(toy: Any) -> None:
    stage = _stage(
        usd_with_frames(
            urdf_to_usd(toy),
            {
                "front_camera": ("base_link", (0.1, 0.0, 0.2), (0.0, 0.0, 0.0)),
                "lidar": ("front_camera", (0.0, 0.0, 0.05), (0.0, 0.0, 0.0)),
            },
        )
    )

    camera = _prim_by_link(stage, "front_camera")
    assert camera
    assert stage.GetPrimAtPath(camera.GetPath().AppendChild("lidar")), "the lidar is on the camera"


def test_a_frame_added_where_one_already_lives_is_an_error(toy: Any) -> None:
    with pytest.raises(ValueError, match="is taken"):
        usd_with_frames(
            urdf_to_usd(toy), {"mount": ("base_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))}
        )


def test_a_frame_hung_off_a_frame_the_scene_does_not_have_is_an_error(toy: Any) -> None:
    with pytest.raises(ValueError, match="no frame 'nowhere'"):
        usd_with_frames(urdf_to_usd(toy), {"spot": ("nowhere", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))})


def test_grafting_at_a_frame_the_root_does_not_have_is_an_error(toy: Any) -> None:
    with pytest.raises(ValueError, match="no prim for that frame"):
        graft_usd(urdf_to_usd(toy), {"nope": urdf_to_usd(toy, name="arm_sub")})


def test_a_subtree_can_be_grafted_at_the_roots_own_name(toy: Any) -> None:
    root = urdf_to_usd(toy)

    stage = _stage(graft_usd(root, {"toy": urdf_to_usd(toy, name="arm_sub")}))

    assert stage.GetPrimAtPath("/toy/arm_sub/base_link")


def test_grafting_two_copies_of_one_description_at_one_frame_is_an_error(toy: Any) -> None:
    root = graft_usd(urdf_to_usd(toy), {"toy": urdf_to_usd(toy, name="arm_sub")})

    with pytest.raises(ValueError, match="is taken"):
        graft_usd(root, {"toy": urdf_to_usd(toy, name="arm_sub")})


def test_a_link_still_wins_a_name_the_root_shares(toy: Any) -> None:
    root = urdf_to_usd(toy, name="base_link")

    stage = _stage(graft_usd(root, {"base_link": urdf_to_usd(toy, name="arm_sub")}))

    assert stage.GetPrimAtPath("/base_link/base_link/arm_sub/base_link"), (
        "the subtree went to the root rather than to the link of that name"
    )


def test_a_prefix_moves_the_root_with_everything_under_it(toy: Any) -> None:
    scene = urdf_to_usd(toy, name="room")

    left = usd_frame_names(prefix_usd_frames(scene, "left"))
    right = usd_frame_names(prefix_usd_frames(scene, "right"))

    assert left & right == set(), f"both copies still answer to {sorted(left & right)}"
    assert "left/room" in left, sorted(left)


def test_grafting_nothing_leaves_the_root_alone(toy: Any) -> None:
    root = urdf_to_usd(toy)

    assert graft_usd(root, {}) == root


_EXTRAS_URDF = """<?xml version="1.0"?>
<robot name="extra">
  <link name="base_link">
    <inertial><mass value="0.718"/></inertial>
    <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
    <vendor:thing xmlns:vendor="http://example.com" note="keep me"/>
  </link>
  <link name="arm"/>
  <joint name="shoulder" type="fixed">
    <parent link="base_link"/><child link="arm"/>
    <origin xyz="0 0 0.1"/>
    <dynamics damping="0.7"/>
  </joint>
  <gazebo reference="base_link"><mu1>0.2</mu1></gazebo>
</robot>
"""


@pytest.fixture
def extra(tmp_path: Path) -> Any:
    urdf = tmp_path / "extra.urdf"
    urdf.write_text(_EXTRAS_URDF)
    return RobotModel.from_file(urdf).load()


def _passthrough(stage: Any, path: str) -> list[str]:
    return list(stage.GetPrimAtPath(path).GetCustomDataByKey(PASSTHROUGH_KEY) or [])


def test_a_links_unconverted_urdf_rides_along_on_its_prim(extra: Any) -> None:
    stage = _stage(urdf_to_usd(extra))

    kept = _passthrough(stage, "/extra/base_link")

    assert any("inertial" in fragment and "0.718" in fragment for fragment in kept)


def test_a_vendors_own_tag_rides_along_too(extra: Any) -> None:
    stage = _stage(urdf_to_usd(extra))

    kept = _passthrough(stage, "/extra/base_link")

    assert any("keep me" in fragment for fragment in kept)


def test_what_the_converter_does_read_is_not_duplicated_into_the_passthrough(
    extra: Any,
) -> None:
    stage = _stage(urdf_to_usd(extra))

    kept = _passthrough(stage, "/extra/base_link")

    assert not any("<visual" in fragment for fragment in kept)


def test_a_joints_unconverted_urdf_rides_along_on_its_joint_prim(extra: Any) -> None:
    stage = _stage(urdf_to_usd(extra))

    kept = _passthrough(stage, "/extra/joints/shoulder")

    assert any("dynamics" in fragment and "0.7" in fragment for fragment in kept)
    assert not any("<origin" in fragment for fragment in kept)


def test_a_link_with_nothing_unconverted_carries_no_passthrough(extra: Any) -> None:
    stage = _stage(urdf_to_usd(extra))

    assert stage.GetPrimAtPath("/extra/base_link/arm").GetCustomDataByKey(PASSTHROUGH_KEY) is None


def test_passthrough_is_customdata_not_an_attribute(extra: Any) -> None:
    usd = urdf_to_usd(extra)

    assert "customData" in usd
    assert "custom " not in usd


_FLOATING_URDF = """<?xml version="1.0"?>
<robot name="floater">
  <link name="world"/>
  <link name="base_link"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link>
  <joint name="floating_base_joint" type="floating">
    <parent link="world"/><child link="base_link"/>
    <origin xyz="0 0 0.8"/>
  </joint>
</robot>
"""


@pytest.fixture
def floater(tmp_path: Path) -> Any:
    urdf = tmp_path / "floater.urdf"
    urdf.write_text(_FLOATING_URDF)
    return RobotModel.from_file(urdf).load()


def test_a_floating_base_converts_at_all(floater: Any) -> None:
    stage = _stage(urdf_to_usd(floater))

    assert stage.GetPrimAtPath("/floater/world/base_link")


def test_a_floating_joints_child_sits_at_the_joint_origin(floater: Any) -> None:
    from pxr import UsdGeom

    stage = _stage(urdf_to_usd(floater))
    prim = stage.GetPrimAtPath("/floater/world/base_link")

    transform = UsdGeom.Xformable(prim).GetLocalTransformation()
    np.testing.assert_allclose(tuple(transform.ExtractTranslation()), (0, 0, 0.8), atol=1e-9)


def test_a_floating_joint_is_the_generic_joint(floater: Any) -> None:
    stage = _stage(urdf_to_usd(floater))

    prim = stage.GetPrimAtPath("/floater/joints/floating_base_joint")
    assert prim, [p.GetName() for p in stage.Traverse()]
    assert str(prim.GetTypeName()) == "PhysicsJoint"
    assert not prim.GetAttribute("physics:axis")


def test_a_planar_joint_locks_the_axes_out_of_its_plane(tmp_path: Path) -> None:
    source = tmp_path / "cart.urdf"
    source.write_text(
        '<robot name="cart"><link name="world"/><link name="body"/>'
        '<joint name="planar_base" type="planar"><parent link="world"/>'
        '<child link="body"/></joint></robot>'
    )
    stage = _stage(urdf_to_usd(RobotModel.from_file(source).load()))

    prim = stage.GetPrimAtPath("/cart/joints/planar_base")
    assert prim
    applied = {str(name) for name in prim.GetAppliedSchemas()}
    assert {"PhysicsLimitAPI:transZ", "PhysicsLimitAPI:rotX", "PhysicsLimitAPI:rotY"} <= applied
    assert prim.GetAttribute("limit:transZ:physics:high").Get() == 0.0


_HASHED_URDF = """<?xml version="1.0"?>
<robot name="hashy">
  <link name="base_link">
    <visual><geometry><mesh filename="meshes/arm.stl"/></geometry></visual>
  </link>
</robot>
"""


@pytest.fixture
def hashy(tmp_path: Path) -> Any:
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "arm.stl").write_bytes(b"solid arm\nendsolid arm\n")
    urdf = tmp_path / "hashy.urdf"
    urdf.write_text(_HASHED_URDF)
    return RobotModel.from_file(urdf).load()


def test_a_mesh_is_identified_by_the_hash_of_its_contents(hashy: Any) -> None:
    stage = _stage(urdf_to_usd(hashy))
    prim = stage.GetPrimAtPath("/hashy/base_link/visual_0")

    expected = asset_hash(b"solid arm\nendsolid arm\n")
    assert prim.GetAssetInfo()["version"] == expected


def test_the_asset_identifier_is_the_path_it_references(hashy: Any) -> None:
    stage = _stage(urdf_to_usd(hashy))
    prim = stage.GetPrimAtPath("/hashy/base_link/visual_0")

    assert prim.GetAssetInfo()["identifier"].path.endswith("meshes/arm.stl")


def test_a_hashed_layer_passes_the_requirement(hashy: Any) -> None:
    require_asset_hashes(urdf_to_usd(hashy), what="this layer")


def test_a_layer_with_no_assets_at_all_passes(toy: Any) -> None:
    require_asset_hashes(urdf_to_usd(toy), what="this layer")


def test_an_unhashed_asset_is_refused_with_the_hash_it_should_have_had(hashy: Any) -> None:
    usd = urdf_to_usd(hashy).replace("string version", "string notversion")

    with pytest.raises(ValueError) as caught:
        require_asset_hashes(usd, what="the subtree from SomeSensor")

    message = str(caught.value)
    assert "the subtree from SomeSensor" in message
    assert "meshes/arm.stl" in message
    assert asset_hash(b"solid arm\nendsolid arm\n") in message


def test_an_unreadable_asset_is_refused_but_says_where_to_hash_it(toy: Any, tmp_path: Path) -> None:
    urdf = tmp_path / "gone.urdf"
    urdf.write_text(
        '<?xml version="1.0"?><robot name="gone"><link name="base_link">'
        '<visual><geometry><mesh filename="/nowhere/at/all.stl"/></geometry></visual>'
        "</link></robot>"
    )
    usd = urdf_to_usd(RobotModel.from_file(urdf).load())

    with pytest.raises(ValueError, match="hash it where the file lives"):
        require_asset_hashes(usd, what="this layer")


def test_unhashed_assets_names_the_prim_carrying_each_one(hashy: Any) -> None:
    usd = urdf_to_usd(hashy).replace("string version", "string notversion")

    missing = unhashed_assets(usd)

    assert list(missing.values()) == ["/hashy/base_link/visual_0"]


def test_a_texture_is_an_asset_the_hash_check_sees(tmp_path: Any) -> None:
    from pxr import Sdf, Usd, UsdGeom, UsdShade

    image = tmp_path / "wood.png"
    image.write_bytes(b"not really a png, but it is a file")
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, "/scene")
    stage.SetDefaultPrim(root.GetPrim())
    texture = UsdShade.Shader.Define(stage, "/scene/Looks/wood/texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(image))

    assert str(image) in unhashed_assets(stage.GetRootLayer().ExportToString())


def test_a_hashed_texture_satisfies_the_check(tmp_path: Any) -> None:
    from pxr import Sdf, Usd, UsdGeom, UsdShade

    image = tmp_path / "wood.png"
    image.write_bytes(b"not really a png, but it is a file")
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, "/scene")
    stage.SetDefaultPrim(root.GetPrim())
    texture = UsdShade.Shader.Define(stage, "/scene/Looks/wood/texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(image))
    texture.GetPrim().SetAssetInfo({"version": asset_hash(image.read_bytes())})

    assert unhashed_assets(stage.GetRootLayer().ExportToString()) == {}


def test_a_scene_can_rename_the_root_everything_hangs_off(toy: Any) -> None:
    scene = urdf_to_usd(toy, name="office")

    assert "office" in usd_frame_names(scene)
    renamed = rename_usd_frames(scene, {"office": "world"})

    assert 'displayName = "world"' in renamed
    assert "world" in usd_frame_names(renamed)


def test_a_link_carries_its_mass_where_a_physics_engine_looks_for_it(tmp_path: Path) -> None:
    from pxr import UsdPhysics

    urdf = tmp_path / "weighed.urdf"
    urdf.write_text(
        """<?xml version="1.0"?>
<robot name="weighed">
  <link name="base">
    <inertial>
      <origin xyz="0 0 -0.076" rpy="0 0 0"/>
      <mass value="3.813"/>
      <inertia ixx="0.010549" iyy="0.0093089" izz="0.0079184" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="frame_only"/>
  <joint name="base_to_frame" type="fixed">
    <parent link="base"/><child link="frame_only"/>
  </joint>
</robot>
"""
    )
    stage = _stage(urdf_to_usd(RobotModel.from_file(urdf).load(), name="weighed"))

    base = _prim_by_link(stage, "base")
    assert base.HasAPI(UsdPhysics.MassAPI)
    mass = UsdPhysics.MassAPI(base)
    assert mass.GetMassAttr().Get() == pytest.approx(3.813)
    assert tuple(mass.GetCenterOfMassAttr().Get()) == pytest.approx((0.0, 0.0, -0.076))
    assert tuple(mass.GetDiagonalInertiaAttr().Get()) == pytest.approx(
        (0.0079184, 0.0093089, 0.010549), rel=1e-5
    )

    assert not _prim_by_link(stage, "frame_only").HasAPI(UsdPhysics.MassAPI)


def test_an_off_diagonal_inertia_is_diagonalised_rather_than_dropped(tmp_path: Path) -> None:
    from pxr import UsdPhysics

    urdf = tmp_path / "tilted.urdf"
    urdf.write_text(
        """<?xml version="1.0"?>
<robot name="tilted">
  <link name="base">
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.02" iyy="0.03" izz="0.04" ixy="0.005" ixz="0" iyz="0"/>
    </inertial>
  </link>
</robot>
"""
    )
    stage = _stage(urdf_to_usd(RobotModel.from_file(urdf).load(), name="tilted"))
    mass = UsdPhysics.MassAPI(_prim_by_link(stage, "base"))
    moments = tuple(mass.GetDiagonalInertiaAttr().Get())
    assert sum(moments) == pytest.approx(0.02 + 0.03 + 0.04)
    axes = mass.GetPrincipalAxesAttr().Get()
    assert axes is not None
