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
from dimos.robot.assets.usd_utils.assets import (
    asset_hash,
    describe_unopenable_assets,
    require_asset_hashes,
    require_fetched_assets,
    unhashed_assets,
    unopenable_assets,
)
from dimos.robot.assets.usd_utils.frames import (
    prefix_usd_frames,
    rename_usd_frames,
    usd_frame_names,
    usd_with_frames,
)
from dimos.robot.assets.usd_utils.from_urdf import PASSTHROUGH_KEY, urdf_to_usd
from dimos.robot.assets.usd_utils.layers import graft_usd

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


def test_two_link_names_that_sanitise_alike_still_get_their_own_prims(tmp_path: Path) -> None:
    """Tf.MakeValidIdentifier is not injective, so the second name needs a prim of its own."""
    source = tmp_path / "twins.urdf"
    source.write_text(
        '<robot name="twins"><link name="base"/><link name="arm-1"/><link name="arm_1"/>'
        '<joint name="a" type="fixed"><parent link="base"/><child link="arm-1"/></joint>'
        '<joint name="b" type="fixed"><parent link="base"/><child link="arm_1"/></joint>'
        "</robot>"
    )
    stage = _stage(urdf_to_usd(RobotModel.from_file(source).load()))

    paths = {_prim_by_link(stage, name).GetPath() for name in ("arm-1", "arm_1")}
    assert len(paths) == 2, f"both links landed on one prim: {paths}"


def test_two_joint_names_that_sanitise_alike_still_get_their_own_prims(tmp_path: Path) -> None:
    source = tmp_path / "twins.urdf"
    source.write_text(
        '<robot name="twins"><link name="base"/><link name="mid"/><link name="tip"/>'
        '<joint name="turn-1" type="fixed"><parent link="base"/><child link="mid"/></joint>'
        '<joint name="turn_1" type="fixed"><parent link="mid"/><child link="tip"/></joint>'
        "</robot>"
    )
    stage = _stage(urdf_to_usd(RobotModel.from_file(source).load()))

    named = {
        prim.GetDisplayName(): str(prim.GetPath())
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith("/twins/joints/")
    }
    assert set(named) == {"turn-1", "turn_1"}, named
    assert len(set(named.values())) == 2, named


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


def test_an_added_frames_pose_survives_into_the_kinematics(toy: Any) -> None:
    """The prim transform places the body, but tf is rebuilt from the weld's local frames."""
    import math

    import numpy as np
    from scipy.spatial.transform import Rotation

    from dimos.robot.assets.model import UsdModel

    xyz, rpy = (0.1, 0.0, 0.42), (math.pi, -0.04, 0.0)
    scene = usd_with_frames(urdf_to_usd(toy), {"spot": ("base_link", xyz, rpy)})

    joint = UsdModel.from_layer(scene).get_joint("spot_joint")
    assert joint is not None, "the added frame is a joint of the description"
    placed = joint.parent_from_child(0.0)
    expected = np.eye(4)
    expected[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    expected[:3, 3] = xyz
    assert np.allclose(placed, expected, atol=1e-6), "the frame hangs where it was added"


def test_a_frame_welded_to_the_world_converts_to_a_joint_on_the_scene_root(toy: Any) -> None:
    import xml.etree.ElementTree as ElementTree

    from dimos.robot.assets.usd_utils.to_urdf_mask import usd_to_urdf

    scene = usd_with_frames(urdf_to_usd(toy), {"spot": ("toy", (0.3, 0.0, 0.7), (0.0, 0.0, 0.0))})

    urdf = ElementTree.fromstring(usd_to_urdf(scene))
    links = {link.get("name") for link in urdf.findall("link")}
    parent_of = {
        joint.find("child").get("link"): joint.find("parent").get("link")
        for joint in urdf.findall("joint")
    }

    assert "toy" in links, "the scene root is a link even though it is not a rigid body"
    assert parent_of["spot"] == "toy", "a body0-less weld hangs off the scene root"
    origin = next(
        joint.find("origin")
        for joint in urdf.findall("joint")
        if joint.find("child").get("link") == "spot"
    )
    assert [float(value) for value in origin.get("xyz").split()] == [0.3, 0.0, 0.7], (
        "the weld carries where the frame actually sits, not the joint's zero offset"
    )
    assert "spot" not in {name for name in links if name not in parent_of}, (
        "the frame is no longer a root of its own disconnected tree"
    )


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


def test_a_planar_joint_moves_in_the_plane_its_normal_names(tmp_path: Path) -> None:
    """URDF's planar axis is the plane's normal, and the locks USD writes are about joint Z."""
    from dimos.robot.assets.usd_utils.to_urdf_mask import usd_to_urdf

    source = tmp_path / "cart.urdf"
    source.write_text(
        '<robot name="cart"><link name="world"/><link name="body"/>'
        '<joint name="planar_base" type="planar"><parent link="world"/>'
        '<child link="body"/><axis xyz="1 0 0"/></joint></robot>'
    )
    recovered = tmp_path / "back.urdf"
    recovered.write_text(usd_to_urdf(urdf_to_usd(RobotModel.from_file(source).load())))

    joint = RobotModel.from_file(recovered).load().get_joint("planar_base")
    assert joint is not None
    np.testing.assert_allclose(joint.axis, [1.0, 0.0, 0.0], atol=1e-6)


def test_a_mimic_joint_keeps_following_its_leader(tmp_path: Path) -> None:
    """USD cannot say "this joint follows that one", so mimic has to travel as passthrough."""
    from dimos.robot.assets.usd_utils.to_urdf_mask import usd_to_urdf

    source = tmp_path / "gripper.urdf"
    source.write_text(
        '<robot name="gripper"><link name="palm"/><link name="left"/><link name="right"/>'
        '<joint name="lead" type="revolute"><parent link="palm"/><child link="left"/>'
        '<axis xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint>'
        '<joint name="follow" type="revolute"><parent link="palm"/><child link="right"/>'
        '<axis xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="1"/>'
        '<mimic joint="lead" multiplier="-1" offset="0.5"/></joint></robot>'
    )
    recovered = tmp_path / "back.urdf"
    recovered.write_text(usd_to_urdf(urdf_to_usd(RobotModel.from_file(source).load())))

    joint = RobotModel.from_file(recovered).load().get_joint("follow")
    assert joint is not None
    assert joint.mimic == ("lead", -1.0, 0.5), "the follower came back independent"


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


def test_a_release_number_in_the_version_field_is_not_taken_for_a_hash(hashy: Any) -> None:
    """USD's version is free-form, so only a sha256 digest can satisfy the hash requirement."""
    layer = urdf_to_usd(hashy)
    posing = layer.replace(
        next(line for line in layer.splitlines() if "sha256:" in line).strip(),
        'string version = "1.0"',
    )

    assert unhashed_assets(posing), "a release number passed as the digest consumers need"
    with pytest.raises(ValueError, match="no usable hash"):
        require_asset_hashes(posing, what="the layer")


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


_REFERENCING_LAYER = """#usda 1.0
(
    defaultPrim = "room"
)

def Xform "room"
{
    def Mesh "there" (
        prepend references = @meshes/there.usdc@
    )
    {
    }

    def Mesh "pointer" (
        prepend references = @meshes/pointer.usdc@
    )
    {
    }

    def Mesh "gone" (
        prepend references = @meshes/gone.usdc@
    )
    {
    }
}
"""


def _referencing_layer(tmp_path: Path) -> Path:
    meshes = tmp_path / "meshes"
    meshes.mkdir()
    (meshes / "there.usdc").write_bytes(b"not really a crate, but readable")
    (meshes / "pointer.usdc").write_bytes(
        b"version https://git-lfs.github.com/spec/v1\noid sha256:0\nsize 1\n"
    )
    return tmp_path


def test_unopenable_assets_separates_a_pointer_stub_from_a_missing_file(tmp_path: Path) -> None:
    root = _referencing_layer(tmp_path)

    unopenable = unopenable_assets(_REFERENCING_LAYER, root)

    assert set(unopenable) == {"meshes/pointer.usdc", "meshes/gone.usdc"}
    assert "Git LFS pointer" in unopenable["meshes/pointer.usdc"]
    assert "No such file" in unopenable["meshes/gone.usdc"]


def test_unopenable_assets_counts_every_relative_asset_when_no_root_was_given(
    tmp_path: Path,
) -> None:
    _referencing_layer(tmp_path)

    unopenable = unopenable_assets(_REFERENCING_LAYER, None)

    assert set(unopenable) == {
        "meshes/there.usdc",
        "meshes/pointer.usdc",
        "meshes/gone.usdc",
    }
    assert all("no asset root" in why for why in unopenable.values())


def test_unopenable_assets_passes_a_layer_whose_every_reference_is_on_disk(
    tmp_path: Path,
) -> None:
    root = _referencing_layer(tmp_path)
    (root / "meshes" / "pointer.usdc").write_bytes(b"real now")
    (root / "meshes" / "gone.usdc").write_bytes(b"here now")

    assert unopenable_assets(_REFERENCING_LAYER, root) == {}


def test_describe_unopenable_assets_says_which_files_and_how_to_fetch_them(
    tmp_path: Path,
) -> None:
    root = _referencing_layer(tmp_path)

    message = describe_unopenable_assets(
        unopenable_assets(_REFERENCING_LAYER, root), root, what="the office scene"
    )

    assert "the office scene names 2 asset(s)" in message
    assert "meshes/pointer.usdc" in message
    assert f'git lfs checkout "{root / "meshes"}"' in message


def test_an_unfetched_pointer_is_refused_and_a_merely_missing_file_is_not(
    tmp_path: Path,
) -> None:
    root = _referencing_layer(tmp_path)
    unopenable = unopenable_assets(_REFERENCING_LAYER, root)

    with pytest.raises(FileNotFoundError) as error:
        require_fetched_assets(unopenable, root, what="the office scene")

    message = str(error.value)
    assert "names 1 asset(s)" in message, "only the pointer stub, not the missing file"
    assert "meshes/pointer.usdc" in message
    assert "meshes/gone.usdc" not in message
    assert "GIT_LFS_SKIP_SMUDGE" in message

    (root / "meshes" / "pointer.usdc").write_bytes(b"real now")
    require_fetched_assets(unopenable_assets(_REFERENCING_LAYER, root), root, what="the scene")


def test_unopenable_assets_ignores_names_a_scene_published_for_the_bus() -> None:
    published = _REFERENCING_LAYER.replace("meshes/", "/dimos/assets/room/")

    assert unopenable_assets(published, None) == {}


_SCENE_WITH_ITS_OWN_SHADER = """#usda 1.0
(
    defaultPrim = "room"
)

over "Shared"
{
    def Shader "Surface"
    {
        uniform token info:id = "UsdPreviewSurface"
        color3f inputs:diffuseColor = (1, 0, 0)
    }
}

def Xform "room"
{
}
"""

_SUBTREE_WITH_AN_OUTSIDE_SHADER = """#usda 1.0
(
    defaultPrim = "arm_sub"
)

over "Shared"
{
    def Shader "Surface"
    {
        uniform token info:id = "UsdPreviewSurface"
        color3f inputs:diffuseColor = (0, 1, 0)
    }
}

def Xform "arm_sub"
{
    def Material "paint" (
        prepend references = </Shared>
    )
    {
    }
}
"""


def test_a_grafted_subtree_keeps_its_own_shaders_when_the_scene_has_prims_by_the_same_name() -> (
    None
):
    from pxr import Gf

    stage = _stage(graft_usd(_SCENE_WITH_ITS_OWN_SHADER, {"room": _SUBTREE_WITH_AN_OUTSIDE_SHADER}))

    surface = stage.GetPrimAtPath("/room/arm_sub/paint/Surface")
    assert surface, "the grafted material composed no shader at all"
    assert surface.GetAttribute("inputs:diffuseColor").Get() == Gf.Vec3f(0.0, 1.0, 0.0), (
        "the grafted subtree is wearing the scene's shader instead of its own: a converted "
        "asset keeps its shader networks beside its default prim, and grafting left the "
        "reference pointing at whatever owns that path in the scene"
    )


_SCENE_WIRED_BY_CONNECTION = """#usda 1.0
(
    defaultPrim = "room"
)

def Scope "Shared"
{
    def Shader "Surface"
    {
        uniform token info:id = "UsdPreviewSurface"
        color3f inputs:diffuseColor = (1, 0, 0)
        token outputs:surface
    }
}

def Xform "room"
{
}
"""

_SUBTREE_WIRED_BY_CONNECTION = """#usda 1.0
(
    defaultPrim = "arm_sub"
)

def Scope "Shared"
{
    def Shader "Surface"
    {
        uniform token info:id = "UsdPreviewSurface"
        color3f inputs:diffuseColor = (0, 1, 0)
        token outputs:surface
    }
}

def Xform "arm_sub"
{
    def Material "paint"
    {
        token outputs:surface.connect = </Shared/Surface.outputs:surface>
    }
}
"""


def test_a_grafted_material_wired_by_connection_keeps_its_own_shader() -> None:
    """Not every asset wires its network through a reference; some connect straight to it."""
    from pxr import Gf

    stage = _stage(graft_usd(_SCENE_WIRED_BY_CONNECTION, {"room": _SUBTREE_WIRED_BY_CONNECTION}))

    paint = stage.GetPrimAtPath("/room/arm_sub/paint").GetAttribute("outputs:surface")
    connections = paint.GetConnections()
    assert connections, "the grafted material lost its surface connection"
    shader = stage.GetPrimAtPath(connections[0].GetPrimPath())
    assert shader.GetAttribute("inputs:diffuseColor").Get() == Gf.Vec3f(0.0, 1.0, 0.0), (
        "the grafted subtree is wearing the scene's shader: its connection named a prim "
        "beside the subtree, and grafting left the absolute path alone"
    )


_SUBTREE_WIRED_TO_WHAT_IT_DOES_NOT_STATE = """#usda 1.0
(
    defaultPrim = "arm_sub"
)

def Xform "arm_sub"
{
    def Material "paint"
    {
        token outputs:surface.connect = </Flattened_Prototype_1.outputs:surface>
    }
}
"""


def test_a_graft_leaves_alone_a_connection_to_a_prim_the_layer_does_not_state() -> None:
    """An instancing prototype is composed, not authored, so Sdf.CopySpec cannot carry it."""
    grafted = graft_usd(
        _SCENE_WIRED_BY_CONNECTION, {"room": _SUBTREE_WIRED_TO_WHAT_IT_DOES_NOT_STATE}
    )

    stage = _stage(grafted)
    paint = stage.GetPrimAtPath("/room/arm_sub/paint").GetAttribute("outputs:surface")
    assert [str(path) for path in paint.GetConnections()] == [
        "/Flattened_Prototype_1.outputs:surface"
    ]


_SCENE_WITH_TWO_SHADERS = """#usda 1.0
(
    defaultPrim = "room"
)

over "Shared"
{
    def Shader "Surface"
    {
        color3f inputs:diffuseColor = (1, 0, 0)
    }
}

over "Deeper"
{
    def Shader "Underneath"
    {
        color3f inputs:diffuseColor = (1, 0, 0)
    }
}

def Xform "room"
{
}
"""

_SUBTREE_REACHING_TWO_HOPS = """#usda 1.0
(
    defaultPrim = "arm_sub"
)

over "Deeper"
{
    def Shader "Underneath"
    {
        color3f inputs:diffuseColor = (0, 0, 1)
    }
}

over "Shared" (
    prepend references = </Deeper>
)
{
    def Shader "Surface"
    {
        color3f inputs:diffuseColor = (0, 1, 0)
    }
}

def Xform "arm_sub"
{
    def Material "paint" (
        prepend references = </Shared>
    )
    {
    }
}
"""


def test_carrying_a_referenced_prim_also_carries_what_that_prim_reaches() -> None:
    from pxr import Gf

    stage = _stage(graft_usd(_SCENE_WITH_TWO_SHADERS, {"room": _SUBTREE_REACHING_TWO_HOPS}))

    paint = stage.GetPrimAtPath("/room/arm_sub/paint")
    assert paint, "the grafted material is missing"
    composed = paint.GetPrototype() if paint.IsInstance() else paint
    shaders = {child.GetName(): child for child in composed.GetChildren()}
    assert sorted(shaders) == ["Surface", "Underneath"], (
        f"the second hop did not compose: got {sorted(shaders)}"
    )
    for name, expected in (("Surface", (0.0, 1.0, 0.0)), ("Underneath", (0.0, 0.0, 1.0))):
        assert shaders[name].GetAttribute("inputs:diffuseColor").Get() == Gf.Vec3f(*expected), (
            f"{name} resolved to the scene's shader, so a prim carried over for the subtree "
            f"kept an arc pointing at whatever owned that path in the scene"
        )
