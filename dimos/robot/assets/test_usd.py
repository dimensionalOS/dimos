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
from dimos.robot.assets.usd import (
    PASSTHROUGH_KEY,
    asset_hash,
    graft_usd,
    require_asset_hashes,
    unhashed_assets,
    urdf_to_usd,
)

# usd-core is an optional extra, unavailable on aarch64 before Python 3.11 (see assets/usd.py).
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


def _stage(usd: str) -> Any:
    from pxr import Sdf, Usd

    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd)
    return Usd.Stage.Open(layer)


def _prim_by_link(stage: Any, link: str) -> Any:
    """The link prim whose name is ``link``, the same way graft_usd finds a mount frame."""
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
    # "arm-1" cannot be a prim name, so displayName is the only way back to the tf frame.
    # It is USD's own field for a name that identifier syntax cannot hold.
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
    # The file never has to exist: nothing here reads it.
    urdf = tmp_path / "meshy.urdf"
    urdf.write_text(_mesh_urdf("meshes/box.stl"))
    return RobotModel.from_file(urdf).load()


def _references(prim: Any) -> list[str]:
    from pxr import Usd

    items: list[str] = []
    for spec in prim.GetPrimStack():
        for ref in spec.referenceList.GetAddedOrExplicitItems():
            items.append(ref.assetPath)
    assert Usd  # the import documents where GetPrimStack comes from
    return items


def test_a_mesh_names_the_path_the_urdf_named(meshy: Any) -> None:
    # assetInfo is USD's own "which asset is this prim", so no attribute of ours and no
    # triangles of ours. UsdManager already serves exactly these paths.
    stage = _stage(urdf_to_usd(meshy))
    mesh = stage.GetPrimAtPath("/meshy/base_link/visual_0")

    assert str(mesh.GetAssetInfo()["identifier"].path) == "meshes/box.stl"


def test_a_mesh_file_is_not_referenced_as_though_it_were_a_usd_layer(meshy: Any) -> None:
    """A reference arc composes another USD layer; an STL is not one.

    Authoring one anyway makes the layer invalid: every standard consumer reports "Cannot
    determine file format ... target=usd" and composes the mesh away, so the robot has no
    geometry in usdview, in Isaac or in MuJoCo. dimos could not see it because it reads the
    path out of the layer and serves the file itself instead of composing the stage.
    """
    stage = _stage(urdf_to_usd(meshy))

    offenders = {
        str(prim.GetPath()): path
        for prim in stage.Traverse()
        for path in _references(prim)
        if not path.endswith((".usd", ".usda", ".usdc", ".usdz"))
    }

    assert offenders == {}


def test_a_mesh_carries_no_geometry_of_its_own(meshy: Any) -> None:
    # Nothing is read, parsed or converted, so trimesh is not a dependency of this path.
    stage = _stage(urdf_to_usd(meshy))
    mesh = stage.GetPrimAtPath("/meshy/base_link/visual_0")

    assert not mesh.GetAttribute("points").HasAuthoredValue()


def test_the_layer_authors_no_custom_attributes(toy: Any) -> None:
    # Standard schemas only: a consumer should need nothing of ours to read this.
    usd = urdf_to_usd(toy)

    assert "custom " not in usd
    assert "dimos:" not in usd


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
    """<inertial> means nothing to this converter, which is not a reason to lose it."""
    stage = _stage(urdf_to_usd(extra))

    kept = _passthrough(stage, "/extra/base_link")

    assert any("inertial" in fragment and "0.718" in fragment for fragment in kept)


def test_a_vendors_own_tag_rides_along_too(extra: Any) -> None:
    # The whole point: a converter cannot know what a vendor put in there.
    stage = _stage(urdf_to_usd(extra))

    kept = _passthrough(stage, "/extra/base_link")

    assert any("keep me" in fragment for fragment in kept)


def test_what_the_converter_does_read_is_not_duplicated_into_the_passthrough(
    extra: Any,
) -> None:
    # visual became a Cube prim, so keeping the XML too would be two sources of truth.
    stage = _stage(urdf_to_usd(extra))

    kept = _passthrough(stage, "/extra/base_link")

    assert not any("<visual" in fragment for fragment in kept)


def test_a_joints_unconverted_urdf_rides_along_on_its_joint_prim(extra: Any) -> None:
    stage = _stage(urdf_to_usd(extra))

    kept = _passthrough(stage, "/extra/joints/shoulder")

    assert any("dynamics" in fragment and "0.7" in fragment for fragment in kept)
    # origin is read into the joint frames, so it is not duplicated here either.
    assert not any("<origin" in fragment for fragment in kept)


def test_a_link_with_nothing_unconverted_carries_no_passthrough(extra: Any) -> None:
    # No empty dictionaries on every prim of every robot.
    stage = _stage(urdf_to_usd(extra))

    assert stage.GetPrimAtPath("/extra/base_link/arm").GetCustomDataByKey(PASSTHROUGH_KEY) is None


def test_passthrough_is_customdata_not_an_attribute(extra: Any) -> None:
    # customData is USD's own field for this, so the "no custom attributes" rule still holds.
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
    """A floating joint has no single position, which is the whole point of it.

    Asking one to pose its child raises, so a robot with a floating base -- a g1, a go2,
    anything that walks -- could not be converted at all.
    """
    stage = _stage(urdf_to_usd(floater))

    assert stage.GetPrimAtPath("/floater/world/base_link")


def test_a_floating_joints_child_sits_at_the_joint_origin(floater: Any) -> None:
    # Its real pose comes from odometry; the origin is the honest stand-in, and it is what
    # parent_from_child(0) gives for every other joint type.
    from pxr import UsdGeom

    stage = _stage(urdf_to_usd(floater))
    prim = stage.GetPrimAtPath("/floater/world/base_link")

    transform = UsdGeom.Xformable(prim).GetLocalTransformation()
    np.testing.assert_allclose(tuple(transform.ExtractTranslation()), (0, 0, 0.8), atol=1e-9)


def test_a_floating_joint_is_the_generic_joint(floater: Any) -> None:
    """The base PhysicsJoint is free in all six degrees, which is what a URDF floating joint is.

    Standard, so a reader does not need metadata of ours to recognise one -- and a prim means a
    layer read back can tell a free base from a body that is merely nested inside another.
    """
    stage = _stage(urdf_to_usd(floater))

    prim = stage.GetPrimAtPath("/floater/joints/floating_base_joint")
    assert prim, [p.GetName() for p in stage.Traverse()]
    assert str(prim.GetTypeName()) == "PhysicsJoint"
    # Nothing locked, and no axis: the generic joint has none.
    assert not prim.GetAttribute("physics:axis")


def test_a_planar_joint_locks_the_axes_out_of_its_plane(tmp_path: Path) -> None:
    """A URDF planar joint moves in its own xy and turns about z, so the other three are locked.

    PhysicsLimitAPI is the standard way to say that, which keeps it apart from a floating joint
    without either of them needing a custom field.
    """
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
    """A path alone cannot say whether the file fetched is the file meant."""
    stage = _stage(urdf_to_usd(hashy))
    prim = stage.GetPrimAtPath("/hashy/base_link/visual_0")

    expected = asset_hash(b"solid arm\nendsolid arm\n")
    assert prim.GetAssetInfo()["version"] == expected


def test_the_asset_identifier_is_the_path_it_references(hashy: Any) -> None:
    stage = _stage(urdf_to_usd(hashy))
    prim = stage.GetPrimAtPath("/hashy/base_link/visual_0")

    assert prim.GetAssetInfo()["identifier"].path.endswith("meshes/arm.stl")


def test_a_hashed_layer_passes_the_requirement(hashy: Any) -> None:
    require_asset_hashes(urdf_to_usd(hashy), what="this layer")  # does not raise


def test_a_layer_with_no_assets_at_all_passes(toy: Any) -> None:
    # Nothing to identify, so nothing to refuse.
    require_asset_hashes(urdf_to_usd(toy), what="this layer")


def test_an_unhashed_asset_is_refused_with_the_hash_it_should_have_had(hashy: Any) -> None:
    """The answer is the fix: the error carries the value to paste.

    That matters most for a layer that is written by hand or baked into a constant, where
    there is no converter to have done it for you.
    """
    usd = urdf_to_usd(hashy).replace("string version", "string notversion")

    with pytest.raises(ValueError) as caught:
        require_asset_hashes(usd, what="the subtree from SomeSensor")

    message = str(caught.value)
    assert "the subtree from SomeSensor" in message
    assert "meshes/arm.stl" in message
    assert asset_hash(b"solid arm\nendsolid arm\n") in message


def test_an_unreadable_asset_is_refused_but_says_where_to_hash_it(toy: Any, tmp_path: Path) -> None:
    # A consumer cannot hash a file it does not have; it can still say what is wrong.
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
