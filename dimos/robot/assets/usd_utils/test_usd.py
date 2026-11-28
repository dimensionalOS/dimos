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

import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

pytest.importorskip("pxr")

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from dimos.robot.assets.model import RobotModel, UsdModel
from dimos.robot.assets.usd_utils.assets import (
    asset_hash,
    publish_usd_assets,
    referenced_asset_paths,
    resolve_usd_assets,
)
from dimos.robot.assets.usd_utils.frames import (
    usd_frame_edges,
    usd_frame_names,
    usd_with_frame_names,
    usd_with_frames,
    usd_world_frame_prims,
)
from dimos.robot.assets.usd_utils.from_urdf import PASSTHROUGH_KEY, urdf_to_usd
from dimos.robot.assets.usd_utils.joints import usd_joints, usd_root_link
from dimos.robot.assets.usd_utils.layers import usd_moved_under, usd_under_namespace

_TOY_URDF = """<robot name="toy">
  <material name="green"><color rgba="0 0 1 1"/></material>
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.05"/><geometry><box size="0.4 0.3 0.1"/></geometry>
      <material name="green"><color rgba="0.2 0.6 0.2 1"/></material>
    </visual>
    <collision><geometry><cylinder radius="0.2" length="0.1"/></geometry></collision>
    <inertial>
      <origin xyz="0 0 -0.076" rpy="0 0 0"/><mass value="3.813"/>
      <inertia ixx="0.010549" iyy="0.0093089" izz="0.0079184" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <vendor:thing xmlns:vendor="http://example.com" note="keep me"/>
  </link>
  <link name="mount"/><link name="spin"/><link name="slider"/>
  <link name="arm_1">
    <visual><geometry><sphere radius="0.05"/></geometry><material name="green"/></visual>
    <inertial>
      <mass value="2"/><inertia ixx="0.02" iyy="0.03" izz="0.04" ixy="0.005" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="base_to_mount" type="fixed">
    <parent link="base_link"/><child link="mount"/>
    <origin xyz="0.1 0 0.1" rpy="0 0 0.3"/><dynamics damping="0.7"/>
  </joint>
  <joint name="mount_to_arm" type="revolute">
    <parent link="mount"/><child link="arm_1"/>
    <origin xyz="0 0 0.02" rpy="0 0.2 0"/><axis xyz="0.6 0.8 0"/>
    <limit lower="-1.5707963267948966" upper="1.5707963267948966" effort="10" velocity="1"/>
  </joint>
  <joint name="arm_to_spin" type="continuous"><parent link="arm_1"/><child link="spin"/>
    <axis xyz="0 0 1"/></joint>
  <joint name="base_to_slider" type="prismatic">
    <parent link="base_link"/><child link="slider"/>
    <origin xyz="0.01 0.02 0.03" rpy="0.1 0.2 0.3"/><axis xyz="0 1 0"/>
    <limit lower="-0.05" upper="0.05" effort="5" velocity="1"/>
  </joint>
  <gazebo reference="base_link"><mu1>0.2</mu1></gazebo>
</robot>
"""


@pytest.fixture
def toy() -> Any:
    return RobotModel.from_urdf_string(_TOY_URDF).load()


def _layer(usd: str) -> Any:
    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd)
    return layer


def _stage(usd: str) -> Any:
    return Usd.Stage.Open(_layer(usd))


def _pose(rotation: Any, xyz: Any) -> Any:
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = rotation.as_matrix(), xyz
    return pose


def _converted(urdf: str) -> Any:
    return _stage(urdf_to_usd(RobotModel.from_urdf_string(urdf).load()))


def _prim_by_link(stage: Any, link: str) -> Any:
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() == link:
            return prim
    raise AssertionError(f"no prim for link {link!r}")


def _welds_holding(stage: Any, frame: Any) -> list[Any]:
    return [
        prim
        for prim in stage.Traverse()
        if prim.IsA(UsdPhysics.FixedJoint)
        and prim.GetRelationship("physics:body1").GetTargets() == [frame.GetPath()]
    ]


def test_a_urdf_becomes_a_usd_physics_layer_that_reads_back_the_same(toy: Any) -> None:
    usd = urdf_to_usd(toy)
    stage = _stage(usd)

    assert stage.GetDefaultPrim().GetName() == "toy"
    arm = _prim_by_link(stage, "arm_1")
    assert str(arm.GetPath()) == "/toy/base_link/mount/arm_1"
    assert {"toy", "toy/base_link/mount/arm_1"} <= usd_frame_names(usd)
    assert not arm.HasAuthoredDisplayName()
    mount_joint = toy.get_joint("base_to_mount")
    local = UsdGeom.Xformable(_prim_by_link(stage, "mount")).GetLocalTransformation()
    np.testing.assert_allclose(np.array(local).T, mount_joint.parent_from_child(0.0), atol=1e-9)

    revolute = stage.GetPrimAtPath("/toy/joints/mount_to_arm")
    assert revolute.GetAttribute("physics:lowerLimit").Get() == pytest.approx(-90.0, abs=1e-3)
    assert revolute.GetAttribute("physics:upperLimit").Get() == pytest.approx(90.0, abs=1e-3)
    assert str(revolute.GetAttribute("physics:axis").Get()) == "Z"
    rot = revolute.GetAttribute("physics:localRot0").Get()
    rotation = Rotation.from_quat([*rot.GetImaginary(), rot.GetReal()])
    arm_joint = toy.get_joint("mount_to_arm")
    expected_axis = Rotation.from_euler("xyz", arm_joint.origin_rpy).apply(arm_joint.axis)
    np.testing.assert_allclose(rotation.apply([0.0, 0.0, 1.0]), expected_axis, atol=1e-6)

    continuous = stage.GetPrimAtPath("/toy/joints/arm_to_spin")
    assert continuous.GetTypeName() == "PhysicsRevoluteJoint"
    assert not continuous.GetAttribute("physics:lowerLimit").HasAuthoredValue()

    collision = stage.GetPrimAtPath("/toy/base_link/collision_1")
    assert collision.GetTypeName() == "Cylinder"
    assert str(collision.GetAttribute("visibility").Get()) == "invisible"
    assert "PhysicsCollisionAPI" in collision.GetAppliedSchemas()
    cube = stage.GetPrimAtPath("/toy/base_link/visual_0")
    assert cube.GetAttribute("size").Get() == pytest.approx(1.0)
    np.testing.assert_allclose(cube.GetAttribute("xformOp:scale").Get(), (0.4, 0.3, 0.1), atol=1e-9)
    np.testing.assert_allclose(
        cube.GetAttribute("primvars:displayColor").Get()[0], (0.2, 0.6, 0.2), atol=1e-6
    )
    # A visual naming a robot-level material takes its colour; an inline one wins over it.
    sphere = stage.GetPrimAtPath("/toy/base_link/mount/arm_1/visual_0")
    np.testing.assert_allclose(
        sphere.GetAttribute("primvars:displayColor").Get()[0], (0.0, 0.0, 1.0), atol=1e-6
    )

    joints = usd_joints(usd)
    assert [joint.name for joint in joints] == [joint.name for joint in toy.joints]
    for got, original in zip(joints, toy.joints, strict=True):
        parent, child = (
            str(_prim_by_link(stage, link).GetPath()).lstrip("/")
            for link in (original.parent_link, original.child_link)
        )
        assert (got.type, got.parent_link, got.child_link) == (original.type, parent, child)
        for position in (0.0, 0.3):
            np.testing.assert_allclose(
                got.parent_from_child(position), original.parent_from_child(position), atol=1e-6
            )
        assert (got.lower, got.upper) == pytest.approx((original.lower, original.upper), abs=1e-6)
    assert usd_root_link(usd) == "toy/base_link"


def test_urdf_names_become_prim_names_or_are_refused() -> None:
    stage = _converted(
        '<robot name="twins"><link name="base"/><link name="mid"/><link name="tip"/>'
        '<joint name="turn-1" type="fixed"><parent link="base"/><child link="mid"/></joint>'
        '<joint name="turn_1" type="fixed"><parent link="mid"/><child link="tip"/></joint></robot>'
    )
    named = {
        prim.GetDisplayName(): str(prim.GetPath())
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith("/twins/joints/")
    }
    assert set(named) == {"turn-1", "turn_1"} and len(set(named.values())) == 2, named

    for link, complaint in (
        ("arm-1", "'arm-1' has at position 3 '-'"),
        ("1arm", "'1arm' starts with '1'"),
        ("a/b", "has a '/'"),
    ):
        with pytest.raises(ValueError, match=complaint):
            _converted(
                f'<robot name="bad"><link name="base"/><link name="{link}"/><joint name="j" '
                f'type="fixed"><parent link="base"/><child link="{link}"/></joint></robot>'
            )


def test_a_mesh_is_a_hashed_asset_and_scaled_shapes_sit_at_their_origin(tmp_path: Path) -> None:
    contents = b"solid arm\nendsolid arm\n"
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "box.stl").write_bytes(contents)
    origin = '<origin xyz="0.2 -0.1 0.05" rpy="0 0 0.3"/>'
    urdf = tmp_path / "meshy.urdf"
    urdf.write_text(
        f'<robot name="meshy"><link name="base_link"><visual>{origin}<geometry>'
        '<box size="0.4 0.3 0.1"/></geometry></visual><visual>'
        f'{origin}<geometry><mesh filename="meshes/box.stl" scale="0.4 0.3 0.1"/></geometry>'
        "</visual></link></robot>"
    )
    stage = _stage(urdf_to_usd(RobotModel.from_file(urdf).load()))
    mesh = stage.GetPrimAtPath("/meshy/base_link/visual_1")

    assert mesh.GetAssetInfo()["identifier"].path.endswith("meshes/box.stl")
    assert mesh.GetAssetInfo()["version"] == asset_hash(contents)
    assert not mesh.GetAttribute("points").HasAuthoredValue()
    references = [
        ref.assetPath
        for prim in stage.Traverse()
        for spec in prim.GetPrimStack()
        for ref in spec.referenceList.GetAddedOrExplicitItems()
    ]
    assert all(path.endswith((".usd", ".usda", ".usdc", ".usdz")) for path in references)
    for shape in ("visual_0", "visual_1"):
        prim = stage.GetPrimAtPath(f"/meshy/base_link/{shape}")
        world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        world = np.array(world)
        # USD matrices are row-vector: the translation is the last row, the axes scaled rows.
        np.testing.assert_allclose(world[3, :3], (0.2, -0.1, 0.05), atol=1e-9)
        np.testing.assert_allclose(
            np.linalg.norm(world[:3, :3], axis=1), (0.4, 0.3, 0.1), atol=1e-9
        )


def test_added_frames_are_welded_where_they_hang_or_refused(toy: Any) -> None:
    spot_xyz, spot_rpy = (0.1, 0.0, 0.42), (math.pi, -0.04, 0.0)
    scene = usd_with_frames(
        urdf_to_usd(toy),
        {
            "spot": ("toy/base_link", spot_xyz, spot_rpy),
            "beacon": ("toy", (0.3, 0.0, 0.7), (0.0, 0.0, 0.0)),
            "front_camera": ("toy/base_link", (0.1, 0.0, 0.2), (0.0, 0.0, 0.0)),
            "lidar": ("toy/base_link/front_camera", (0.0, 0.0, 0.05), (0.0, 0.0, 0.0)),
        },
    )
    stage = _stage(scene)

    (spot_weld,) = _welds_holding(stage, _prim_by_link(stage, "spot"))
    base_path = _prim_by_link(stage, "base_link").GetPath()
    assert spot_weld.GetRelationship("physics:body0").GetTargets() == [base_path]
    (beacon_weld,) = _welds_holding(stage, _prim_by_link(stage, "beacon"))
    assert beacon_weld.GetRelationship("physics:body0").GetTargets() == []
    camera = _prim_by_link(stage, "front_camera")
    assert stage.GetPrimAtPath(camera.GetPath().AppendChild("lidar")), "the lidar is on the camera"

    # The prim transform places the body, but tf is rebuilt from the weld's local frames.
    joints = UsdModel.from_layer(scene).joints
    spot_joint = next(joint for joint in joints if joint.name == "spot_joint")
    expected = _pose(Rotation.from_euler("xyz", spot_rpy), spot_xyz)
    np.testing.assert_allclose(spot_joint.parent_from_child(0.0), expected, atol=1e-6)
    beacon_joint = next(joint for joint in joints if joint.child_link == "toy/beacon")
    assert beacon_joint.parent_link == "toy", "a body0-less weld hangs off the scene root"
    assert beacon_joint.origin_xyz == pytest.approx((0.3, 0.0, 0.7))

    for name, parent, complaint in (
        ("mount", "toy/base_link", "is taken"),
        ("a/b", "toy/base_link", "has no '/'"),
        ("front-camera", "toy/base_link", "'front-camera' has at position 5 '-'"),
        ("spot", "nowhere", "no frame 'nowhere'"),
    ):
        with pytest.raises(ValueError, match=complaint):
            usd_with_frames(urdf_to_usd(toy), {name: (parent, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))})


def test_a_links_mass_is_where_an_engine_looks_and_the_rest_rides_along(toy: Any) -> None:
    usd = urdf_to_usd(toy)
    stage = _stage(usd)

    base = UsdPhysics.MassAPI(_prim_by_link(stage, "base_link"))
    assert base.GetMassAttr().Get() == pytest.approx(3.813)
    assert tuple(base.GetCenterOfMassAttr().Get()) == pytest.approx((0.0, 0.0, -0.076))
    assert tuple(base.GetDiagonalInertiaAttr().Get()) == pytest.approx(
        (0.0079184, 0.0093089, 0.010549), rel=1e-5
    )
    # An off-diagonal inertia is diagonalised rather than dropped.
    tilted = UsdPhysics.MassAPI(_prim_by_link(stage, "arm_1"))
    assert sum(tilted.GetDiagonalInertiaAttr().Get()) == pytest.approx(0.02 + 0.03 + 0.04)
    assert tilted.GetPrincipalAxesAttr().Get() is not None
    assert not _prim_by_link(stage, "mount").HasAPI(UsdPhysics.MassAPI)

    def kept(path: str) -> list[str]:
        return list(stage.GetPrimAtPath(path).GetCustomDataByKey(PASSTHROUGH_KEY) or [])

    link_kept = kept("/toy/base_link")
    assert any("inertial" in fragment and "3.813" in fragment for fragment in link_kept)
    assert any("keep me" in fragment for fragment in link_kept)
    assert not any("<visual" in fragment for fragment in link_kept)
    joint_kept = kept("/toy/joints/base_to_mount")
    assert any("dynamics" in fragment and "0.7" in fragment for fragment in joint_kept)
    assert not any("<origin" in fragment for fragment in joint_kept)
    assert not kept("/toy/base_link/mount")
    assert "customData" in usd and "custom " not in usd and "dimos:" not in usd


def test_floating_planar_and_mimic_joints_survive_the_trip_through_usd() -> None:
    revolute = '<axis xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="1"/>'
    urdf = (
        '<robot name="cart"><link name="world"/><link name="base"/><link name="body"/>'
        '<link name="left"/><link name="right"/>'
        '<joint name="float" type="floating"><parent link="world"/><child link="base"/>'
        '<origin xyz="0 0 0.8"/></joint>'
        '<joint name="planar" type="planar"><parent link="base"/><child link="body"/>'
        '<axis xyz="1 0 0"/></joint>'
        '<joint name="lead" type="revolute"><parent link="base"/><child link="left"/>'
        f"{revolute}</joint>"
        f'<joint name="follow" type="revolute"><parent link="base"/><child link="right"/>{revolute}'
        '<mimic joint="lead" multiplier="-1" offset="0.5"/></joint></robot>'
    )
    stage = _converted(urdf)

    base = stage.GetPrimAtPath("/cart/world/base")
    translation = UsdGeom.Xformable(base).GetLocalTransformation().ExtractTranslation()
    np.testing.assert_allclose(tuple(translation), (0, 0, 0.8), atol=1e-9)
    floating = stage.GetPrimAtPath("/cart/joints/float")
    assert str(floating.GetTypeName()) == "PhysicsJoint"
    assert not floating.GetAttribute("physics:axis")
    planar = stage.GetPrimAtPath("/cart/joints/planar")
    applied = {str(name) for name in planar.GetAppliedSchemas()}
    assert {"PhysicsLimitAPI:transZ", "PhysicsLimitAPI:rotX", "PhysicsLimitAPI:rotY"} <= applied
    assert planar.GetAttribute("limit:transZ:physics:high").Get() == 0.0

    joints = {
        joint.name: joint
        for joint in UsdModel.from_robot_model(RobotModel.from_urdf_string(urdf)).joints
    }
    # URDF's planar axis is the plane's normal, and the locks USD writes are about joint Z.
    np.testing.assert_allclose(joints["planar"].axis, [1.0, 0.0, 0.0], atol=1e-6)
    assert joints["follow"].mimic == ("lead", -1.0, 0.5), "the follower came back independent"


def _rig(joint: str, axis: str = "X", pos1: str = "(0, 0, 0.2)") -> str:
    body = '(prepend apiSchemas = ["PhysicsRigidBodyAPI"])'
    return f"""#usda 1.0
(defaultPrim = "rig")
def Xform "rig" {{
    def Xform "base" {body} {{
        def Scope "Geometry" {{
            def Xform "mast" {body} {{
                double3 xformOp:translate = (0, 0, 0.5)
                uniform token[] xformOpOrder = ["xformOp:translate"]
            }}
        }}
    }}
    def Xform "arm" {body} {{
        def {joint} "elbow" {{
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
    by_child = {joint.child_link: joint for joint in joints}
    pose, link = np.eye(4), "rig/arm"
    while link in by_child:
        joint = by_child[link]
        pose = joint.parent_from_child(angle if joint.type != "fixed" else 0.0) @ pose
        link = joint.parent_link
    return pose


def test_a_usd_joint_turns_about_its_anchor_and_axis_and_loose_bodies_are_welded() -> None:
    # base_from_joint @ motion @ inverse(arm_from_joint), straight from UsdPhysics.
    base_from_joint = _pose(Rotation.identity(), (0, 0, 1))
    joint_from_arm = np.linalg.inv(_pose(Rotation.from_euler("z", np.pi / 2), (0, 0, 0.2)))
    for joint, angles in (
        ("PhysicsFixedJoint", (0.0,)),
        ("PhysicsRevoluteJoint", (0.0, 0.7, -1.3)),
    ):
        joints = usd_joints(_rig(joint))
        for angle in angles:
            motion = _pose(Rotation.from_rotvec([angle, 0, 0]), (0, 0, 0))
            expected = base_from_joint @ motion @ joint_from_arm
            np.testing.assert_allclose(_arm_pose(joints, angle), expected, atol=1e-6)

    # The axis token is in the joint frame; localRot1 turns it into the child's frame.
    elbow = next(joint for joint in usd_joints(_rig("PhysicsRevoluteJoint", "Y", "(0, 0, 0)")))
    np.testing.assert_allclose(
        elbow.axis, Rotation.from_euler("z", np.pi / 2).apply([0, 1, 0]), atol=1e-6
    )

    # A nested body no joint holds is welded where it sits, even through a scope.
    rig = _rig("PhysicsFixedJoint")
    mast = "rig/base/Geometry/mast"
    weld = next(joint for joint in usd_joints(rig) if joint.child_link == mast)
    assert (weld.type, weld.parent_link) == ("fixed", "rig/base")
    np.testing.assert_allclose(weld.origin_xyz, (0, 0, 0.5), atol=1e-9)
    assert usd_root_link(rig) == "rig/base"
    # A frame is named by its prim path through any scope on the way.
    assert {"rig/base/Geometry", mast} <= usd_frame_names(rig)
    edges = {(parent, child): pose for parent, child, pose in usd_frame_edges(rig)}
    assert {("rig/base", "rig/base/Geometry"), ("rig/base/Geometry", mast)} <= set(edges)
    assert edges["rig/base/Geometry", mast][2, 3] == pytest.approx(0.5)


_ROOT = "/robots/arm"

_EVERY_KIND_OF_ASSET = """#usda 1.0
(defaultPrim = "arm"
    subLayers = [@lights.usda@ (offset = 7)])
def Xform "arm" (
    append references = @appended.usda@</clip> (offset = 3; scale = 2; customData = {string take = "b"})
    prepend payload = @payload.usda@ (offset = 5)
) {
    asset[] textures = [@albedo.png@, @normal.png@]
}
"""

_EVERY_KIND_OF_FILE = {"lights.usda", "appended.usda", "payload.usda", "albedo.png", "normal.png"}


def _targets(prim: Any, relationship: str) -> list[str]:
    return [str(target) for target in prim.GetRelationship(relationship).GetTargets()]


def test_every_kind_of_asset_is_listed_resolved_and_published(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    assert referenced_asset_paths(_EVERY_KIND_OF_ASSET) == _EVERY_KIND_OF_FILE
    resolved = resolve_usd_assets(_EVERY_KIND_OF_ASSET, _ROOT)
    assert referenced_asset_paths(resolved) == {
        str(Path(_ROOT) / name) for name in _EVERY_KIND_OF_FILE
    }
    published, public = publish_usd_assets(_EVERY_KIND_OF_ASSET)
    assert set(public.values()) == _EVERY_KIND_OF_FILE
    assert referenced_asset_paths(published) == set(public)
    assert not [name for name in public if name.startswith("/") or ".." in name.split("/")]
    assert list(_layer(resolved).subLayerPaths) == [f"{_ROOT}/lights.usda"]

    # A moved arc keeps its timing and custom data.
    for moved in (resolved, published):
        layer = _layer(moved)
        assert layer.subLayerOffsets[0].offset == 7
        prim = layer.GetPrimAtPath("/arm")
        (reference,) = prim.referenceList.appendedItems
        assert reference.assetPath != "appended.usda", "the reference did not move at all"
        assert (reference.layerOffset.offset, reference.layerOffset.scale) == (3, 2)
        assert (dict(reference.customData), str(reference.primPath)) == ({"take": "b"}, "/clip")
        (payload,) = prim.payloadList.prependedItems
        assert payload.assetPath != "payload.usda", "the payload did not move at all"
        assert payload.layerOffset.offset == 5

    # A deleted reference is not a file the layer needs, but it still moves.
    deleting = '#usda 1.0\n\ndef Xform "arm" (\n    delete references = @gone.usda@\n)\n{\n}\n'
    assert referenced_asset_paths(deleting) == set()
    deleting_layer = _layer(resolve_usd_assets(deleting, _ROOT))
    deleted = deleting_layer.GetPrimAtPath("/arm")
    assert [arc.assetPath for arc in deleted.referenceList.deletedItems] == [f"{_ROOT}/gone.usda"]

    # Assets already in place are left alone, without a list editor per arc.
    def opened(spec: Any) -> None:
        pytest.fail(f"opened a list editor on {spec.path}")

    for name in ("referenceList", "payloadList"):
        monkeypatch.setattr(Sdf.PrimSpec, name, property(opened))
    assert resolve_usd_assets(resolved, _ROOT) == resolved


_BOUND_ARM = """#usda 1.0
(defaultPrim = "arm"
    upAxis = "Z")
def Xform "arm" {
    def Xform "base" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "MaterialBindingAPI"]) {
        rel material:binding = </Looks/paint>
        float level.connect = </arm/tip.level>
    }
    def Xform "tip" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]
        prepend references = </arm/base>) {
        float level
    }
    def PhysicsFixedJoint "weld" {
        rel physics:body0 = </arm/base>
        rel physics:body1 = </arm/tip>
    }
}
def Scope "Looks" {
    def Material "paint" {}
}
"""


@pytest.mark.parametrize("namespace", ["left", "fleet/robot0", "arm"])
def test_a_layer_nested_under_a_namespace_names_each_frame_as_on_tf(namespace: str) -> None:
    nested = usd_under_namespace(_BOUND_ARM, namespace)
    stage = _stage(nested)

    under = {f"{namespace}/{frame}" for frame in usd_frame_names(_BOUND_ARM)}
    ancestors = {"/".join(namespace.split("/")[: n + 1]) for n in range(namespace.count("/") + 1)}
    assert usd_frame_names(nested) == under | ancestors
    assert str(stage.GetDefaultPrim().GetPath()) == f"/{namespace.partition('/')[0]}"
    assert stage.GetMetadata("upAxis") == "Z"
    base = stage.GetPrimAtPath(f"/{namespace}/arm/base")
    assert _targets(base, "material:binding") == [f"/{namespace}/Looks/paint"]
    assert [str(c) for c in base.GetAttribute("level").GetConnections()] == [
        f"/{namespace}/arm/tip.level"
    ]
    weld = stage.GetPrimAtPath(f"/{namespace}/arm/weld")
    assert _targets(weld, "physics:body1") == [f"/{namespace}/arm/tip"]
    tip = stage.GetPrimAtPath(f"/{namespace}/arm/tip")
    assert tip.HasAPI(UsdPhysics.RigidBodyAPI), "the internal reference still composes"
    assert not stage.GetPrimAtPath("/arm/base") or namespace == "arm"


def test_a_namespaced_layer_moves_anywhere_and_its_frames_keep_their_tf_ids() -> None:
    assert usd_under_namespace(_BOUND_ARM, "") == _BOUND_ARM
    with pytest.raises(ValueError, match="'2nd' starts with '2'"):
        usd_under_namespace(_BOUND_ARM, "fleet/2nd")

    named = usd_with_frame_names(_BOUND_ARM, "fleet/robot0", {"arm/tip": "fleet/robot0/tip"})
    served = usd_under_namespace(named, "fleet/robot0")
    prims = usd_world_frame_prims(served)
    assert prims["fleet/robot0/tip"] == "fleet/robot0/arm/tip"
    for under in ("room/spot", "left"):
        moved = usd_moved_under(served, "fleet/robot0", under)
        stage = _stage(moved)
        assert not stage.GetPrimAtPath("/fleet")
        assert str(stage.GetDefaultPrim().GetPath()) == f"/{under.partition('/')[0]}"
        weld = stage.GetPrimAtPath(f"/{under}/arm/weld")
        assert _targets(weld, "physics:body1") == [f"/{under}/arm/tip"]
        assert stage.GetPrimAtPath(f"/{under}/arm/tip").HasAPI(UsdPhysics.RigidBodyAPI)
        prims = usd_world_frame_prims(moved)
        assert prims["fleet/robot0/tip"] == f"{under}/arm/tip"
        assert prims["fleet/robot0/arm/base"] == f"{under}/arm/base"
        assert "fleet/robot0/arm/tip" not in prims
