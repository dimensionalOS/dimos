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

from collections.abc import Iterable
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import threading
from typing import Any

from pydantic import Field
import pytest

from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.usd_utils.frames import usd_frame_names, usd_with_frames
from dimos.robot.assets.usd_utils.from_urdf import urdf_to_usd
from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.sim2 import scene
from dimos.sim2.scene import overlay_usds

# Before importing the sim, whose spawn points register a USD schema on import.
pytest.importorskip("pxr")

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.robot.assets.model import UsdModel
from dimos.sim2.scene import SimScene, SimSceneConfig, SpawnMounts, scene_usd


def _spot(name: str, translate: str, size: str = "1, 1, 2") -> str:
    return f"""
            def Xform "{name}" (
                prepend apiSchemas = ["DimosSpawnAPI"]
            )
            {{
                double3 xformOp:translate = ({translate})
                uniform token[] xformOpOrder = ["xformOp:translate"]

                def Cube "volume"
                {{
                    uniform token purpose = "guide"
                    double size = 1
                    float3 xformOp:scale = ({size})
                    uniform token[] xformOpOrder = ["xformOp:scale"]
                }}
            }}"""


def _room(spawns: str) -> str:
    return f"""#usda 1.0
(
    defaultPrim = "room"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "room" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
)
{{
    def Xform "desk" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    )
    {{
        double3 xformOp:translate = (1, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}

    def Scope "aisle"
    {{{spawns}
    }}
}}
"""


def _scene(path: Path) -> type[SimScene]:
    class _RoomConfig(SimSceneConfig):
        usd: UsdModel = Field(default_factory=lambda: scene_usd(path))

    class _Room(SimScene):
        config: _RoomConfig
        mounts = SpawnMounts(path)

    return _Room


@pytest.fixture
def room(tmp_path: Path) -> type[SimScene]:
    path = tmp_path / "room.usda"
    path.write_text(_room(_spot("spot", "0, 3, 0") + _spot("shelf", "2, 0, 1", "0.3, 0.3, 0.3")))
    return _scene(path)


def test_a_scenes_mounts_are_its_spawn_points(room: type[SimScene]) -> None:
    assert list(room.mounts) == ["room/aisle/shelf", "room/aisle/spot"]
    assert room.mounts.spot.frame == "room/aisle/spot"


def test_a_scene_publishes_only_the_path_to_its_mounts(room: type[SimScene]) -> None:
    scene = room()
    try:
        edges = {(t.frame_id, t.child_frame_id) for t in scene.static_transforms()}
    finally:
        scene.stop()

    # Each spawn point is welded to the rigid body it is under, so that is its tf parent, and is called by its name.
    assert edges == {("room", "spot"), ("room", "shelf")}


def test_a_spawn_point_is_a_frame_where_its_prim_stands(room: type[SimScene]) -> None:
    scene = room()
    try:
        spot = next(t for t in scene.static_transforms() if t.child_frame_id == "spot")
    finally:
        scene.stop()

    assert (spot.translation.x, spot.translation.y, spot.translation.z) == (0.0, 3.0, 0.0)


def test_a_scene_works_its_edges_out_once_and_restamps_them(
    room: type[SimScene], monkeypatch: pytest.MonkeyPatch
) -> None:
    scene = room()
    try:
        worked_out = []
        namespaced = scene._usd_edges_as_namespaced_transforms

        def working_out(*args: Any) -> list[Transform]:
            worked_out.append(1)
            return namespaced(*args)

        monkeypatch.setattr(scene, "_usd_edges_as_namespaced_transforms", working_out)
        stamps = [scene.static_transforms(ts=float(tick)) for tick in range(3)]
    finally:
        scene.stop()

    assert len(worked_out) == 1
    assert [{t.ts for t in edges} for edges in stamps] == [{0.0}, {1.0}, {2.0}]


def test_a_scene_without_spawn_points_is_refused(tmp_path: Path) -> None:
    path = tmp_path / "bare.usda"
    path.write_text(_room(""))
    with pytest.raises(ValueError, match="needs spawn points"):
        _scene(path)()


def test_a_spawn_point_without_a_volume_is_refused(tmp_path: Path) -> None:
    path = tmp_path / "loose.usda"
    path.write_text(
        _room(
            '\n            def Xform "spot" (\n                prepend apiSchemas = ["DimosSpawnAPI"]\n            )\n            {\n            }'
        )
    )
    with pytest.raises(ValueError, match="has no volume"):
        _scene(path)()


pytest.importorskip("pxr")


def test_threads_writing_the_same_layer_at_once_all_get_it() -> None:
    writers, turns = 8, 50
    start = threading.Barrier(writers)

    def write(turn: int) -> Path:
        start.wait()
        return scene.layer_file(f"#usda 1.0\n# {turn}\n", called="race")

    with ThreadPoolExecutor(writers) as pool:
        for turn in range(turns):
            layers = list(pool.map(write, [turn] * writers))
            assert len(set(layers)) == 1
            assert layers[0].read_text() == f"#usda 1.0\n# {turn}\n"


def test_a_mesh_the_layer_names_that_is_a_git_lfs_pointer_is_fetched_before_it_is_read(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from dimos.robot.assets.model import UsdModel
    from dimos.robot.assets.usd_utils import lfs
    from dimos.utils.data import _is_lfs_pointer_file

    meshes = tmp_path / "robot" / "meshes"
    meshes.mkdir(parents=True)
    (meshes / "arm.usdc").write_text("version https://git-lfs.github.com/spec/v1\noid sha256:0\n")
    (meshes / "leg.usdc").write_bytes(b"PXR-USDC already fetched")
    robot = tmp_path / "robot" / "robot.usda"
    references = "".join(
        f'def Xform "{name}" (references = @./meshes/{name}.usdc@) {{}}\n'
        for name in ("arm", "leg")
    )
    robot.write_text(f"#usda 1.0\n{references}")
    pulled: list[list[Path]] = []

    def pull(paths: Iterable[Path]) -> None:
        pulled.append(sorted(path for path in paths if _is_lfs_pointer_file(path)))
        for path in pulled[-1]:
            path.write_bytes(b"PXR-USDC fetched")

    monkeypatch.setattr(lfs, "pull_lfs_files", pull)

    scene.usd_layer(UsdModel.from_file(robot))

    assert pulled == [[meshes / "arm.usdc"]]
    assert (meshes / "arm.usdc").read_bytes() == b"PXR-USDC fetched"


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
  <link name="arm_1"><visual><geometry><sphere radius="0.05"/></geometry></visual></link>
  <link name="spin"/>
  <joint name="base_to_mount" type="fixed">
    <parent link="base_link"/><child link="mount"/>
    <origin xyz="0.1 0 0.1" rpy="0 0 0.3"/>
  </joint>
  <joint name="mount_to_arm" type="revolute">
    <parent link="mount"/><child link="arm_1"/>
    <origin xyz="0 0 0.02" rpy="0 0.2 0"/><axis xyz="0.6 0.8 0"/>
    <limit lower="-1.5707963267948966" upper="1.5707963267948966" effort="10" velocity="1"/>
  </joint>
  <joint name="arm_to_spin" type="continuous">
    <parent link="arm_1"/><child link="spin"/>
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
        if prim.GetName() == link:
            return prim
    raise AssertionError(f"no prim for link {link!r}")


# The toy's frame an arm mounts on in these tests, and so the arm's namespace.
_MOUNT = "toy/base_link/mount"


def _mounted_arm(toy: Any, mount: str = _MOUNT) -> tuple[str, str]:
    """A second toy as a part provides it on ``mount``: its prims under that frame."""
    return usd_under_namespace(urdf_to_usd(_renamed(toy, "arm_sub")), mount), mount


def _renamed(model: Any, name: str) -> Any:
    """The same description under another robot name, so its top prim is another prim."""
    return RobotModel.from_urdf_string(model.xml.replace('name="toy"', f'name="{name}"', 1)).load()


def test_a_mounted_part_overlays_at_its_own_prim_paths(toy: Any) -> None:
    stage = _stage(overlay_usds(urdf_to_usd(toy), [_mounted_arm(toy)]))

    assert stage.GetPrimAtPath(f"/{_MOUNT}/arm_sub/base_link")
    assert stage.GetDefaultPrim().GetName() == "toy"


def test_a_mounted_part_keeps_its_joints_pointing_at_its_own_links(toy: Any) -> None:
    stage = _stage(overlay_usds(urdf_to_usd(toy), [_mounted_arm(toy)]))
    joint = stage.GetPrimAtPath(f"/{_MOUNT}/arm_sub/joints/base_to_mount")

    assert joint
    targets = joint.GetRelationship("physics:body0").GetTargets()
    assert str(targets[0]) == f"/{_MOUNT}/arm_sub/base_link"


def test_a_mounted_part_is_welded_to_its_mount_with_no_offset(toy: Any) -> None:
    from pxr import Gf

    stage = _stage(overlay_usds(urdf_to_usd(toy), [_mounted_arm(toy)]))
    welds = [prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsJoint"]

    assert len(welds) == 1
    weld = welds[0]
    assert weld.GetDisplayName() == "base_link_mount"
    assert str(weld.GetRelationship("physics:body0").GetTargets()[0]) == f"/{_MOUNT}"
    assert str(weld.GetRelationship("physics:body1").GetTargets()[0]) == (
        f"/{_MOUNT}/arm_sub/base_link"
    )
    for attribute in ("physics:localPos0", "physics:localPos1"):
        assert weld.GetAttribute(attribute).Get() == Gf.Vec3f(0.0, 0.0, 0.0)
    for attribute in ("physics:localRot0", "physics:localRot1"):
        assert weld.GetAttribute(attribute).Get() == Gf.Quatf(1.0, 0.0, 0.0, 0.0)


def test_a_mounted_part_without_a_rigid_body_is_not_welded(toy: Any) -> None:
    marker = '#usda 1.0\n(\n    defaultPrim = "marker"\n)\n\ndef Xform "marker"\n{\n}\n'
    mounted = (usd_under_namespace(marker, _MOUNT), _MOUNT)

    stage = _stage(overlay_usds(urdf_to_usd(toy), [mounted]))

    assert stage.GetPrimAtPath(f"/{_MOUNT}/marker")
    assert not [prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsJoint"]


def test_only_the_root_s_physics_scene_survives(toy: Any) -> None:
    from pxr import UsdPhysics

    def with_scene(usd: str) -> str:
        stage = _stage(usd)
        UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        return str(stage.GetRootLayer().ExportToString())

    arm = usd_under_namespace(with_scene(urdf_to_usd(_renamed(toy, "arm_sub"))), _MOUNT)
    stage = _stage(overlay_usds(with_scene(urdf_to_usd(toy)), [(arm, _MOUNT)]))

    scenes = [str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    assert scenes == ["/PhysicsScene"]


def test_a_mounted_part_s_frames_are_its_own_under_the_mount(toy: Any) -> None:
    """Every frame of the overlay is a frame of the root or of the mounted part, already under its mount."""
    root = urdf_to_usd(toy)
    arm, mount = _mounted_arm(toy)

    composed = usd_frame_names(overlay_usds(root, [(arm, mount)]))

    assert composed == usd_frame_names(root) | usd_frame_names(arm)


def test_a_frame_can_be_added_to_a_scene_and_mounted_at(toy: Any) -> None:
    grown = usd_with_frames(
        urdf_to_usd(toy), {"spot": ("toy/base_link", (1.0, 2.0, 0.0), (0.0, 0.0, 0.0))}
    )
    stage = _stage(overlay_usds(grown, [_mounted_arm(toy, "toy/base_link/spot")]))

    assert _prim_by_link(stage, "spot"), "the frame is a link, so a part can mount on it"
    assert stage.GetPrimAtPath("/toy/base_link/spot/arm_sub/base_link")


_SCENE = """#usda 1.0
(
    defaultPrim = "room"
)

def Xform "room"
{
}

def Material "Materials"
{
    def Material "paint"
    {
        custom string dimos_owner = "room"
    }
}
"""


_PAINTED_ARM = """#usda 1.0
(
    defaultPrim = "arm"
)

def Xform "arm"
{
    def Mesh "shell" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {
        rel material:binding = </Materials/paint>
        rel material:binding:physics = </Materials/rubber>
    }

    def Xform "looks" (
        prepend apiSchemas = ["MaterialBindingAPI", "CollectionAPI:trim"]
    )
    {
        prepend rel collection:trim:includes = </Parts/trim>
        rel material:binding:collection:trim = [<.collection:trim>, </Materials/chrome>]
    }
}

def Scope "Materials"
{
    def Material "paint"
    {
        custom string dimos_owner = "arm"
    }

    def Material "rubber"
    {
    }

    def Material "chrome"
    {
    }
}

def Xform "Parts"
{
    def Xform "trim"
    {
    }
}
"""


def _mounted_in_the_room(usd: str) -> Any:
    """The scene with ``usd`` mounted on its room, as a part provides it: its prims under that frame."""
    return _stage(overlay_usds(_SCENE, [(usd_under_namespace(usd, "room"), "room")]))


def _targets(stage: Any, prim_path: str, relationship: str) -> list[Any]:
    return list(stage.GetPrimAtPath(prim_path).GetRelationship(relationship).GetTargets())


def test_a_mounted_mesh_keeps_the_material_bound_beside_its_default_prim() -> None:
    stage = _mounted_in_the_room(_PAINTED_ARM)

    (paint,) = _targets(stage, "/room/arm/shell", "material:binding")
    assert str(paint) != "/Materials/paint", "the arm is wearing the room's paint"
    assert stage.GetPrimAtPath(paint).GetAttribute("dimos_owner").Get() == "arm"
    assert stage.GetPrimAtPath("/Materials/paint").GetAttribute("dimos_owner").Get() == "room"


def test_a_mounted_physics_binding_follows_it_too() -> None:
    stage = _mounted_in_the_room(_PAINTED_ARM)

    (rubber,) = _targets(stage, "/room/arm/shell", "material:binding:physics")
    assert stage.GetPrimAtPath(rubber), f"the physics material at {rubber} was left behind"


def test_a_mounted_collection_binding_keeps_its_material_and_members() -> None:
    stage = _mounted_in_the_room(_PAINTED_ARM)

    collection, chrome = _targets(stage, "/room/arm/looks", "material:binding:collection:trim")
    assert str(collection) == "/room/arm/looks.collection:trim"
    assert stage.GetPrimAtPath(chrome), f"the chrome material at {chrome} was left behind"
    (trim,) = _targets(stage, "/room/arm/looks", "collection:trim:includes")
    assert stage.GetPrimAtPath(trim), f"the collection member at {trim} was left behind"


def test_two_bindings_to_one_material_beside_the_default_prim_stay_one() -> None:
    twice = _PAINTED_ARM.replace("</Materials/rubber>", "</Materials/paint>")
    stage = _mounted_in_the_room(twice)

    assert _targets(stage, "/room/arm/shell", "material:binding") == _targets(
        stage, "/room/arm/shell", "material:binding:physics"
    )


def test_a_moved_material_does_not_swallow_one_its_name_now_shadows() -> None:
    # /Looks/paint and /paint both move under the namespace, and neither may land on the other.
    arm = """#usda 1.0
(
    defaultPrim = "arm"
)

def Xform "arm"
{
    def Mesh "shell" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {
        rel material:binding = </Looks/paint>
        rel material:binding:physics = </paint>
    }
}

def Scope "Looks"
{
    def Material "paint"
    {
        custom string dimos_owner = "looks"
    }
}

def Material "paint"
{
    custom string dimos_owner = "root"
}
"""
    stage = _mounted_in_the_room(arm)

    (looks,) = _targets(stage, "/room/arm/shell", "material:binding")
    (root,) = _targets(stage, "/room/arm/shell", "material:binding:physics")
    assert looks != root
    assert stage.GetPrimAtPath(looks).GetAttribute("dimos_owner").Get() == "looks"
    assert stage.GetPrimAtPath(root).GetAttribute("dimos_owner").Get() == "root"


# Before importing the sim, whose spawn points register a USD schema on import.
pytest.importorskip("pxr")

from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.sim2.scene import (
    LOGISTICS,
    scene_spawn_points,
    scene_usd,
    spawn_frames_in_file,
    spawn_volume_paths,
    toy_robot,
)
from dimos.sim2.sim import compose_usds
from dimos.simulation.scenes.office_555.room import SCENE

_SPOT = "logistics/spawns/go2"


def _logistics_scaled(volume_scale: str) -> str:
    return LOGISTICS.read_text().replace(
        "float3 xformOp:scale = (1.2, 1.2, 1)", f"float3 xformOp:scale = ({volume_scale})"
    )


def _toy_on_spot() -> str:
    """The toy as it serves itself on the spot, its base moved from where its own USD puts it (9, 9, 9) to the spot."""
    return usd_under_namespace(toy_robot().usd.replace("(9, 9, 9)", "(0, 0, 0)"), _SPOT)


def test_a_robot_that_fits_its_spawn_point_is_placed() -> None:
    world = compose_usds([(scene_usd(LOGISTICS).usd, None), (_toy_on_spot(), _SPOT)])
    assert f"{_SPOT}/{toy_robot().root_link}" in world.frame_names()


def test_a_robot_is_measured_where_it_stands_on_its_spawn_point() -> None:
    """The toy's own USD puts its base 9 m off its mount, outside the spot."""
    with pytest.raises(ValueError, match="too big for the spawn points"):
        compose_usds(
            [(scene_usd(LOGISTICS).usd, None), (usd_under_namespace(toy_robot().usd, _SPOT), _SPOT)]
        )


def test_a_robot_too_big_for_its_spawn_point_is_refused(tmp_path) -> None:
    small = tmp_path / "logistics.usda"
    small.write_text(_logistics_scaled("0.05, 0.05, 0.05"))
    room = scene_usd(small).usd
    with pytest.raises(ValueError, match=rf"(?s)too big for the spawn points.*{_SPOT}"):
        compose_usds([(room, None), (_toy_on_spot(), _SPOT)])


# Office 555's scene is LFS, over the hosted runner's pull cap.
@pytest.mark.self_hosted
def test_office_555_has_spawn_points_on_both_floors_and_for_objects() -> None:
    names = {frame.rpartition("/")[2] for frame in spawn_frames_in_file(SCENE)}
    assert {"main", "left", "right", "overview"} <= names
    assert len([n for n in names if n.startswith("ground_") and "_on_" not in n]) >= 15
    assert len([n for n in names if n.startswith("upper_") and "_on_" not in n]) >= 10
    assert len([n for n in names if "_on_" in n]) >= 10


# Office 555's scene is LFS, over the hosted runner's pull cap.
@pytest.mark.self_hosted
def test_office_555_floor_spots_stand_on_its_two_floors() -> None:
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(SCENE))
    cache = UsdGeom.XformCache()
    floors = {
        round(
            cache.GetLocalToWorldTransform(
                stage.GetPrimAtPath(f"/{point.frame}")
            ).ExtractTranslation()[2],
            1,
        )
        for point in scene_spawn_points(SCENE.read_text(), SCENE.parent)
        if point.name.startswith(("ground_", "upper_")) and "_on_" not in point.name
    }
    assert floors == {0.0, 4.7}


def test_a_spawn_points_volume_is_found_without_composing_its_layer() -> None:
    assert spawn_volume_paths(LOGISTICS) == {f"/{_SPOT}/volume"}
