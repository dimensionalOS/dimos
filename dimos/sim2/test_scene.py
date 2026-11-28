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
from unittest.mock import Mock

from pydantic import Field
import pytest

# Before importing the sim, whose spawn points register a USD schema on import.
pytest.importorskip("pxr")

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from dimos.robot.assets.model import RobotModel, UsdModel
from dimos.robot.assets.usd_utils import lfs
from dimos.robot.assets.usd_utils.frames import usd_frame_names, usd_with_frames
from dimos.robot.assets.usd_utils.from_urdf import urdf_to_usd
from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.sim2 import scene
from dimos.sim2.scene import (
    LOGISTICS,
    SimScene,
    SimSceneConfig,
    SpawnMounts,
    overlay_usds,
    scene_spawn_points,
    scene_usd,
    spawn_frames_in_file,
    spawn_volume_paths,
    toy_robot,
)
from dimos.sim2.sim import compose_usds
from dimos.simulation.scenes.office_555.room import SCENE
from dimos.utils.data import _is_lfs_pointer_file


def _spot(name: str, translate: str, size: str = "1, 1, 2") -> str:
    return f"""
        def Xform "{name}" (prepend apiSchemas = ["DimosSpawnAPI"]) {{
            double3 xformOp:translate = ({translate})
            uniform token[] xformOpOrder = ["xformOp:translate"]
            def Cube "volume" {{
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
def Xform "room" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {{
    def Xform "desk" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {{
        double3 xformOp:translate = (1, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}
    def Scope "aisle" {{{spawns}
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


def test_a_scenes_spawn_points_are_its_mounts_and_its_only_published_frames(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "room.usda"
    path.write_text(_room(_spot("spot", "0, 3, 0") + _spot("shelf", "2, 0, 1", "0.3, 0.3, 0.3")))
    room = _scene(path)
    assert list(room.mounts) == ["room/aisle/shelf", "room/aisle/spot"]
    assert room.mounts.spot.frame == "room/aisle/spot"

    running = room()
    try:
        edges = "_usd_edges_as_namespaced_transforms"
        working_out = Mock(wraps=getattr(running, edges))
        monkeypatch.setattr(running, edges, working_out)
        stamps = [running.static_transforms(ts=float(tick)) for tick in range(3)]
    finally:
        running.stop()

    # Each spawn point is welded to the rigid body it is under, so that is its tf parent.
    assert {(t.frame_id, t.child_frame_id) for t in stamps[0]} == {
        ("room", "spot"),
        ("room", "shelf"),
    }
    spot = next(t for t in stamps[0] if t.child_frame_id == "spot")
    assert (spot.translation.x, spot.translation.y, spot.translation.z) == (0.0, 3.0, 0.0)
    # The edges are worked out once and only restamped after.
    assert working_out.call_count == 1
    assert [{t.ts for t in edges} for edges in stamps] == [{0.0}, {1.0}, {2.0}]


@pytest.mark.parametrize(
    "spawns, refusal",
    [
        ("", "needs spawn points"),
        ('def Xform "spot" (prepend apiSchemas = ["DimosSpawnAPI"]) {}', "has no volume"),
    ],
    ids=["no_spawn_points", "spawn_point_without_volume"],
)
def test_a_scene_with_unusable_spawn_points_is_refused(
    tmp_path: Path, spawns: str, refusal: str
) -> None:
    path = tmp_path / "room.usda"
    path.write_text(_room(spawns))
    with pytest.raises(ValueError, match=refusal):
        _scene(path)()


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


_URDF = """<?xml version="1.0"?>
<robot name="toy">
  <link name="base_link"><visual><geometry><box size="0.4 0.3 0.1"/></geometry></visual></link>
  <link name="mount"/>
  <joint name="base_to_mount" type="fixed">
    <parent link="base_link"/><child link="mount"/>
    <origin xyz="0.1 0 0.1" rpy="0 0 0.3"/>
  </joint>
</robot>
"""

# The toy's frame an arm mounts on in these tests, and so the arm's namespace.
_MOUNT = "toy/base_link/mount"


def _stage(usd: str) -> Any:
    layer = Sdf.Layer.CreateAnonymous(".usda")
    assert layer.ImportFromString(usd)
    return Usd.Stage.Open(layer)


def _toy_usd(name: str = "toy", physics_scene: bool = False) -> str:
    """The toy as USD, under another robot name so its top prim is another prim."""
    toy = RobotModel.from_urdf_string(_URDF.replace('name="toy"', f'name="{name}"', 1)).load()
    if not physics_scene:
        return urdf_to_usd(toy)
    stage = _stage(urdf_to_usd(toy))
    UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    return str(stage.GetRootLayer().ExportToString())


def _welds(stage: Any) -> list[Any]:
    return [prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsJoint"]


def test_a_mounted_part_overlays_under_its_mount_welded_with_no_offset_if_it_has_a_body() -> None:
    root = _toy_usd(physics_scene=True)
    arm = usd_under_namespace(_toy_usd("arm_sub", physics_scene=True), _MOUNT)
    composed = overlay_usds(root, [(arm, _MOUNT)])
    stage = _stage(composed)

    assert stage.GetPrimAtPath(f"/{_MOUNT}/arm_sub/base_link")
    assert stage.GetDefaultPrim().GetName() == "toy"
    # Its joints still point at its own links.
    joint = stage.GetPrimAtPath(f"/{_MOUNT}/arm_sub/joints/base_to_mount")
    assert str(joint.GetRelationship("physics:body0").GetTargets()[0]) == (
        f"/{_MOUNT}/arm_sub/base_link"
    )
    (weld,) = _welds(stage)
    assert weld.GetDisplayName() == "base_link_mount"
    assert str(weld.GetRelationship("physics:body0").GetTargets()[0]) == f"/{_MOUNT}"
    assert str(weld.GetRelationship("physics:body1").GetTargets()[0]) == (
        f"/{_MOUNT}/arm_sub/base_link"
    )
    for side in ("0", "1"):
        assert weld.GetAttribute(f"physics:localPos{side}").Get() == Gf.Vec3f(0.0, 0.0, 0.0)
        assert weld.GetAttribute(f"physics:localRot{side}").Get() == Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    # Only the root's physics scene survives.
    scenes = [str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    assert scenes == ["/PhysicsScene"]
    assert usd_frame_names(composed) == usd_frame_names(root) | usd_frame_names(arm)

    # A part mounts on an added frame, and one without a rigid body is not welded.
    grown = usd_with_frames(
        _toy_usd(), {"spot": ("toy/base_link", (1.0, 2.0, 0.0), (0.0, 0.0, 0.0))}
    )
    arm = usd_under_namespace(_toy_usd("arm_sub"), "toy/base_link/spot")
    stage = _stage(overlay_usds(grown, [(arm, "toy/base_link/spot")]))
    assert stage.GetPrimAtPath("/toy/base_link/spot").HasAPI(UsdPhysics.RigidBodyAPI)
    assert stage.GetPrimAtPath("/toy/base_link/spot/arm_sub/base_link")

    marker = '#usda 1.0\n(defaultPrim = "marker")\ndef Xform "marker" {}\n'
    stage = _stage(overlay_usds(_toy_usd(), [(usd_under_namespace(marker, _MOUNT), _MOUNT)]))
    assert stage.GetPrimAtPath(f"/{_MOUNT}/marker")
    assert not _welds(stage)


_SCENE = """#usda 1.0
(defaultPrim = "room")
def Xform "room" {}
def Material "Materials" {
    def Material "paint" {
        custom string dimos_owner = "room"
    }
}
"""

_PAINTED_ARM = """#usda 1.0
(defaultPrim = "arm")
def Xform "arm" {
    def Mesh "shell" (prepend apiSchemas = ["MaterialBindingAPI"]) {
        rel material:binding = </Materials/paint>
        rel material:binding:physics = </Materials/rubber>
    }
    def Mesh "coat" (prepend apiSchemas = ["MaterialBindingAPI"]) {
        rel material:binding = </Materials/paint>
        rel material:binding:physics = </Materials/paint>
    }
    def Mesh "shadowed" (prepend apiSchemas = ["MaterialBindingAPI"]) {
        rel material:binding = </Looks/paint>
        rel material:binding:physics = </paint>
    }
    def Xform "looks" (prepend apiSchemas = ["MaterialBindingAPI", "CollectionAPI:trim"]) {
        prepend rel collection:trim:includes = </Parts/trim>
        rel material:binding:collection:trim = [<.collection:trim>, </Materials/chrome>]
    }
}
def Scope "Materials" {
    def Material "paint" {
        custom string dimos_owner = "arm"
    }
    def Material "rubber" {}
    def Material "chrome" {}
}
def Xform "Parts" {
    def Xform "trim" {}
}
def Scope "Looks" {
    def Material "paint" {
        custom string dimos_owner = "looks"
    }
}
def Material "paint" {
    custom string dimos_owner = "root"
}
"""


def _mounted_in_the_room(usd: str) -> Any:
    return _stage(overlay_usds(_SCENE, [(usd_under_namespace(usd, "room"), "room")]))


def _targets(stage: Any, prim_path: str, relationship: str) -> list[Any]:
    return list(stage.GetPrimAtPath(prim_path).GetRelationship(relationship).GetTargets())


def _owner(stage: Any, path: Any) -> str:
    return stage.GetPrimAtPath(path).GetAttribute("dimos_owner").Get()


def test_a_mounted_part_keeps_every_material_and_member_it_binds_beside_its_default_prim() -> None:
    stage = _mounted_in_the_room(_PAINTED_ARM)

    (paint,) = _targets(stage, "/room/arm/shell", "material:binding")
    assert str(paint) != "/Materials/paint", "the arm is wearing the room's paint"
    assert _owner(stage, paint) == "arm"
    assert _owner(stage, "/Materials/paint") == "room"
    (rubber,) = _targets(stage, "/room/arm/shell", "material:binding:physics")
    assert stage.GetPrimAtPath(rubber), f"the physics material at {rubber} was left behind"
    collection, chrome = _targets(stage, "/room/arm/looks", "material:binding:collection:trim")
    assert str(collection) == "/room/arm/looks.collection:trim"
    assert stage.GetPrimAtPath(chrome), f"the chrome material at {chrome} was left behind"
    (trim,) = _targets(stage, "/room/arm/looks", "collection:trim:includes")
    assert stage.GetPrimAtPath(trim), f"the collection member at {trim} was left behind"
    # Two bindings to one material stay one, and /Looks/paint and /paint never land on each other.
    for mesh, owners in (("coat", ["arm", "arm"]), ("shadowed", ["looks", "root"])):
        bound = [
            target
            for relationship in ("material:binding", "material:binding:physics")
            for target in _targets(stage, f"/room/arm/{mesh}", relationship)
        ]
        assert [_owner(stage, path) for path in bound] == owners
        assert (bound[0] == bound[1]) == (mesh == "coat")


_SPOT = "logistics/spawns/go2"


@pytest.mark.parametrize(
    "volume_scale, toy_at, refusal",
    [
        ("1.2, 1.2, 1", "(0, 0, 0)", None),
        # The toy's own USD puts its base 9 m off its mount, outside the spot.
        ("1.2, 1.2, 1", "(9, 9, 9)", "too big for the spawn points"),
        ("0.05, 0.05, 0.05", "(0, 0, 0)", rf"(?s)too big for the spawn points.*{_SPOT}"),
    ],
    ids=["fits", "measured_where_it_stands", "too_big"],
)
def test_a_robot_is_placed_only_where_it_fits_its_spawn_point(
    tmp_path: Path, volume_scale: str, toy_at: str, refusal: str | None
) -> None:
    logistics = tmp_path / "logistics.usda"
    logistics.write_text(
        LOGISTICS.read_text().replace(
            "float3 xformOp:scale = (1.2, 1.2, 1)", f"float3 xformOp:scale = ({volume_scale})"
        )
    )
    assert spawn_volume_paths(logistics) == {f"/{_SPOT}/volume"}
    toy = usd_under_namespace(toy_robot().usd.replace("(9, 9, 9)", toy_at), _SPOT)
    usds = [(scene_usd(logistics).usd, None), (toy, _SPOT)]
    if refusal is None:
        assert f"{_SPOT}/{toy_robot().root_link}" in compose_usds(usds).frame_names()
    else:
        with pytest.raises(ValueError, match=refusal):
            compose_usds(usds)


# Office 555's scene is LFS, over the hosted runner's pull cap.
@pytest.mark.self_hosted
def test_office_555_has_spawn_points_on_both_floors_and_for_objects() -> None:
    names = {frame.rpartition("/")[2] for frame in spawn_frames_in_file(SCENE)}
    assert {"main", "left", "right", "overview"} <= names
    assert len([n for n in names if n.startswith("ground_") and "_on_" not in n]) >= 15
    assert len([n for n in names if n.startswith("upper_") and "_on_" not in n]) >= 10
    assert len([n for n in names if "_on_" in n]) >= 10

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
