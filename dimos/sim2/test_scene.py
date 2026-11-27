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

from pydantic import Field
import pytest

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.robot.assets.model import UsdModel
from dimos.sim2.scene import SimScene, SimSceneConfig, SpawnMounts, scene_usd

pytest.importorskip("pxr")


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

    # Each spawn point is welded to the rigid body it is under, so that is its tf parent.
    assert edges == {("room", "room/aisle/spot"), ("room", "room/aisle/shelf")}


def test_a_spawn_point_is_a_frame_where_its_prim_stands(room: type[SimScene]) -> None:
    scene = room()
    try:
        spot = next(t for t in scene.static_transforms() if t.child_frame_id == "room/aisle/spot")
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
