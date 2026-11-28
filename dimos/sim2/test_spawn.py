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

import pytest

# Before importing the sim, whose spawn points register a USD schema on import.
pytest.importorskip("pxr")

from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.sim2.assets.logistics import LOGISTICS
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.scene import scene_usd
from dimos.sim2.sim import compose_usds
from dimos.sim2.spawn import scene_spawn_points, spawn_frames_in_file, spawn_volume_paths
from dimos.simulation.scenes.office_555.room import SCENE

_SPOT = "logistics/spawns/go2"


def _room(volume_scale: str) -> str:
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
    small.write_text(_room("0.05, 0.05, 0.05"))
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
