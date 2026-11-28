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

import pytest

# Every body here is read out of a USD layer, and a runner without usd-core cannot read one.
pytest.importorskip("pxr")

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.assets import resolve_usd_assets
from dimos.robot.assets.usd_utils.layers import usd_under_namespace
from dimos.sim2.dimsim.room import ROOM, DimSimRoom
from dimos.sim2.dimsim.scene import stage_bodies
from dimos.sim2.scene import scene_usd
from dimos.sim2.sim import compose_usds

BOT = "dimsim_room/spawns/go2"
BOT_USD = Path(__file__).parent / "assets" / "planar_bot.usda"


def _bodies(usd: UsdModel, exclude: tuple[str, ...] = ()) -> dict[str, dict]:
    from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr

    pxr = load_pxr()
    stage = pxr.Usd.Stage.Open(layer_from(pxr, usd.usd, "a scene", usd.asset_root))
    return {body["name"]: body for body in stage_bodies(pxr, stage, exclude)[0]}


def test_the_room_is_a_scene_of_fixed_pieces_and_a_crate_with_one_spawn_point() -> None:
    assert list(DimSimRoom.mounts) == [BOT]
    bodies = _bodies(scene_usd(ROOM))
    fixed = {name for name, body in bodies.items() if "mass" not in body}
    assert fixed == {
        f"/dimsim_room/Geometry/{piece}"
        for piece in ("floor", "north_wall", "south_wall", "east_wall", "west_wall", "table")
    }
    wall = bodies["/dimsim_room/Geometry/north_wall"]
    assert wall["pos"] == pytest.approx([0.0, 3.05, 0.6])
    assert wall["geoms"][0]["type"] == "box"
    assert wall["geoms"][0]["size"] == pytest.approx([3.1, 0.05, 0.6])
    # The crate is dynamic and addressable by a short name, and no spawn volume is a body.
    crate = bodies["crate"]
    assert crate["mass"] == 2.0
    assert crate["pos"] == pytest.approx([1.5, 0.0, 0.2])
    assert crate["geoms"][0]["size"] == pytest.approx([0.2, 0.2, 0.2])
    assert len(bodies) == 7


def test_a_robot_in_a_composed_room_is_left_out_of_it() -> None:
    bot = UsdModel.from_file(BOT_USD)
    room = resolve_usd_assets(scene_usd(ROOM).usd, str(ROOM.parent))
    composed = compose_usds([(room, None), (usd_under_namespace(bot.usd, BOT), BOT)])
    assert len(_bodies(composed)) == 8, "the bot's box was never mounted in"
    left = _bodies(composed, exclude=(BOT,)).values()
    assert not [geom for body in left for geom in body["geoms"] if f"/{BOT}/" in geom["name"]]
