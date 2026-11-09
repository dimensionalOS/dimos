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

"""What a USD scene comes to in Rapier: the room fixed, the props dynamic, the robots left out."""

import pytest

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd import graft_usd, prefix_usd_frames, resolve_usd_assets
from dimos.sim2.assets.logistics import LOGISTICS
from dimos.sim2.assets.logistics_props import LOGISTICS_PROPS
from dimos.sim2.assets.toy import toy_robot
from dimos.sim2.dimsim.scene import dimsim_bodies


def _by_name(usd: UsdModel, exclude: tuple[str, ...] = ()) -> dict:
    bodies, _meshes = dimsim_bodies(usd, exclude=exclude)
    return {body["name"]: body for body in bodies}


def test_the_room_is_every_piece_of_it_and_all_of_it_fixed():
    """A floor and eight boxes, none of them a rigid body, so Rapier holds every one still."""
    bodies = _by_name(UsdModel.from_file(LOGISTICS))
    assert len(bodies) == 9
    assert all("mass" not in body for body in bodies.values())
    wall = bodies["/logistics/Geometry/north_wall"]
    assert wall["pos"] == pytest.approx([0.0, 5.0, 1.5])
    # A Cube's size is 2 and its scale is the half extents, which is what a cuboid collider takes.
    assert wall["geoms"][0]["type"] == "box"
    assert wall["geoms"][0]["size"] == pytest.approx([6.0, 0.1, 1.5])
    floor = bodies["/logistics/Geometry/floor"]
    assert floor["geoms"][0]["type"] == "plane"
    assert floor["geoms"][0]["size"][:2] == pytest.approx([10.0, 10.0])


def test_the_props_are_dynamic_and_addressable_by_a_short_name():
    """The room, plus three bodies with an authored mass: what an action can push and a frame reports."""
    bodies = _by_name(UsdModel.from_file(LOGISTICS_PROPS))
    assert len(bodies) == 12
    assert {name for name, body in bodies.items() if "mass" in body} == {
        "box_small",
        "box_tall",
        "ball",
    }
    assert bodies["box_tall"]["mass"] == 3.0
    assert bodies["box_tall"]["pos"] == pytest.approx([2.4, 0.3, 0.25])
    assert bodies["box_tall"]["geoms"][0]["size"] == pytest.approx([0.2, 0.2, 0.25])
    assert bodies["ball"]["geoms"][0] == {
        "name": "/logistics/Props/ball",
        "pos": pytest.approx([0.0, 0.0, 0.0]),
        "quat": [0.0, 0.0, 0.0, 1.0],
        "type": "sphere",
        "size": pytest.approx([0.2]),
    }
    # The quat world.ts reads is x, y, z, w, so an unrotated body ends in the 1.
    assert bodies["box_small"]["quat"] == [0.0, 0.0, 0.0, 1.0]


def test_a_robot_in_a_composed_scene_is_left_out_of_it():
    """Because this engine replaces a whole robot with one kinematic box of its own.

    A scene arrives from `get_scene` with the robots already grafted into it, so every prim of
    one has to go -- otherwise the dog is in the room twice, once as scenery that cannot move.
    """
    room = UsdModel.from_file(LOGISTICS_PROPS).with_frames(
        {"dog": ("logistics", (0.0, 0.0, 0.3), (0.0, 0.0, 0.0))}
    )
    # Resolved first, as a manager does on the way out: the props layer names the room relative to
    # itself, and a layer that has left its directory behind has nothing for that to resolve
    # against.
    composed = UsdModel.from_layer(
        graft_usd(
            resolve_usd_assets(room.usd, str(LOGISTICS_PROPS.parent)),
            {"dog": prefix_usd_frames(toy_robot().usd, "dog")},
        )
    )
    whole, _meshes = dimsim_bodies(composed)
    assert len(whole) == 14, "the dog's two links were never grafted in"
    bodies = _by_name(composed, exclude=("dog",))
    assert len(bodies) == 12
    # By the geoms, which keep their prim paths: a body of the dog's is short-named by then.
    assert not [
        geom for body in bodies.values() for geom in body["geoms"] if "/dog/" in geom["name"]
    ]
