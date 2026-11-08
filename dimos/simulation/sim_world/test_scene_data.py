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


"""The sim's world as the files the controller draws it from."""

from __future__ import annotations

import json

import mujoco
import numpy as np
import pytest

from dimos.simulation.sim_world.body import COLLISION_GROUP
from dimos.simulation.sim_world.scene_data import SCENE_DATA_FORMAT, SCENE_FILE, scene_files


def _scene() -> tuple[mujoco.MjModel, mujoco.MjData, int]:
    """A room with a wall, a wall's contact shape, and a robot standing in it."""
    spec = mujoco.MjSpec()
    wall = spec.worldbody.add_body()
    wall.name = "wall"
    drawn = wall.add_geom()
    drawn.name = "wall_visual"
    drawn.type = mujoco.mjtGeom.mjGEOM_BOX
    drawn.size = [1.0, 0.1, 1.2]
    contact = wall.add_geom()
    contact.name = "wall_contact"
    contact.type = mujoco.mjtGeom.mjGEOM_BOX
    contact.size = [1.0, 0.1, 1.2]
    contact.group = COLLISION_GROUP

    robot = spec.worldbody.add_body()
    robot.name = "base"
    robot.add_freejoint()
    robot.add_geom().size = [0.2, 0.2, 0.2]
    link = robot.add_body()
    link.name = "arm"
    link.add_geom().size = [0.1, 0.1, 0.1]

    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")


def _index(files: dict[str, bytes]) -> dict[str, object]:
    index: dict[str, object] = json.loads(files[SCENE_FILE])
    return index


def test_it_names_the_world_and_leaves_the_robot_to_its_own_description() -> None:
    """The robot is drawn from its USD, so sending it twice would draw it twice."""
    model, data, robot = _scene()
    index = _index(scene_files(model, data, "id", "odom", robot))

    assert index["format"] == SCENE_DATA_FORMAT
    assert index["frame"] == "odom"
    assert [body["name"] for body in index["bodies"]] == ["wall"]  # type: ignore[index,union-attr]


def test_contact_geometry_is_not_sent_to_be_drawn() -> None:
    """Group 3 is what the robot bumps into, and MuJoCo's own viewer hides it.

    A generated scene has none of it so this made no difference, but a converted room carries
    its contact decomposition there in bulk: office_555 has 3083 convex pieces standing behind
    464 visual meshes, and drawing those is both the wrong picture and seven times the payload.
    """
    model, data, robot = _scene()
    index = _index(scene_files(model, data, "id", "odom", robot))

    drawn = [geom["name"] for body in index["bodies"] for geom in body["geoms"]]  # type: ignore[index,union-attr]
    assert drawn == ["wall_visual"]


def test_a_mesh_comes_with_the_stl_the_index_names() -> None:
    """The index is useless on its own: every file it names is asked for by that name."""
    spec = mujoco.MjSpec()
    mesh = spec.add_mesh()
    mesh.name = "lump"
    mesh.uservert = [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1]
    body = spec.worldbody.add_body()
    body.name = "prop"
    geom = body.add_geom()
    geom.type = mujoco.mjtGeom.mjGEOM_MESH
    geom.meshname = "lump"
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    files = scene_files(model, data, "id", "odom", -1)
    index = _index(files)
    named = index["meshes"]["lump"]["file"]  # type: ignore[index,call-overload]

    assert named in files, "the index named a file that was not sent"
    assert np.frombuffer(files[named][80:84], dtype="<u4")[0] == 4, "not four triangles of STL"


@pytest.mark.parametrize("kind", ["box", "sphere", "capsule", "cylinder", "plane"])
def test_the_primitive_shapes_survive_the_trip(kind: str) -> None:
    """Each is a shape the controller builds itself, so an unmapped type is a missing object."""
    spec = mujoco.MjSpec()
    body = spec.worldbody.add_body()
    body.name = kind
    geom = body.add_geom()
    geom.type = getattr(mujoco.mjtGeom, f"mjGEOM_{kind.upper()}")
    geom.size = [0.3, 0.3, 0.3]
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    index = _index(scene_files(model, data, "id", "odom", -1))
    assert [geom["type"] for geom in index["bodies"][0]["geoms"]] == [kind]  # type: ignore[index,union-attr]
