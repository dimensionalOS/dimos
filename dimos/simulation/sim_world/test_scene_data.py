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


"""The world a viewer fetches leaves the robot out, and its STL and PNG files rebuild the geometry and textures exactly."""

import io
import json
from types import SimpleNamespace

import mujoco
import numpy as np
from PIL import Image

from dimos.msgs.files.File import File
from dimos.msgs.std_msgs.String import String
from dimos.simulation.sim_world.scene_data import SCENE_DATA_FORMAT, SCENE_FILE, scene_files
from dimos.simulation.sim_world.world import SimWorld

WORLD = """
<mujoco>
  <asset>
    <texture name="floor" type="2d" builtin="checker" width="8" height="8" rgb1="0 0 0" rgb2="1 1 1"/>
    <material name="floor" texture="floor" texrepeat="1 1" texuniform="true"/>
    <mesh name="rock" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
  </asset>
  <worldbody>
    <geom name="floor_0" type="box" size="5 4 0.1" material="floor"/>
    <geom name="wall_0" type="box" size="1 0.1 1" rgba="0.8 0.8 0.8 1"/>
    <geom name="rock_0" type="mesh" mesh="rock" pos="2 0 0"/>
    <body name="base_link" pos="0 0 0.5">
      <freejoint/>
      <geom type="sphere" size="0.2"/>
      <body name="leg"><geom type="capsule" size="0.05 0.2"/></body>
    </body>
  </worldbody>
</mujoco>
"""


def _scene() -> tuple[mujoco.MjModel, dict[str, bytes]]:
    model = mujoco.MjModel.from_xml_string(WORLD)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    robot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    return model, scene_files(model, data, "office-1", "map", robot)


def _stl_triangles(data: bytes) -> np.ndarray:
    count = int(np.frombuffer(data, "<u4", 1, 80)[0])
    record = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    return np.frombuffer(data, record, count, 84)["vertices"]


def test_the_index_is_the_world_without_the_robot() -> None:
    _, files = _scene()
    index = json.loads(files[SCENE_FILE])
    assert (index["format"], index["id"], index["frame"]) == (SCENE_DATA_FORMAT, "office-1", "map")
    assert [body["name"] for body in index["bodies"]] == ["world"]
    floor, wall, rock = index["bodies"][0]["geoms"]
    assert (floor["texture"], floor["texrepeat"], floor["texuniform"]) == (
        "floor",
        [1.0, 1.0],
        True,
    )
    assert "texture" not in wall and rock["mesh"] == "rock"
    # every file the index names is served, and nothing else
    named = {index["meshes"]["rock"]["file"], index["textures"]["floor"]["file"]}
    assert set(files) == {SCENE_FILE, *named}


def test_the_files_rebuild_the_mesh_and_texture_exactly() -> None:
    model, files = _scene()
    index = json.loads(files[SCENE_FILE])
    mesh = model.mesh("rock").id
    vertices = model.mesh_vert[
        model.mesh_vertadr[mesh] : model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]
    ]
    faces = model.mesh_face[
        model.mesh_faceadr[mesh] : model.mesh_faceadr[mesh] + model.mesh_facenum[mesh]
    ]
    assert np.array_equal(
        _stl_triangles(files[index["meshes"]["rock"]["file"]]), vertices[faces].astype(np.float32)
    )
    texture = index["textures"]["floor"]
    pixels = np.asarray(Image.open(io.BytesIO(files[texture["file"]])))
    tex = model.texture("floor").id
    expected = model.tex_data[model.tex_adr[tex] : model.tex_adr[tex] + 8 * 8 * 3].reshape(8, 8, 3)
    assert (texture["width"], texture["height"], texture["channels"]) == (8, 8, 3)
    assert np.array_equal(pixels, expected)


def test_the_world_answers_only_the_files_it_serves() -> None:
    _, files = _scene()
    sent: list[File] = []
    world = SimpleNamespace(
        _sim=object(),
        _scene_files=lambda _sim: files,
        file_info=SimpleNamespace(publish=sent.append),
    )
    for name in (SCENE_FILE, "robot.urdf", "/abs/path/to/mesh.STL", *files):
        SimWorld._on_render_file_request(world, String(name))  # type: ignore[arg-type]
    assert [f.uri for f in sent] == [SCENE_FILE, *files]
    assert sent[0].data == files[SCENE_FILE]
