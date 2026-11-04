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


"""The world a viewer is sent leaves the robot out and carries the floor's texture and how it tiles."""

import json

import mujoco

from dimos.simulation.sim_world.scene_data import SCENE_DATA_FORMAT, scene_data

WORLD = """
<mujoco>
  <asset>
    <texture name="floor" type="2d" builtin="checker" width="8" height="8" rgb1="0 0 0" rgb2="1 1 1"/>
    <material name="floor" texture="floor" texrepeat="1 1" texuniform="true"/>
  </asset>
  <worldbody>
    <geom name="floor_0" type="box" size="5 4 0.1" material="floor"/>
    <geom name="wall_0" type="box" size="1 0.1 1" rgba="0.8 0.8 0.8 1"/>
    <body name="base_link" pos="0 0 0.5">
      <freejoint/>
      <geom type="sphere" size="0.2"/>
      <body name="leg"><geom type="capsule" size="0.05 0.2"/></body>
    </body>
  </worldbody>
</mujoco>
"""


def test_the_scene_is_the_world_without_the_robot() -> None:
    model = mujoco.MjModel.from_xml_string(WORLD)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    robot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    sent = json.loads(scene_data(model, data, "office-1", "map", robot))
    assert (sent["format"], sent["id"], sent["frame"]) == (SCENE_DATA_FORMAT, "office-1", "map")
    assert [body["name"] for body in sent["bodies"]] == ["world"]
    floor, wall = sent["bodies"][0]["geoms"]
    assert (floor["name"], floor["texture"], floor["texrepeat"], floor["texuniform"]) == (
        "floor_0",
        "floor",
        [1.0, 1.0],
        True,
    )
    assert "texture" not in wall
    texture = sent["textures"]["floor"]
    assert (texture["width"], texture["height"], texture["channels"]) == (8, 8, 3)
