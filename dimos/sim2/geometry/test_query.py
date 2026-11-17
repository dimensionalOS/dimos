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

import mujoco
import numpy as np
import pytest

from dimos.robot.assets.model import UsdModel
from dimos.sim2.geometry.mujoco import build_scene
from dimos.sim2.geometry.query import MujocoGeometry


def test_optical_rays_skip_authored_hidden_collision_geometry():
    model = mujoco.MjModel.from_xml_string(
        "<mujoco><worldbody>"
        '<geom name="visible" type="sphere" size=".1" pos="2 0 0"/>'
        '<geom name="hidden" type="sphere" size=".2" pos="1 0 0" group="3"/>'
        '<geom name="decoration" type="sphere" size=".1" pos="0 2 0" contype="0" conaffinity="0"/>'
        "</worldbody></mujoco>"
    )
    geometry = MujocoGeometry(model, "test")
    geometry.select_surfaces()
    mujoco.mj_forward(model, geometry.data)
    hit = np.array([-1], dtype=np.int32)
    distance = mujoco.mj_ray(
        model,
        geometry.data,
        np.zeros(3),
        np.array([1.0, 0, 0]),
        np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8),
        1,
        -1,
        hit,
    )
    assert hit[0] == model.geom("visible").id
    assert distance == pytest.approx(1.9)


def test_hidden_referenced_shapes_remain_in_physics_but_not_optical_resources(tmp_path):
    (tmp_path / "geometry.usda").write_text("""#usda 1.0
(defaultPrim = "world")
def Xform "world" {
    def Cube "visible" {
        double size = 1
    }
    def Cube "hull" {
        double size = 1.1
    }
}
""")
    source = tmp_path / "world.usda"
    source.write_text("""#usda 1.0
(defaultPrim = "world")
def Xform "world" (prepend references = @geometry.usda@</world>) {
    over "hull" {
        token visibility = "invisible"
    }
}
""")
    world = UsdModel.from_file(source)
    physical = build_scene(world).compile()
    optical = build_scene(world, kinematic=True).compile()
    assert physical.ngeom == 2
    assert optical.ngeom == 1
    assert optical.geom(0).name.endswith("/visible")
