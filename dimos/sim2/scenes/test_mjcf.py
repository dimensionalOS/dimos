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

"""An MJCF environment survives the trip into a composed USD scene."""

import math

import mujoco
import numpy as np
import pytest

from dimos.sim2.geometry.mujoco import compile_scene
from dimos.sim2.scenes.mjcf import components_from_mjcf, free_place
from dimos.sim2.scenes.urdf import prepare_scene

# A room in miniature: a slab floor, a flat textured wall that is only drawn, a
# free crate, a cabinet whose door turns on a damped hinge and carries a handle,
# and a ball joint, which has no URDF form. The door overlaps the cabinet's frame;
# their contact masks say that the two never collide.
ROOM = """
<mujoco>
  <asset>
    <mesh name="quad" inertia="shell" vertex="0 0 0  2 0 0  2 2 0  0 2 0" face="0 1 2  0 2 3"/>
    <texture name="wood" type="2d" builtin="flat" rgb1="0.6 0.4 0.2" width="8" height="8"/>
    <material name="wood" texture="wood" rgba="1 1 1 1"/>
  </asset>
  <worldbody>
    <geom name="floor" type="box" size="3 3 0.05" pos="0 0 -0.05"/>
    <geom name="wall" type="mesh" mesh="quad" material="wood" pos="-1 2 0" euler="90 0 0"
          contype="0" conaffinity="0"/>
    <body name="crate" pos="1 0 0.2">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="2"/>
    </body>
    <body name="cabinet" pos="-1 0 0.5">
      <geom name="frame" type="box" size="0.3 0.3 0.5" contype="1" conaffinity="0"/>
      <body name="door" pos="0.32 0.3 0">
        <joint name="hinge" type="hinge" axis="0 0 1" range="0 90" limited="true" damping="0.5"/>
        <geom type="box" size="0.03 0.3 0.5" pos="0 -0.3 0" mass="1" contype="1" conaffinity="0"/>
        <body name="handle" pos="0.05 -0.5 0">
          <geom type="capsule" size="0.01 0.05" mass="0.1" contype="1" conaffinity="0"/>
        </body>
      </body>
    </body>
    <body name="lamp" pos="0 -1 1">
      <joint type="ball"/>
      <geom type="sphere" size="0.05" mass="0.2"/>
    </body>
  </worldbody>
</mujoco>
"""


@pytest.fixture
def room(tmp_path):
    source = tmp_path / "room.xml"
    source.write_text(ROOM)
    return components_from_mjcf(source, tmp_path / "environment")


def test_a_room_becomes_one_fixed_part_and_one_component_per_movable_tree(room, tmp_path):
    # The fixed part is one component for each contact mask: the frame, then the slab.
    assert set(room.components) == {"env_static", "env_static_1_1", "env_crate", "env_door"}
    assert room.components["env_door"].collision_exclusions == (("env/static", "env/door"),)
    assert room.has_floor  # The slab, so the caller adds no second ground.
    assert room.welded == ("lamp",) and room.skipped == ()
    crate, door = room.components["env_crate"], room.components["env_door"]
    assert crate.floating and crate.xyz == pytest.approx((1, 0, 0.2))
    assert not door.floating and door.xyz == pytest.approx((-0.68, 0.3, 0.5))
    # A hinge keeps its own damping as a drive with no stiffness.
    assert door.drives == {"env/hinge": (0.0, 0.5, 0.0)}
    hinge = door.model.load().get_joint("env/hinge")
    assert hinge.type == "revolute" and hinge.parent_link == "env/door/base"
    assert (hinge.lower, hinge.upper) == pytest.approx((0, math.pi / 2))
    # A textured surface keeps the mean color of its texture.
    static = (tmp_path / "environment" / "static.urdf").read_text()
    assert 'rgba="0.6 0.4 0.2 1"' in static


def test_the_composed_scene_simulates_what_the_source_states(room, tmp_path):
    scene = prepare_scene(
        room.components, ground_plane=not room.has_floor, mesh_dir=tmp_path / "usd_meshes"
    )
    # The flat wall has no volume: it must still load, because nothing weighs it.
    model = compile_scene(scene)
    source = mujoco.MjModel.from_xml_string(ROOM)
    data, expected = mujoco.MjData(model), mujoco.MjData(source)
    mujoco.mj_forward(model, data)
    mujoco.mj_forward(source, expected)
    assert model.body("env/crate").mass[0] == pytest.approx(2)
    # The handle moves with the door, so they are one body with both masses.
    door = model.body("env/door")
    assert door.mass[0] == pytest.approx(1.1)
    np.testing.assert_allclose(
        data.subtree_com[door.id], expected.subtree_com[source.body("door").id], atol=1e-6
    )
    joint = model.joint("env/hinge")
    assert joint.type[0] == mujoco.mjtJoint.mjJNT_HINGE
    np.testing.assert_allclose(joint.range, [0, math.pi / 2], atol=1e-6)
    np.testing.assert_allclose(
        data.xanchor[joint.id], expected.xanchor[source.joint("hinge").id], atol=1e-6
    )
    np.testing.assert_allclose(
        data.xaxis[joint.id], expected.xaxis[source.joint("hinge").id], atol=1e-6
    )
    # The door overlaps its frame by a centimetre. The source's contact masks keep
    # the two apart, and so does the scene: nothing pushes the door open.
    for _ in range(500):
        mujoco.mj_step(model, data)
    assert abs(data.qpos[joint.qposadr[0]]) < 1e-3
    assert not any(
        {model.body(model.geom_bodyid[c.geom1]).name, model.body(model.geom_bodyid[c.geom2]).name}
        == {"env/door", "env/static"}
        for c in data.contact[: data.ncon]
    )
    # The slab collides: the crate drops 10 cm onto it and rests there.
    for _ in range(1000):
        mujoco.mj_step(model, data)
    assert data.xpos[model.body("env/crate").id][2] == pytest.approx(0.1, abs=0.005)


def test_a_free_place_has_floor_under_it_and_nothing_in_the_footprint(tmp_path):
    source = tmp_path / "room.xml"
    source.write_text(ROOM)
    # A clear spot is kept as it is.
    assert free_place(source, (0.0, -2.0), radius=0.3, height=0.8) == pytest.approx((0, -2, 0))
    # The middle of the cabinet is taken: the nearest spot clears its 0.3 m half width.
    x, y, floor = free_place(source, (-1.0, 0.0), radius=0.3, height=0.8)
    assert floor == pytest.approx(0, abs=1e-6)
    assert max(abs(x + 1), abs(y)) >= 0.6 and math.hypot(x + 1, y) < 1.2
    # A movable object is in the way as much as a fixed one.
    x, y, _ = free_place(source, (1.0, 0.0), radius=0.3, height=0.8)
    assert max(abs(x - 1), abs(y)) >= 0.4
    # Beyond the slab there is no floor to stand on.
    assert free_place(source, (10.0, 10.0), radius=0.3, height=0.8, reach=1.0) is None
