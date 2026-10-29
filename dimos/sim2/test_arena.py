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

"""Importing an authored arena must preserve its native physical parameters."""

import mujoco
import numpy as np
from robosuite.models.tasks.task import Task

from dimos.sim2.arena import AuthoredArena
from dimos.sim2.unreal.assets import export_scene


def test_arena_preserves_anonymous_nested_defaults_units_and_includes(tmp_path):
    included = tmp_path / "fixture.xml"
    included.write_text("""
<mujocoinclude>
  <body name="door" childclass="fixture" pos="0 0 1">
    <joint name="hinge" axis="0 0 1" range="-45 45"/>
    <geom name="panel" type="box" size=".1 .2 .3"/>
    <body name="handle" childclass="handle" pos=".1 0 0">
      <geom name="grip" type="sphere" size=".04"/>
    </body>
  </body>
</mujocoinclude>
""")
    path = tmp_path / "arena.xml"
    path.write_text("""
<mujoco>
  <compiler angle="degree"/>
  <default>
    <geom friction=".23 .004 .0008" density="730"/>
    <joint damping="2.75" armature=".03"/>
    <default class="fixture">
      <geom friction=".4 .006 .001"/>
      <default class="handle"><geom density="900"/></default>
    </default>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 .1"/>
    <include file="fixture.xml"/>
  </worldbody>
</mujoco>
""")
    native = mujoco.MjModel.from_xml_path(str(path))
    imported = Task(AuthoredArena(path), []).get_model()

    assert (imported.nbody, imported.ngeom, imported.nmocap) == (
        native.nbody,
        native.ngeom,
        native.nmocap,
    )

    for kind, fields in (
        (mujoco.mjtObj.mjOBJ_GEOM, ("geom_friction",)),
        (mujoco.mjtObj.mjOBJ_BODY, ("body_mass", "body_inertia")),
    ):
        count = native.ngeom if kind == mujoco.mjtObj.mjOBJ_GEOM else native.nbody
        indices = [
            mujoco.mj_name2id(imported, kind, mujoco.mj_id2name(native, kind, i))
            for i in range(count)
        ]
        for name in fields:
            np.testing.assert_allclose(
                getattr(imported, name)[indices], getattr(native, name), atol=1e-6
            )
    # Native MjSpec XML serialization rounds numeric attributes to six digits.
    for name in ("dof_damping", "dof_armature", "jnt_range"):
        np.testing.assert_allclose(getattr(imported, name), getattr(native, name), atol=1e-6)
    assert imported.geom("grip").friction.tolist() == [0.4, 0.006, 0.001]
    np.testing.assert_allclose(imported.joint("hinge").range, [-np.pi / 4, np.pi / 4], atol=1e-6)


def test_arena_removes_only_injected_ui_targets_and_preserves_authored_mocap_geometry(tmp_path):
    path = tmp_path / "authored.xml"
    path.write_text("""
        <mujoco><worldbody>
          <body name="left_eef_target" mocap="true" pos="1 2 3">
            <geom name="authored_box" type="box" size=".1 .2 .3" group="2" rgba=".2 .4 .6 1"/>
          </body>
          <body name="tracking_fixture" mocap="true" pos="-2 1 4">
            <geom name="authored_sphere" type="sphere" size=".2" group="2" rgba=".7 .3 .1 1"/>
          </body>
        </worldbody></mujoco>
    """)
    native = mujoco.MjModel.from_xml_path(str(path))
    imported = Task(AuthoredArena(path), []).get_model()

    assert [imported.body(i).name for i in range(imported.nbody)] == [
        native.body(i).name for i in range(native.nbody)
    ]
    assert [imported.geom(i).name for i in range(imported.ngeom)] == [
        "authored_box",
        "authored_sphere",
    ]
    assert imported.nmocap == native.nmocap == 2
    for field in (
        "body_pos",
        "body_quat",
        "body_mocapid",
        "geom_pos",
        "geom_size",
        "geom_group",
        "geom_rgba",
    ):
        np.testing.assert_allclose(getattr(imported, field), getattr(native, field), atol=1e-6)


def test_arena_keeps_authored_collision_colors_and_inherited_materials(tmp_path):
    path = tmp_path / "painted.xml"
    path.write_text("""
        <mujoco>
          <asset>
            <material name="paint" rgba=".8 .3 .1 1" roughness=".25"/>
            <material name="metal" rgba=".2 .4 .6 1" roughness=".1"/>
          </asset>
          <default><geom material="paint" friction=".3 .01 .001"/></default>
          <worldbody>
            <geom name="inherited" type="box" size=".2 .3 .4"/>
            <geom name="explicit" type="sphere" size=".1" material="metal" rgba=".6 .7 .8 1"/>
            <geom name="visual" type="sphere" size=".1" group="1"/>
          </worldbody>
        </mujoco>
    """)
    native = mujoco.MjModel.from_xml_path(str(path))
    imported = Task(AuthoredArena(path), []).get_model()

    for name in ("inherited", "explicit", "visual"):
        original, copied = native.geom(name), imported.geom(name)
        np.testing.assert_allclose(copied.rgba, original.rgba)
        np.testing.assert_allclose(copied.friction, original.friction)
        assert imported.mat(int(copied.matid[0])).name == native.mat(int(original.matid[0])).name
    data = mujoco.MjData(imported)
    mujoco.mj_forward(imported, data)
    colors = [geom["rgba"] for geom in export_scene(imported, data)["geoms"]]
    np.testing.assert_allclose(colors, [[0.8, 0.3, 0.1, 1], [0.6, 0.7, 0.8, 1], [0.8, 0.3, 0.1, 1]])
