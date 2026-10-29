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
