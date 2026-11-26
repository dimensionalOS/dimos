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

"""Exercise the native MJCF/USD round trip with visual and contact geometry."""

from importlib import import_module
from pathlib import Path

import mujoco
import numpy as np
from pxr import Usd

from dimos.experimental.gta_sim2.prepare_m20 import restore_m20_contacts


def test_visual_mesh_is_not_a_collider_and_friction_survives(tmp_path: Path) -> None:
    import_module("mujoco._usd")
    source = tmp_path / "robot.xml"
    source.write_text("""<mujoco model="M20">
      <asset><mesh name="shell" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
        <material name="original" rgba="0 0 0 0"/></asset>
      <worldbody><body name="base">
        <geom type="mesh" mesh="shell" contype="0" conaffinity="0" group="2"/>
        <geom type="cylinder" size=".09 .027" group="1" friction=".8 .01 .001" material="original"
              solref=".005 1" quat=".70710678 .70710678 0 0"/>
      </body></worldbody>
    </mujoco>""")
    native = mujoco.MjSpec.from_file(str(source)).compile()
    stage = Usd.Stage.Open(str(source))
    restore_m20_contacts(stage, native)
    converted = tmp_path / "robot.usda"
    stage.Export(str(converted))
    result = mujoco.MjSpec.from_file(str(converted)).compile()
    contact = np.flatnonzero((result.geom_contype != 0) | (result.geom_conaffinity != 0))
    assert len(contact) == 1
    index = contact[0]
    assert result.geom_type[index] == mujoco.mjtGeom.mjGEOM_CYLINDER
    np.testing.assert_allclose(result.geom_friction[index], [0.8, 0.01, 0.001], rtol=1e-6)
    np.testing.assert_allclose(result.geom_solref[index], [0.005, 1])
    visual = np.flatnonzero(result.geom_type == mujoco.mjtGeom.mjGEOM_MESH)
    assert len(visual) == 1
    np.testing.assert_array_equal(result.geom_contype[visual], 0)
    np.testing.assert_array_equal(result.geom_conaffinity[visual], 0)
