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

from dimos.simulation.mujoco.shadows import disable_shadows

_LIGHTS = """
<mujoco>
  <worldbody>
    <light name="sun" directional="true" pos="0 0 3" dir="0 0 -1"/>
    <light name="narrow" pos="0 0 2" dir="0 0 -1" cutoff="30"/>
    <light name="wide" pos="0 0 2" dir="0 0 -1" cutoff="120"/>
    <geom type="plane" size="1 1 0.1"/>
  </worldbody>
</mujoco>
"""


def test_disable_shadows_drops_the_shadow_map_and_narrow_spot_lights() -> None:
    model = mujoco.MjModel.from_xml_string(_LIGHTS)
    assert model.vis.quality.shadowsize > 0

    disable_shadows(model)

    assert model.vis.quality.shadowsize == 0
    active = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_LIGHT, i): bool(model.light_active[i])
        for i in range(model.nlight)
    }
    assert active == {"sun": True, "narrow": False, "wide": True}
