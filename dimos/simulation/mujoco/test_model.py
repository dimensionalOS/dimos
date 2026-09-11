# Copyright 2025-2026 Dimensional Inc.
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
import pytest

from dimos.simulation.mujoco.model import get_assets


@pytest.mark.self_hosted
def test_legacy_assets_include_hand_inclusive_g1_meshes() -> None:
    xml = """
        <mujoco>
          <asset><mesh name="pelvis" file="pelvis.STL"/></asset>
          <worldbody><geom type="mesh" mesh="pelvis"/></worldbody>
        </mujoco>
    """

    model = mujoco.MjModel.from_xml_string(xml, assets=get_assets())

    assert model.nmesh == 1
