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
import numpy as np
import pytest

from dimos.simulation.mujoco.freejoint import body_twist

_MODEL = """
<mujoco>
  <option gravity="0 0 0" timestep="0.0001"/>
  <worldbody>
    <body>
      <freejoint/>
      <geom type="box" size="0.3 0.2 0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""


def test_body_twist_matches_finite_difference_of_pose() -> None:
    model = mujoco.MjModel.from_xml_string(_MODEL)
    data = mujoco.MjData(model)
    data.qpos[3:7] = [np.cos(0.4), 0.0, 0.0, np.sin(0.4)]
    data.qvel[:] = [0.5, -0.2, 0.1, 0.0, 0.0, 0.3]
    mujoco.mj_forward(model, data)

    quat = data.qpos[3:7].copy()
    linear, angular = body_twist(quat, data.qvel)
    pos, rotation = data.xpos[1].copy(), data.xmat[1].reshape(3, 3).copy()
    mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    dt = model.opt.timestep

    body_displacement = rotation.T @ (data.xpos[1] - pos) / dt
    delta = rotation.T @ data.xmat[1].reshape(3, 3)
    body_rates = np.array([delta[2, 1], delta[0, 2], delta[1, 0]]) / dt
    assert linear == pytest.approx(body_displacement, abs=1e-3)
    assert angular == pytest.approx(body_rates, abs=1e-3)
