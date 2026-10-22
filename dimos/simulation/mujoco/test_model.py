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

from dimos.simulation.mujoco.model import add_lidar_site


@pytest.mark.parametrize(
    "robot,mount,position",
    [("unitree_g1", "torso_link", [0.05, 0, 0.5]), ("unitree_go1", "trunk", [0.3, 0, 0.1])],
)
def test_lidar_site_attaches_to_robot_with_forward_x_and_up_z(
    robot: str, mount: str, position: list[float]
) -> None:
    robot_xml = f"""
        <mujoco>
          <worldbody>
            <body name="{mount}" pos="1 2 3">
              <camera name="head_camera"/>
              <camera name="lidar_front_camera"/>
            </body>
          </worldbody>
        </mujoco>
    """.encode()

    model = mujoco.MjModel.from_xml_string(add_lidar_site(robot_xml, robot).decode())
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    site_id = model.site("lidar").id
    assert model.site_bodyid[site_id] == model.body(mount).id
    np.testing.assert_allclose(data.site_xpos[site_id], np.array(position) + np.array([1, 2, 3]))
    np.testing.assert_allclose(data.site_xmat[site_id].reshape(3, 3), np.eye(3))
    assert model.camera("head_camera").id >= 0
    assert model.camera("lidar_front_camera").id >= 0
