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
from microduck_world.camera import configure_clipping, configure_head_camera


def test_camera_uses_published_mount_and_follows_head_motion():
    spec = mujoco.MjSpec.from_string("""<mujoco><worldbody>
      <body name="trunk_base"><geom type="sphere" size=".1"/>
        <body name="head" pos="0 0 .2">
          <joint name="head_yaw" axis="0 0 1"/>
          <geom type="sphere" size=".05"/>
          <site name="head_camera" pos=".05 0 0"/>
          <camera name="head_camera" pos=".05 0 0" quat="0 0 1 0"/>
        </body>
      </body>
    </worldbody></mujoco>""")
    configure_head_camera(spec)
    model = spec.compile()
    data = mujoco.MjData(model)
    camera = model.camera("head_camera").id
    site = model.site("head_camera").id
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.cam_xpos[camera], [0.05, 0, 0.2])
    np.testing.assert_allclose(-data.cam_xmat[camera].reshape(3, 3)[:, 2], [1, 0, 0])
    np.testing.assert_allclose(data.cam_xmat[camera].reshape(3, 3)[:, 1], [0, 0, 1])

    data.qpos[model.joint("head_yaw").qposadr[0]] = np.pi / 2
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.cam_xpos[camera], data.site_xpos[site], atol=1e-12)
    np.testing.assert_allclose(data.cam_xpos[camera], [0, 0.05, 0.2], atol=1e-12)
    np.testing.assert_allclose(-data.cam_xmat[camera].reshape(3, 3)[:, 2], [0, 1, 0], atol=1e-12)


@pytest.mark.parametrize("extent", [2.0, 15.0])
def test_expanding_world_does_not_hide_nearby_objects_from_camera(extent):
    model = mujoco.MjModel.from_xml_string("<mujoco/>")
    model.stat.extent = extent
    configure_clipping(model)
    assert model.vis.map.znear * model.stat.extent == pytest.approx(0.005)
    assert model.vis.map.zfar * model.stat.extent == pytest.approx(30.0)
