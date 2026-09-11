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

"""The synthetic lidar includes physical floors and omits moving robot/cargo."""

import mujoco
import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.navigation_cloud import environment_cloud
from dimos.robot.galaxea.r1pro.packing_sim import PACKING_BODIES


@pytest.fixture
def cloud_scene():
    cargo = "".join(
        f'<body name="{name}" pos="5 0 1"><geom type="box" size=".1 .1 .1"/></body>'
        for name in ("task_bin", *PACKING_BODIES)
    )
    model = mujoco.MjModel.from_xml_string(f"""<mujoco><worldbody>
      <geom name="floor" type="box" size="1 1 .05" pos="0 0 -.05"/>
      <geom name="wall" type="box" size=".05 1 .5" pos="1.05 0 .5"/>
      <geom name="visual_only" type="box" size=".1 .1 .1" pos="9 0 1" contype="0" conaffinity="0"/>
      <body name="base_link" pos="4 0 1"><geom type="box" size=".2 .2 .2"/>
        <body name="articulated_arm"><joint type="hinge"/><geom type="box" size=".1 .1 .4"/></body>
      </body>{cargo}</worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_complete_cloud_includes_floor_and_wall_without_robot_or_cargo(cloud_scene):
    points = environment_cloud(*cloud_scene, spacing=0.1)
    floor = points[np.isclose(points[:, 2], 0)]
    assert floor[:, 0].min() <= -0.99
    assert floor[:, 1].max() >= 0.99
    assert points[:, 2].max() == pytest.approx(1)
    assert points[:, 0].max() <= 1.10001
    assert np.isfinite(points).all()


def test_map_sampling_is_repeatable(cloud_scene):
    assert np.array_equal(
        environment_cloud(*cloud_scene, spacing=0.1), environment_cloud(*cloud_scene, spacing=0.1)
    )
