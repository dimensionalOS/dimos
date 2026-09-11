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
from dimos.robot.pollen.microduck.sim_module import LIDAR_CAMERA_SPECS
from microduck_world.sensors import RobotSensors


def test_range_sensor_sees_front_wall_without_revealing_object_behind_it():
    spec = mujoco.MjSpec.from_string("""<mujoco><worldbody>
      <body name="trunk_base"><geom type="sphere" size=".1"/>
      </body>
      <geom name="wall" type="box" pos="0 0 -1" size="2 2 .05"/>
      <geom name="hidden" type="sphere" pos="0 0 -2" size=".2"/>
    </worldbody></mujoco>""")
    for name, _ in LIDAR_CAMERA_SPECS:
        spec.body("trunk_base").add_camera(name=name)
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    original_groups = model.geom_group.copy()
    points = RobotSensors._range_points(model, data, "", np.array([[0.0, 0.0, -1.0]]))
    np.testing.assert_allclose(points, [[0, 0, -0.95]] * 3)
    np.testing.assert_array_equal(model.geom_group, original_groups)
