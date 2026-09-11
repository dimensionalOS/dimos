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
from dimos.robot.pollen.microduck.gait import MicroduckObserver


@pytest.fixture
def two_ducks():
    spec = mujoco.MjSpec()
    for prefix in ("", "guest_"):
        robot = mujoco.MjSpec.from_string("""<mujoco><worldbody>
        <body name="trunk_base" pos="0 0 .2"><freejoint name="trunk_base_freejoint"/>
        <geom type="sphere" size=".03" mass="1"/><site name="imu"/>
        <body pos="0 0 .1"><joint name="knee"/><geom type="sphere" size=".01" mass=".1"/>
        </body></body></worldbody><sensor><gyro name="imu_ang_vel" site="imu"/></sensor>
        </mujoco>""")
        spec.attach(robot, prefix=prefix, frame=spec.worldbody.add_frame())
    model = spec.compile()
    return model, mujoco.MjData(model)


def test_standing_one_duck_preserves_the_other_ducks_motion(two_ducks):
    model, data = two_ducks
    host = MicroduckObserver(model, ["knee"], np.array([0.2], dtype=np.float32))
    guest = MicroduckObserver(model, ["knee"], np.array([0.3], dtype=np.float32), prefix="guest_")
    data.qvel[:] = 1.25
    guest.initial_qpos(data)
    np.testing.assert_array_equal(data.qvel[: guest.root_qvel_adr], 1.25)
    np.testing.assert_array_equal(data.qvel[guest.root_qvel_adr :], 0)
    assert data.qpos[model.joint("guest_knee").qposadr[0]] == pytest.approx(0.3)
    assert data.qpos[model.joint("knee").qposadr[0]] == 0
    assert host.root_qpos_adr != guest.root_qpos_adr


def test_observations_read_only_the_named_robot(two_ducks):
    model, data = two_ducks
    guest = MicroduckObserver(model, ["knee"], np.array([0.3], dtype=np.float32), prefix="guest_")
    data.qpos[model.joint("guest_knee").qposadr[0]] = 0.7
    data.qpos[model.joint("knee").qposadr[0]] = 2
    mujoco.mj_forward(model, data)
    obs = guest.build(data, np.zeros(1, dtype=np.float32), np.zeros(13, dtype=np.float32))
    assert obs[6] == pytest.approx(0.4)
