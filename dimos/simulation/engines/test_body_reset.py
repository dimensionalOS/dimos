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

import numpy as np
import pytest

from dimos.simulation.engines.mujoco_engine import MujocoEngine

pytestmark = pytest.mark.mujoco


def test_free_body_pose_edit_clears_velocity_and_reset_restores_spawn(tmp_path):
    path = tmp_path / "free.xml"
    path.write_text("""<mujoco><worldbody>
      <body name="bottle" pos="0.2 0.3 0.5"><freejoint/><geom size="0.02" mass="0.1"/></body>
      <body name="fixed"><geom size="0.01"/></body>
    </worldbody></mujoco>""")
    engine = MujocoEngine(config_path=path, headless=True)
    try:
        engine.data.qvel[:] = [1, 2, 3, 4, 5, 6]
        engine.set_body_pose("bottle", [0.4, -0.2, 0.6], [0, 0, 2, 2])
        xyz, quat = engine.get_body_pose("bottle")
        np.testing.assert_allclose(xyz, [0.4, -0.2, 0.6])
        np.testing.assert_allclose(quat, [0, 0, 2**-0.5, 2**-0.5])
        np.testing.assert_array_equal(engine.data.qvel, np.zeros(6))
        with pytest.raises(ValueError, match="free joint"):
            engine.set_body_pose("fixed", [0, 0, 0], [0, 0, 0, 1])
        with pytest.raises(ValueError, match="nonzero"):
            engine.set_body_pose("bottle", [0, 0, 0], [0, 0, 0, 0])
        engine.reset()
        np.testing.assert_allclose(engine.get_body_pose("bottle")[0], [0.2, 0.3, 0.5])
    finally:
        engine.disconnect()
