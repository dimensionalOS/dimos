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

"""xArm MuJoCo scenes cancel the arm's weight within each joint's actuator force range."""

import mujoco
import numpy as np
import pytest

from dimos.utils.data import LfsPath

pytestmark = pytest.mark.self_hosted

SCENES = [
    "xarm6/scene.xml",
    "xarm7/scene.xml",
    "xarm_grasp_sim/scene.xml",
    "robosuite/lift/scene.xml",
]


@pytest.mark.parametrize("scene", SCENES)
def test_arm_gravity_compensation_is_force_limited(scene: str) -> None:
    model = mujoco.MjModel.from_xml_path(str(LfsPath(scene)))
    arm = [model.joint(f"joint{i}").id for i in range(1, 8) if f"joint{i}" in _joints(model)]

    assert len(arm) >= 6
    assert model.body_gravcomp[model.body("link1").id] == 1.0
    for joint in arm:
        a = int(np.flatnonzero(model.actuator_trnid[:, 0] == joint)[0])
        assert model.jnt_actgravcomp[joint]
        assert model.jnt_actfrclimited[joint]
        np.testing.assert_allclose(model.jnt_actfrcrange[joint], model.actuator_forcerange[a])


def _joints(model: mujoco.MjModel) -> set[str]:
    return {model.joint(j).name for j in range(model.njnt)}
