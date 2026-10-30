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

"""MotorScene is independently usable through the upstream environment lifecycle."""

from contextlib import closing

import numpy as np
import pytest

from dimos.sim2.environment import MotorScene


@pytest.mark.parametrize("hard_reset", [False, True])
def test_motor_scene_can_step_and_reset_without_a_dimos_runtime(
    scene_path, robot_factory, hard_reset
):
    action = np.array([[0.3, 0.02], [0, 0], [2, 0], [0, 0], [0.4, 0]])
    with closing(
        MotorScene(scene_path, models={"left": robot_factory}, hard_reset=hard_reset)
    ) as environment:
        environment.step(action.ravel())

        assert environment.sim.data.ctrl == pytest.approx([0.8, 0.02])
        environment.reset()
        assert environment.controllers["left"].positions == pytest.approx([0.1, 0.0])
        assert environment.sim.data.ctrl == pytest.approx([0.0, 0.0])

        environment.step(action.ravel())

        assert environment.sim.data.ctrl == pytest.approx([0.8, 0.02])
