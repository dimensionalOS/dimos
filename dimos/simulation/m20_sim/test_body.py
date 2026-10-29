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

from __future__ import annotations

from collections.abc import Callable

import mujoco
import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.simulation.m20_sim.body import CONTROL_DT, LeggedM20, m20_spec
from dimos.simulation.m20_sim.policy import M20Policy


@pytest.fixture(scope="module")
def policy() -> M20Policy:
    return M20Policy.load()


@pytest.fixture(scope="module")
def flat_model() -> mujoco.MjModel:
    spec = m20_spec()
    ground = spec.worldbody.add_geom()
    ground.type = mujoco.mjtGeom.mjGEOM_BOX
    ground.pos = (5.0, 0.0, -0.1)
    ground.size = (20.0, 20.0, 0.1)
    model: mujoco.MjModel = spec.compile()
    return model


@pytest.fixture
def make_robot(flat_model: mujoco.MjModel, policy: M20Policy) -> Callable[[], LeggedM20]:
    def make() -> LeggedM20:
        robot = LeggedM20(flat_model, mujoco.MjData(flat_model), policy)
        robot.reset(0.0, 0.0, 0.0, 0.0)
        return robot

    return make


def _drive(
    robot: LeggedM20, seconds: float, command: tuple[float, float, float]
) -> NDArray[np.float64]:
    """Position and unwrapped yaw after driving, asserting the robot stayed upright."""
    yaws = []
    for _ in range(round(seconds / CONTROL_DT)):
        robot.tick(np.asarray(command, dtype=np.float64))
        yaws.append(robot.yaw())
    assert robot.upright() > 0.9
    return np.append(robot.base_pose()[0], np.unwrap(yaws)[-1])


def test_reset_places_the_wheels_on_the_requested_height(
    make_robot: Callable[[], LeggedM20],
) -> None:
    robot = make_robot()
    robot.reset(2.0, 1.0, 0.3, 1.0)
    assert robot.base_pose()[0][:2] == pytest.approx([2.0, 1.0])
    assert robot.yaw() == pytest.approx(1.0)
    model, data = robot.model, robot.data
    lowest = min(float(data.geom_xpos[g][2] - model.geom_size[g][0]) for g in robot.wheels)
    assert lowest == pytest.approx(0.3, abs=1e-6)


def test_stands_still_without_a_command(make_robot: Callable[[], LeggedM20]) -> None:
    end = _drive(make_robot(), 4.0, (0.0, 0.0, 0.0))
    assert np.hypot(end[0], end[1]) < 0.2
    assert 0.5 < end[2] < 0.6


def test_drives_forward_slower_than_commanded(make_robot: Callable[[], LeggedM20]) -> None:
    end = _drive(make_robot(), 6.0, (0.5, 0.0, 0.0))
    assert 2.0 < end[0] < 3.0
    assert abs(end[1]) < 0.3


def test_a_positive_yaw_command_turns_left(make_robot: Callable[[], LeggedM20]) -> None:
    end = _drive(make_robot(), 4.0, (0.0, 0.0, 0.5))
    assert 3.0 < end[3] < 5.0
