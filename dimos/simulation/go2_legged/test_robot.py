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

from dimos.simulation.go2_legged.policy import OnnxGo2Policy
from dimos.simulation.go2_legged.robot import (
    CONTROL_DT,
    LeggedGo2,
    apply_fitted_physics,
    go2_spec,
)

pytestmark = pytest.mark.self_hosted


@pytest.fixture(scope="module")
def policy() -> OnnxGo2Policy:
    return OnnxGo2Policy.load()


@pytest.fixture(scope="module")
def flat_model() -> mujoco.MjModel:
    spec = go2_spec()
    ground = spec.worldbody.add_geom()
    ground.type = mujoco.mjtGeom.mjGEOM_BOX
    ground.pos = (5.0, 0.0, -0.1)
    ground.size = (10.0, 5.0, 0.1)
    model = spec.compile()
    apply_fitted_physics(model)
    return model


@pytest.fixture
def make_robot(flat_model: mujoco.MjModel, policy: OnnxGo2Policy) -> Callable[[], LeggedGo2]:
    def make() -> LeggedGo2:
        robot = LeggedGo2(flat_model, mujoco.MjData(flat_model), policy)
        robot.reset(0.0, 0.0, 0.0, 0.0)
        return robot

    return make


def _walk(
    robot: LeggedGo2, seconds: float, command: tuple[float, float, float]
) -> NDArray[np.float64]:
    for _ in range(round(seconds / CONTROL_DT)):
        robot.tick(np.asarray(command, dtype=np.float64))
    assert robot.upright() > 0.9
    return np.append(robot.base_pose()[0], robot.yaw())


def test_reset_places_the_feet_on_the_requested_height(make_robot: Callable[[], LeggedGo2]) -> None:
    robot = make_robot()
    robot.reset(2.0, 1.0, 0.3, 1.0)
    base = robot.base_pose()[0]
    assert base[:2] == pytest.approx([2.0, 1.0], abs=0.02)
    assert robot.yaw() == pytest.approx(1.0)
    model, data = robot.model, robot.data
    lowest = min(float(data.geom_xpos[g][2] - model.geom_size[g][0]) for g in robot.feet)
    assert lowest == pytest.approx(0.3, abs=1e-6)


def test_stays_upright_without_a_command(make_robot: Callable[[], LeggedGo2]) -> None:
    end = _walk(make_robot(), 3.0, (0.0, 0.0, 0.0))
    assert np.hypot(end[0], end[1]) < 0.3
    assert 0.25 < end[2] < 0.4


def test_walks_forward_at_about_the_commanded_speed(make_robot: Callable[[], LeggedGo2]) -> None:
    end = _walk(make_robot(), 6.0, (0.8, 0.0, 0.0))
    assert 3.0 < end[0] < 5.0
    assert abs(end[1]) < 1.0


def test_turns_at_about_the_commanded_rate(make_robot: Callable[[], LeggedGo2]) -> None:
    end = _walk(make_robot(), 4.0, (0.0, 0.0, 0.5))
    assert 1.2 < end[3] < 2.4


def test_rollouts_are_deterministic(make_robot: Callable[[], LeggedGo2]) -> None:
    a = _walk(make_robot(), 2.0, (0.5, 0.0, 0.3))
    b = _walk(make_robot(), 2.0, (0.5, 0.0, 0.3))
    assert np.array_equal(a, b)
