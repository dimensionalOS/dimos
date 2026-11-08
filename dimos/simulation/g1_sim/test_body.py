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

"""The G1 walking in MuJoCo, from the one USD that describes it."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from dimos.simulation.g1_sim.body import LeggedG1, g1_spec
from dimos.simulation.g1_sim.policy import G1Policy

pytestmark = pytest.mark.self_hosted


@pytest.fixture(scope="module")
def standing() -> LeggedG1:
    """The G1 on a bare floor, reset into the policy's default pose."""
    spec = g1_spec()
    floor = spec.worldbody.add_geom()
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.size = [0.0, 0.0, 0.05]
    floor.contype = 1
    floor.conaffinity = 1
    model = spec.compile()
    body = LeggedG1(model, mujoco.MjData(model), G1Policy.load(LeggedG1.dt))
    body.reset(0.0, 0.0, 0.0, 0.0)
    return body


def test_the_layer_alone_gives_a_complete_robot(standing: LeggedG1) -> None:
    """Every joint actuated, both soles found: the layer carries the whole MJCF's physics."""
    assert len(standing.joint_names) == 29
    assert standing.model.nu == 29
    assert len(standing.feet) == 4


def test_it_walks_forward_on_its_feet(standing: LeggedG1) -> None:
    """The point of the whole exercise: a humanoid simulated out of a USD and nothing else.

    It used to fall through the floor. The vendor MJCF switches collision off robot-wide and
    puts the soles back in contact with two explicit <pair> elements, which MuJoCo's USD
    converter drops -- so the robot free-fell about 78 m in four seconds.
    """
    standing.reset(0.0, 0.0, 0.0, 0.0)
    start, _ = standing.base_pose()

    for _ in range(200):  # 4 s at 50 Hz
        standing.tick(np.array([0.4, 0.0, 0.0]))

    end, _ = standing.base_pose()
    assert end[0] - start[0] > 1.0, "it did not walk forward"
    assert abs(end[1] - start[1]) < 1.0, "it wandered sideways"
    assert end[2] > 0.5, "it fell over or through the floor"
    assert standing.upright() > 0.9


def test_it_walks_along_its_own_forward_axis(standing: LeggedG1) -> None:
    """Started facing +y, a forward command has to move it along +y.

    The actor is given linear velocity in the **base** frame. Handing it the world frame instead
    is invisible while the robot faces +x, because the two frames agree there -- which is why
    this test turns it a quarter turn first.
    """
    standing.reset(0.0, 0.0, 0.0, float(np.pi / 2))
    start, _ = standing.base_pose()

    for _ in range(200):
        standing.tick(np.array([0.4, 0.0, 0.0]))

    end, _ = standing.base_pose()
    forward, sideways = end[1] - start[1], end[0] - start[0]
    assert forward > 1.0, "it did not walk along the way it was facing"
    assert abs(sideways) < abs(forward) / 2.0, "it drifted more than it walked"
