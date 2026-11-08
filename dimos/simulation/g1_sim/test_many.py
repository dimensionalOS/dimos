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


"""Several G1s in one MuJoCo model, which is what a shared world is made of."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from dimos.simulation.g1_sim.body import LeggedG1, g1_spec
from dimos.simulation.g1_sim.policy import G1Policy
from dimos.simulation.g1_sim.world import G1_RIG
from dimos.simulation.sim_world.world import add_housings

pytestmark = pytest.mark.self_hosted

PLACES = {"a_": (0.0, 0.0), "b_": (2.0, 0.0), "c_": (-2.0, 0.0)}
FORWARD = np.array([0.4, 0.0, 0.0])
STILL = np.zeros(3)


@pytest.fixture(scope="module")
def policy() -> G1Policy:
    return G1Policy.load(LeggedG1.dt)


def _three() -> mujoco.MjModel:
    """Three G1s on a bare floor, each attached under its own prefix."""
    spec = mujoco.MjSpec()
    floor = spec.worldbody.add_geom()
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.size = [0.0, 0.0, 0.05]
    floor.contype = 1
    floor.conaffinity = 1
    robot = g1_spec()
    # taken before the attach, which keeps the parent's and warns about every difference
    spec.option.iterations = robot.option.iterations
    spec.option.ls_iterations = robot.option.ls_iterations
    spec.option.gravity = robot.option.gravity
    for prefix, (x, y) in PLACES.items():
        frame = spec.worldbody.add_frame()
        frame.pos = [x, y, 0.0]
        spec.attach(g1_spec(), prefix=prefix, frame=frame)
        add_housings(spec, G1_RIG, prefix)
    return spec.compile()


@pytest.fixture(scope="module")
def three(policy: G1Policy) -> tuple[dict[str, LeggedG1], mujoco.MjModel, mujoco.MjData]:
    model = _three()
    data = mujoco.MjData(model)
    robots = {p: LeggedG1(model, data, policy, prefix=p) for p in PLACES}
    return robots, model, data


def _stand(robots: dict[str, LeggedG1]) -> None:
    for prefix, (x, y) in PLACES.items():
        robots[prefix].reset(x, y, 0.0, 0.0)


def test_each_robot_reads_its_own_free_joint(
    three: tuple[dict[str, LeggedG1], mujoco.MjModel, mujoco.MjData],
) -> None:
    """The slices used to be module constants, which is right only for the first robot.

    A model with three G1s has three free joints at three qpos addresses. Read through a
    constant, robots two and three report robot one's pose and velocity -- the arrays are the
    right shape, so nothing raises and the sim just behaves strangely.
    """
    robots, model, _ = three
    starts = {prefix: robot.base.position.start for prefix, robot in robots.items()}

    assert len(set(starts.values())) == len(PLACES), f"two robots share an address: {starts}"
    assert all(len(robot.joint_names) == 29 for robot in robots.values())
    assert model.nu == 29 * len(PLACES)


def test_resetting_one_robot_leaves_the_others_where_they_stood(
    three: tuple[dict[str, LeggedG1], mujoco.MjModel, mujoco.MjData],
) -> None:
    """`mj_resetData` clears the whole model, so a robot may only write its own addresses."""
    robots, _, _ = three
    _stand(robots)
    before = {prefix: robot.base_pose()[0].copy() for prefix, robot in robots.items()}

    robots["b_"].reset(5.0, 5.0, 0.0, 0.0)

    assert np.allclose(robots["b_"].base_pose()[0][:2], [5.0, 5.0])
    for prefix in ("a_", "c_"):
        assert np.allclose(robots[prefix].base_pose()[0], before[prefix]), f"{prefix} moved"


def test_they_walk_independently_on_one_shared_policy(
    three: tuple[dict[str, LeggedG1], mujoco.MjModel, mujoco.MjData],
) -> None:
    """One told to walk, two told to stand, all three driven from the same loaded actor.

    The gait phase and the last action are per robot. Sharing one actor's state between them
    was measured at three falls in four seconds -- so the actor is cloned per robot, and this is
    the test that says the clone is real.

    The one told to walk covers 2.76 m, which is what it covers alone on a bare floor to the
    centimetre: two neighbours in the model cost it nothing. The two told to stand still creep
    about 0.5 m forward in the same four seconds, because this actor marches in place rather
    than standing, so the test is that walking and standing are far apart, not that standing is
    motionless.
    """
    robots, model, data = three
    _stand(robots)
    start = {prefix: robot.base_pose()[0].copy() for prefix, robot in robots.items()}

    for _ in range(200):  # 4 s at 50 Hz
        robots["a_"].control(FORWARD)
        robots["b_"].control(STILL)
        robots["c_"].control(STILL)
        for _ in range(robots["a_"].substeps):
            mujoco.mj_step(model, data)

    moved = {p: robots[p].base_pose()[0] - start[p] for p in PLACES}
    assert all(robot.upright() > 0.9 for robot in robots.values()), "one of them fell"
    assert moved["a_"][0] > 2.0, "the one told to walk did not"
    for prefix in ("b_", "c_"):
        assert moved[prefix][0] < moved["a_"][0] / 3, f"{prefix} walked without being told to"
        assert abs(moved[prefix][1]) < 0.5, f"{prefix} wandered sideways"
