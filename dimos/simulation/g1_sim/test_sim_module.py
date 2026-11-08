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


"""The contract between the world and a robot's controller, when they are separate modules."""

from __future__ import annotations

from collections.abc import Iterator
from threading import Thread
import time

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.simulation.g1_sim.policy import DEFAULT_POSE
from dimos.simulation.g1_sim.sim_module import G1Sim
from dimos.simulation.g1_sim.world import G1_RIG, SimG1Many
from dimos.simulation.scenes.procedural import Scene, office
from dimos.simulation.sim_world.mujoco_sim import base_odometry
from dimos.simulation.sim_world.shared import Placement, SharedWorld

pytestmark = pytest.mark.self_hosted

MAIN, OTHER = "main_", "other_"


@pytest.fixture(scope="module")
def scene() -> Scene:
    return office(1)


@pytest.fixture
def world(scene: Scene) -> SharedWorld:
    x, y, _ = scene.start
    world = SharedWorld(
        scene,
        1,
        [Placement(MAIN, G1_RIG, (x, y, 0.0)), Placement(OTHER, G1_RIG, (x + 1.5, y, 0.0))],
    )
    world.reset()
    return world


@pytest.fixture
def controller() -> Iterator[G1Sim]:
    """A controller module. Its threads are up from construction, so it is stopped either way."""
    module = G1Sim(prefix=MAIN)
    yield module
    module.stop()


@pytest.fixture
def sim(request: pytest.FixtureRequest, world: SharedWorld) -> Iterator[SimG1Many]:
    """A world module wrapped round an already-built SharedWorld, stopped on the way out."""
    deadline = getattr(request, "param", 1.0)
    module = SimG1Many(
        robots={MAIN: (0.0, 0.0, 0.0), OTHER: (1.5, 0.0, 0.0)}, control_deadline=deadline
    )
    module._world = world
    yield module
    module.stop()


def test_the_controller_sees_what_the_in_loop_robot_saw(
    world: SharedWorld, controller: G1Sim
) -> None:
    """The whole design rests on this: two messages carry the actor's observation intact.

    In the single-robot sim the actor reads MjData directly. Here it gets an Odometry and a
    JointState off the bus and rebuilds the observation from them, so if the two disagree the
    robot is being driven on something other than what it feels -- and a gait is sensitive
    enough that it would show up as a fall with nothing to point at.
    """
    for _ in range(25):  # half a second, so nothing is still at rest
        world.tick()
    occupant = world.occupants[MAIN]
    positions, velocities = occupant.chassis.joint_state()
    truth = occupant.chassis.observe()

    rebuilt = controller._observe(base_odometry(occupant, MAIN, world.t), positions, velocities)

    assert np.allclose(rebuilt.linear_velocity, truth.linear_velocity, atol=1e-9)
    assert np.allclose(rebuilt.angular_velocity, truth.angular_velocity, atol=1e-9)
    assert np.allclose(rebuilt.gravity, truth.gravity, atol=1e-9)
    assert np.allclose(rebuilt.joint_position, truth.joint_position)
    assert np.allclose(rebuilt.joint_velocity, truth.joint_velocity)


def test_the_world_routes_targets_by_the_prefix_on_the_joint_names(
    world: SharedWorld, sim: SimG1Many
) -> None:
    """Every robot answers on one topic, so the names are what say whose targets these are."""
    names = list(world.occupants[OTHER].chassis.joint_names)
    bent = DEFAULT_POSE.copy()
    bent[0] += 0.4

    sim._on_target(JointState(ts=world.t, name=names, position=bent.tolist(), velocity=[]))

    assert set(sim._targets) == {OTHER}, "the targets went to the wrong robot"
    assert sim._targets[OTHER][0] == world.t
    assert np.allclose(sim._targets[OTHER][1], bent)


def test_a_target_for_nobody_here_is_refused(world: SharedWorld, sim: SimG1Many) -> None:
    """A stray message must not be applied to whichever robot happens to match first."""
    sim._on_target(JointState(ts=world.t, name=["someone_else_knee"], position=[0.0], velocity=[]))

    assert sim._targets == {}


@pytest.mark.parametrize("sim", [0.2], indirect=True)
def test_a_silent_controller_has_its_tick_counted_as_a_miss(
    world: SharedWorld, sim: SimG1Many
) -> None:
    """Lockstep with a deadline: a controller that never answers must not wedge the world.

    The actuators hold what they were last given, which is what a real one does, and the miss
    is counted so a run that was quietly limping can be told from one that was not.
    """
    names = list(world.occupants[MAIN].chassis.joint_names)
    sim._on_target(JointState(ts=1.0, name=names, position=DEFAULT_POSE.tolist(), velocity=[]))

    began = time.monotonic()
    sim._await_targets(1.0)
    waited = time.monotonic() - began

    assert waited >= 0.2, "it did not wait for the missing controller"
    assert sim.misses() == {OTHER: 1}


@pytest.mark.parametrize("sim", [10.0], indirect=True)
def test_the_wait_ends_as_soon_as_the_last_robot_answers(
    world: SharedWorld, sim: SimG1Many
) -> None:
    """The deadline is a timeout, not a pace: a tick ends the moment everyone has answered."""
    for prefix in (MAIN, OTHER):
        names = list(world.occupants[prefix].chassis.joint_names)
        message = JointState(ts=2.0, name=names, position=DEFAULT_POSE.tolist(), velocity=[])
        if prefix == MAIN:
            sim._on_target(message)
        else:
            Thread(
                target=lambda m=message: (time.sleep(0.1), sim._on_target(m)), daemon=True
            ).start()

    began = time.monotonic()
    sim._await_targets(2.0)
    waited = time.monotonic() - began

    assert 0.05 < waited < 5.0, f"it waited {waited:.2f}s rather than for the second answer"
    assert sim.misses() == {}
