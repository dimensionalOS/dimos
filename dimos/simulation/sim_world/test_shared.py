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


"""Three G1s in one world: one model, one step, and each robot's own lidar frame."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.simulation.g1_sim.policy import DEFAULT_POSE
from dimos.simulation.g1_sim.world import G1_RIG
from dimos.simulation.scenes.procedural import Scene, office
from dimos.simulation.sim_world.shared import Placement, SharedWorld
from dimos.simulation.sim_world.world import FRAME_DT

pytestmark = pytest.mark.self_hosted

APART = 1.5


@pytest.fixture(scope="module")
def scene() -> Scene:
    return office(1)


@pytest.fixture
def world(scene: Scene) -> SharedWorld:
    x, y, _ = scene.start
    world = SharedWorld(
        scene,
        1,
        [
            Placement("a_", G1_RIG, (x, y, 0.0)),
            Placement("b_", G1_RIG, (x + APART, y, 0.0)),
            Placement("c_", G1_RIG, (x, y + APART, 0.0)),
        ],
    )
    world.reset()
    return world


def test_three_robots_compose_into_one_model(world: SharedWorld) -> None:
    """One model, three of everything, and nobody standing inside the furniture."""
    assert world.model.nu == 29 * 3
    assert len(world.occupants) == 3
    assert all(len(occupant.chassis.joint_names) == 29 for occupant in world.occupants.values())
    for prefix in world.occupants:
        assert world.contacts(prefix) == [], f"{prefix} starts inside something"


def test_one_step_moves_every_robot(world: SharedWorld) -> None:
    """A shared MjData can only be stepped by the world, so the world is what steps it.

    Each robot is handed a different target and all three react to the same `tick`. Were the
    robots stepping for themselves, the first would carry the others along at its own rate.
    """
    bent = DEFAULT_POSE.copy()
    bent[0] += 0.3
    world.apply("a_", bent)
    world.apply("b_", DEFAULT_POSE)
    world.apply("c_", DEFAULT_POSE)
    before = {p: o.chassis.joint_state()[0].copy() for p, o in world.occupants.items()}

    for _ in range(10):
        world.tick()

    after = {p: o.chassis.joint_state()[0] for p, o in world.occupants.items()}
    assert abs(after["a_"][0] - before["a_"][0]) > 0.05, "the one that was bent did not move"
    assert world.t == pytest.approx(10 * world.dt)


def test_every_robot_gets_its_own_frame_at_10hz(world: SharedWorld) -> None:
    """One lidar frame per robot per FRAME_DT, with that robot's own returns in it.

    Counting frames is not enough on its own: a robot whose lidar was never cast still gets a
    frame handed back, an empty one, so the points are what says the cast happened.
    """
    frames: dict[str, int] = dict.fromkeys(world.occupants, 0)
    points: dict[str, int] = dict.fromkeys(world.occupants, 0)
    for _ in range(world.ticks_per_frame * 2):
        for prefix, frame in world.tick().items():
            if frame is not None:
                frames[prefix] += 1
                points[prefix] += len(frame.points)

    assert frames == dict.fromkeys(world.occupants, 2)
    assert all(count > 1000 for count in points.values()), f"a lidar never fired: {points}"
    assert world.t == pytest.approx(2 * FRAME_DT)


def test_each_robot_can_see_the_others(world: SharedWorld) -> None:
    """The point of sharing a world rather than running three of them side by side.

    Three separate sims would each hold one robot and an empty patch of floor where the others
    stand. Here the neighbours are in the model, so they turn up in the cloud -- `a_` has one
    1.5 m to its east and another 1.5 m to its north, and returns land on both.
    """
    for _ in range(world.ticks_per_frame):
        frames = world.tick()
    frame = frames["a_"]
    assert frame is not None

    origin, rotation = world.occupants["a_"].odom_pose()
    world_points = origin + frame.points.astype(np.float64) @ rotation.T
    for prefix in ("b_", "c_"):
        neighbour, _ = world.occupants[prefix].chassis.base_pose()
        near = np.linalg.norm(world_points[:, :2] - neighbour[:2], axis=1) < 0.5
        assert near.sum() > 50, f"a_ saw only {int(near.sum())} returns off {prefix}"


def test_a_rig_that_cannot_share_a_world_says_so(scene: Scene) -> None:
    """A Rig without a chassis can still have a world built around it alone, but not share one."""
    from dataclasses import replace

    with pytest.raises(ValueError, match="no chassis"):
        Placement("a_", replace(G1_RIG, chassis=None), (0.0, 0.0, 0.0))


def test_two_robots_may_not_share_a_prefix(scene: Scene) -> None:
    """Their names would collide in the model, which MuJoCo reports much later and obscurely."""
    x, y, _ = scene.start
    with pytest.raises(ValueError, match="share a prefix"):
        SharedWorld(
            scene,
            1,
            [Placement("a_", G1_RIG, (x, y, 0.0)), Placement("a_", G1_RIG, (x + APART, y, 0.0))],
        )
