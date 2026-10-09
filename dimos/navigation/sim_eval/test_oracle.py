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

import math

import numpy as np
import pytest

from dimos.navigation.sim_eval.ground_truth import GroundTruth
from dimos.navigation.sim_eval.oracle import RouteTracker, Tracking, walk
from dimos.simulation.go2_legged.policy import OnnxGo2Policy
from dimos.simulation.go2_sim.world import Go2Sim
from dimos.simulation.scenes.procedural import office

STRAIGHT = np.array([[x, 0.0, 0.0] for x in np.arange(0.0, 5.05, 0.05)])


def test_tracker_drives_forward_along_the_route_and_turns_toward_it() -> None:
    tracker = RouteTracker(STRAIGHT)
    command, arrived = tracker.step(0.0, 0.0, 0.0)
    assert not arrived
    assert command[0] == pytest.approx(Tracking().speed)
    assert command[2] == pytest.approx(0.0)
    command, _ = tracker.step(1.0, -0.5, 0.0)
    assert command[2] > 0.0
    command, _ = tracker.step(1.0, 0.0, math.pi / 2)
    assert command[2] < 0.0
    command, _ = tracker.step(1.0, 0.0, math.pi)
    assert command[0] == 0.0 and command[2] != 0.0


def test_tracker_arrives_at_the_end_and_never_backs_up() -> None:
    tracker = RouteTracker(STRAIGHT)
    tracker.step(4.0, 0.0, 0.0)
    command, _ = tracker.step(3.0, 0.0, 0.0)
    assert command[0] > 0.0
    command, arrived = tracker.step(4.8, 0.0, 0.0)
    assert arrived
    assert np.all(command == 0.0)


@pytest.mark.self_hosted
def test_walk_reaches_the_next_room() -> None:
    scene = office(1)
    gt = GroundTruth(scene)
    xs = gt.origin[0] + (np.arange(gt.shape[0]) + 0.5) * gt.cell
    far_room = gt.walkable & (xs[:, None] > 0.6 * scene.params["width"])
    goal = gt.center(*np.unravel_index(np.argmax(np.where(far_room, gt.clearance, -1)), gt.shape))
    route = gt.route(scene.start, goal, centered=True)
    assert route is not None
    sim = Go2Sim(scene, seed=1, policy=OnnxGo2Policy.load())
    sim.reset(*scene.start, 0.0)
    assert walk(sim, route.points, timeout_s=20.0 + 6.0 * route.length)
    position, _ = sim.base_pose()
    assert np.linalg.norm(position[:2] - goal[:2]) < 0.5
    assert all(c.kind == "floor" for c in sim.contacts())
