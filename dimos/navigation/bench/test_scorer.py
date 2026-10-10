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

from dataclasses import replace

import numpy as np
import pytest

from dimos.msgs.sim_msgs.Contacts import Contact
from dimos.navigation.bench.scorer import (
    Commands,
    ContactSample,
    PathSample,
    Poses,
    Recording,
    score,
)
from dimos.navigation.bench.suite import Rules

RULES = Rules()
START = np.array([1.0, 1.0, 0.3])
GOAL = (5.0, 1.0, 0.0)
T0 = 100.0
SPEED = 0.5


def _walk(
    seconds: float,
    arrive: bool = True,
    hold_last: float = 0.0,
    command_last: bool = True,
    goal: tuple[float, float, float] = GOAL,
) -> Recording:
    """A straight walk toward the goal at SPEED, with the goal echoed and the stack commanding."""
    t = T0 + np.arange(0, seconds, 0.02)
    distance = np.hypot(*(np.array(goal[:2]) - START[:2]))
    moving = np.clip(t - T0 - 1.0, 0.0, min(seconds - hold_last - 1.0, distance / SPEED))
    xyz = START + np.outer(moving * SPEED, [1.0, 0.0, 0.0])
    tc = T0 + np.arange(0, seconds, 0.1)
    v = np.tile([SPEED, 0.0, 0.0], (len(tc), 1))
    if not command_last:
        v[tc > T0 + seconds - hold_last] = 0.0
    return Recording(
        end=T0 + seconds,
        pose=Poses(t, xyz, np.zeros((len(t), 3))),
        contacts=[ContactSample(T0 - 5.0, [Contact("foot", "floor")])],
        goals=[(T0 - 20.0, (9.0, 9.0, 0.3)), (T0, goal), (T0 + 1.0, goal)],
        planner_paths=[PathSample(T0 + 0.5, np.array([START, [*goal]]))],
        commands=Commands(tc, v),
        arrivals=[T0 + seconds - 0.1] if arrive else [],
    )


def test_a_clean_arrival_is_a_success_with_full_metrics() -> None:
    s = score(_walk(10.0), RULES, GOAL, route_length=4.0)
    assert s.outcome == "success" and s.signature is None
    assert s.window == (T0, T0 + 9.9) and s.arrived_s == pytest.approx(9.9)
    assert s.traveled_m == pytest.approx(4.0, abs=0.05) and s.spl == pytest.approx(1.0, abs=0.02)
    assert s.final_error_xy < 0.05 and s.final_xy == pytest.approx((5.0, 1.0), abs=0.05)
    assert (s.collisions, s.reroutes, s.yaw_reversals, s.fell, s.missing) == (0, 0, 0, False, [])
    assert s.empty_path_s == pytest.approx(0.5)


def test_every_failure_has_an_outcome_and_a_signature() -> None:
    bumped = replace(
        _walk(10.0),
        contacts=[
            ContactSample(T0 - 5.0, [Contact("foot", "floor")]),
            ContactSample(T0 + 3.0, [Contact("foot", "floor"), Contact("leg", "clutter")]),
            ContactSample(T0 + 4.5, [Contact("foot", "floor")]),
        ],
    )
    s = score(bumped, RULES, GOAL, route_length=4.0)
    assert (s.outcome, s.collisions, s.collision_s, s.spl) == (
        "reached_with_collision",
        1,
        1.5,
        0.0,
    )
    assert score(_walk(10.0), RULES, (7.0, 1.0, 0.0)).outcome == "wrong_place"
    assert score(_walk(10.0), RULES, (5.0, 1.0, 1.0)).outcome == "wrong_floor"
    tipped = _walk(10.0)
    rpy = tipped.pose.rpy.copy()
    rpy[-10:, 1] = 1.4
    tipped = replace(tipped, pose=replace(tipped.pose, rpy=rpy))
    assert score(tipped, RULES, GOAL).signature == "body_fell"
    stuck = _walk(30.0, arrive=False, hold_last=15.0)
    assert score(stuck, RULES, GOAL).signature == "commanded_no_progress"
    stalled = _walk(30.0, arrive=False, hold_last=15.0, command_last=False)
    assert score(stalled, RULES, GOAL).outcome == "stalled"
    refused = replace(stalled, local_paths=[PathSample(T0 + 20.0, np.array([START]))])
    assert score(refused, RULES, GOAL).signature == "local_refused"
    followed = replace(stalled, local_paths=[PathSample(T0 + 20.0, np.array([START, [*GOAL]]))])
    assert score(followed, RULES, GOAL).signature == "follower_zeroed"
    empty = replace(
        _walk(30.0, arrive=False), planner_paths=[PathSample(T0 + 0.5, np.zeros((0, 3)))]
    )
    assert score(empty, RULES, GOAL).signature == "planner_empty"
    far = (40.0, 1.0, 0.0)
    late = replace(_walk(60.0, arrive=False, goal=far), arrivals=[T0 + 55.0])
    s = score(late, RULES, far, route_length=4.0)
    assert s.signature == "out_of_time" and s.window == (T0, T0 + RULES.timeout_s(4.0))
