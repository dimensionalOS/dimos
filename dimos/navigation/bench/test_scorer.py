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
from typing import get_args

import numpy as np
import pytest

from dimos.msgs.sim_msgs.Contacts import Contact
from dimos.navigation.bench.ground_truth import GO2
from dimos.navigation.bench.scorer import (
    Commands,
    ContactSample,
    Outcome,
    PathSample,
    Poses,
    Recording,
    definitions,
    score,
)
from dimos.navigation.bench.suite import Rules
from dimos.simulation.scenes.procedural import room

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
    assert s.feature_m is None
    beside = score(_walk(10.0), RULES, GOAL, feature=((START[0] + GOAL[0]) / 2, START[1] + 0.3))
    assert beside.feature_m == pytest.approx(0.3, abs=0.02)
    assert s.outcome == "success" and s.signature is None and s.signatures == []
    assert s.window == (T0, T0 + 9.9) and s.arrived_s == pytest.approx(9.9)
    assert s.traveled_m == pytest.approx(4.0, abs=0.05) and s.spl == pytest.approx(1.0, abs=0.02)
    assert s.final_error_xy < 0.05 and s.final_xy == pytest.approx((5.0, 1.0), abs=0.05)
    assert (s.collisions, s.reroutes, s.yaw_reversals, s.fell, s.missing) == (0, 0, 0, False, [])
    assert (s.reached, s.claimed) == (True, True)
    assert (s.contacts, s.first_contact, s.first_contact_s) == ({}, None, None)
    assert s.empty_path_s == pytest.approx(0.5)


def _trunk_on_floor(seconds: float) -> Recording:
    return replace(
        _walk(10.0),
        contacts=[
            ContactSample(T0 - 5.0, [Contact("foot", "floor")]),
            ContactSample(T0 + 3.0, [Contact("foot", "floor"), Contact("trunk", "floor")]),
            ContactSample(T0 + 3.0 + seconds, [Contact("foot", "floor")]),
        ],
    )


def test_every_outcome_is_defined_with_the_limits_of_the_rules() -> None:
    meanings = definitions(replace(RULES, stuck_s=7.0, stuck_moving_fraction=0.6))
    assert list(meanings) == list(get_args(Outcome))
    assert "at least 60% of the last 7 s" in meanings["stuck"]
    assert "{" not in "".join(meanings.values())


def test_a_brief_trunk_touch_is_not_a_fall() -> None:
    touched = score(_trunk_on_floor(0.06), RULES, GOAL)
    assert (touched.outcome, touched.fell) == ("success", False)
    assert touched.trunk_floor_s == pytest.approx(0.06)
    lying = score(_trunk_on_floor(RULES.fall_trunk_floor_s + 0.1), RULES, GOAL)
    assert (lying.outcome, lying.signature) == ("fall", "body_fell")


def test_standing_on_the_goal_without_reporting_arrival_is_unclaimed() -> None:
    for command_last in (True, False):
        silent = _walk(30.0, arrive=False, hold_last=15.0, command_last=command_last)
        s = score(silent, RULES, GOAL, route_length=4.0)
        assert (s.outcome, s.signature) == ("unclaimed", "arrival_mismatch")
        assert (s.reached, s.claimed, s.spl) == (True, False, 0.0)
    lying = replace(silent, contacts=_trunk_on_floor(1.0).contacts)
    assert (score(lying, RULES, GOAL).outcome, score(lying, RULES, GOAL).reached) == ("fall", False)


def test_a_recording_that_shows_no_outcome_takes_the_drivers_verdict() -> None:
    walking = _walk(5.0, arrive=False)
    alone = score(walking, RULES, GOAL, route_length=4.0)
    assert (alone.outcome, alone.mismatch) == ("timeout", False)
    told = score(walking, RULES, GOAL, route_length=4.0, verdict="stuck")
    assert (told.outcome, told.signature, told.mismatch) == ("stuck", "commanded_no_progress", True)
    with pytest.raises(ValueError, match="arrived is not in the recording"):
        score(walking, RULES, GOAL, route_length=4.0, verdict="arrived")


def test_a_verdict_the_recording_contradicts_is_a_mismatch() -> None:
    def mismatch(recording: Recording, verdict: str) -> tuple[str, bool]:
        s = score(recording, RULES, GOAL, verdict=verdict)
        return s.outcome, s.mismatch

    assert mismatch(_walk(10.0), "arrived") == ("success", False)
    assert mismatch(_walk(10.0), "stuck") == ("success", True)
    assert mismatch(_trunk_on_floor(1.0), "arrived") == ("fall", True)
    silent = _walk(30.0, arrive=False, hold_last=15.0)
    assert mismatch(silent, "stuck") == ("unclaimed", False)
    stalled = _walk(30.0, arrive=False, hold_last=25.0, command_last=False)
    assert mismatch(stalled, "stalled") == ("stalled", False)
    assert mismatch(stalled, "stuck") == ("stalled", True)


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
    assert (s.contacts, s.first_contact) == ({"leg clutter": 1}, "leg clutter")
    assert s.first_contact_s == pytest.approx(3.0)
    assert s.first_contact_speed == pytest.approx(SPEED, abs=0.01)
    far_off = score(_walk(10.0), RULES, (7.0, 1.0, 0.0))
    assert (far_off.outcome, far_off.reached, far_off.claimed) == ("wrong_place", False, True)
    assert score(_walk(10.0), RULES, (5.0, 1.0, 1.0)).outcome == "wrong_floor"
    tipped = _walk(10.0)
    rpy = tipped.pose.rpy.copy()
    rpy[-10:, 1] = 1.4
    tipped = replace(tipped, pose=replace(tipped.pose, rpy=rpy))
    assert score(tipped, RULES, GOAL).signature == "body_fell"
    stuck = _walk(30.0, arrive=False, hold_last=25.0)
    assert score(stuck, RULES, GOAL).signature == "commanded_no_progress"
    stalled = _walk(30.0, arrive=False, hold_last=25.0, command_last=False)
    assert score(stalled, RULES, GOAL).outcome == "stalled"
    refused = replace(stalled, local_paths=[PathSample(T0 + 20.0, np.array([START]))])
    assert score(refused, RULES, GOAL).signature == "local_refused"
    assert score(refused, RULES, GOAL).signatures == ["local_refused"]
    followed = replace(stalled, local_paths=[PathSample(T0 + 20.0, np.array([START, [*GOAL]]))])
    assert score(followed, RULES, GOAL).signature == "follower_zeroed"
    empty = replace(
        _walk(30.0, arrive=False, hold_last=25.0),
        planner_paths=[PathSample(T0 + 0.5, np.zeros((0, 3)))],
    )
    assert score(empty, RULES, GOAL).signature == "planner_empty"
    far = (40.0, 1.0, 0.0)
    late = replace(_walk(60.0, arrive=False, goal=far), arrivals=[T0 + 55.0])
    s = score(late, RULES, far, route_length=4.0)
    assert s.signature == "out_of_time" and s.window == (T0, T0 + RULES.timeout_s(4.0))


def test_margin_is_the_gap_to_the_nearest_obstacle_at_body_height() -> None:
    assert score(_walk(10.0), RULES, GOAL).margin_m is None
    scene = room(1, width=8.0, length=6.0)
    z0 = scene.start[2]
    open_floor = score(_walk(10.0), RULES, GOAL, scene=scene)
    assert open_floor.margin_m == pytest.approx(1.0 - GO2.radius)
    assert open_floor.margin_s == 0.0
    assert open_floor.final_margin_m == pytest.approx(1.0 - GO2.radius, abs=0.02)
    pose = _walk(10.0).pose
    by_wall = replace(_walk(10.0), pose=replace(pose, xyz=pose.xyz - [0.0, 0.8, 0.0]))
    assert score(by_wall, RULES, GOAL, scene=scene).margin_m == pytest.approx(0.2 - GO2.radius)
    scene.add((3.0, 1.3, z0), (3.3, 1.6, z0 + 0.1), "clutter")
    scene.add((2.0, 0.5, z0 + 0.7), (2.5, 1.5, z0 + 0.8), "clutter")
    past_a_block = score(_walk(10.0), RULES, GOAL, scene=scene)
    assert past_a_block.margin_m == pytest.approx(0.3 - GO2.radius)
    assert past_a_block.margin_s == pytest.approx(5.0, abs=0.05)
    scene.add((4.0, 0.5, z0 + 0.5), (4.5, 1.5, z0 + 0.55), "clutter")
    under_a_slab = score(_walk(10.0), RULES, GOAL, scene=scene)
    assert under_a_slab.margin_m == pytest.approx(-GO2.radius)
    assert under_a_slab.margin_s == pytest.approx(7.0, abs=0.05)
    assert under_a_slab.final_margin_m == pytest.approx(0.5 - GO2.radius, abs=0.02)


def _stuck_touching(seconds: float) -> Recording:
    """A stuck walk with a leg on clutter for the given part of the final stuck window."""
    start = T0 + 30.0 - RULES.stuck_s
    return replace(
        _walk(30.0, arrive=False, hold_last=25.0),
        contacts=[
            ContactSample(T0 - 5.0, [Contact("foot", "floor")]),
            ContactSample(start - 3.0, [Contact("leg", "clutter"), Contact("foot", "floor")]),
            ContactSample(start + 1.0, [Contact("leg", "clutter"), Contact("trunk", "wall")]),
            ContactSample(start + 2.0, [Contact("leg", "clutter")]),
            ContactSample(start + seconds, [Contact("foot", "floor")]),
        ],
    )


def test_a_stuck_body_held_against_an_object_is_pushing() -> None:
    held = RULES.stuck_s * 0.5
    s = score(_stuck_touching(held + 0.1), RULES, GOAL)
    assert s.signatures == ["commanded_no_progress", "pushing"]
    assert s.signature == "commanded_no_progress"
    assert s.pushing_s == pytest.approx(held + 0.1) and s.pushing_on == "leg clutter"
    brief = score(_stuck_touching(held - 0.1), RULES, GOAL)
    assert brief.signatures == ["commanded_no_progress"]
    assert brief.pushing_s == pytest.approx(held - 0.1) and brief.pushing_on == "leg clutter"
    free = score(_walk(30.0, arrive=False, hold_last=25.0), RULES, GOAL)
    assert (free.signatures, free.pushing_s, free.pushing_on) == (
        ["commanded_no_progress"],
        0.0,
        None,
    )
    stalled = replace(
        _walk(30.0, arrive=False, hold_last=25.0, command_last=False),
        contacts=_stuck_touching(held + 0.1).contacts,
    )
    assert score(stalled, RULES, GOAL).signatures == []
    unrecorded = score(replace(_stuck_touching(held + 0.1), contacts=None), RULES, GOAL)
    assert (unrecorded.signatures, unrecorded.pushing_s, unrecorded.pushing_on) == (
        ["commanded_no_progress"],
        None,
        None,
    )
