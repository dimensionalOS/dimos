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
from pathlib import Path

import numpy as np
import pytest

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path as PathMsg
from dimos.msgs.sim_msgs.Contacts import Contact, Contacts
from dimos.msgs.std_msgs.Bool import Bool
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
    pose = Poses(t, xyz, np.zeros((len(t), 3)))
    tc = T0 + np.arange(0, seconds, 0.1)
    v = np.tile([SPEED, 0.0, 0.0], (len(tc), 1))
    if not command_last:
        v[tc > T0 + seconds - hold_last] = 0.0
    path = np.array([START, [*goal]])
    return Recording(
        end=T0 + seconds,
        pose=pose,
        contacts=[ContactSample(T0 - 5.0, [Contact("foot", "floor")])],
        goals=[(T0, goal), (T0 + 1.0, goal), (T0 + 2.0, goal)],
        planner_paths=[PathSample(T0 + 0.5, path)],
        commands=Commands(tc, v),
        arrivals=[T0 + seconds - 0.1] if arrive else [],
    )


def test_a_clean_arrival_is_a_success_with_full_spl() -> None:
    s = score(_walk(10.0), RULES, GOAL, route_length=4.0)
    assert s.outcome == "success"
    assert s.window == (T0, T0 + 9.9)
    assert s.arrived_s == pytest.approx(9.9)
    assert s.traveled_m == pytest.approx(4.0, abs=0.05)
    assert s.spl == pytest.approx(1.0, abs=0.02)
    assert s.final_error_xy < 0.05
    assert s.collisions == 0
    assert s.reroutes == 0
    assert s.empty_path_s == pytest.approx(0.5)
    assert s.yaw_reversals == 0
    assert s.fell is False
    assert s.missing == []


def test_the_window_starts_at_the_first_goal_echo() -> None:
    rec = _walk(10.0)
    rec = replace(rec, goals=[(T0 - 20.0, (9.0, 9.0, 0.3)), *rec.goals])
    assert score(rec, RULES, GOAL).window[0] == T0


def test_a_collision_on_the_way_counts_against_the_arrival() -> None:
    rec = replace(
        _walk(10.0),
        contacts=[
            ContactSample(T0 - 5.0, [Contact("foot", "floor")]),
            ContactSample(T0 + 3.0, [Contact("foot", "floor"), Contact("leg", "clutter")]),
            ContactSample(T0 + 4.5, [Contact("foot", "floor")]),
        ],
    )
    s = score(rec, RULES, GOAL, route_length=4.0)
    assert s.outcome == "reached_with_collision"
    assert s.collisions == 1
    assert s.collision_s == pytest.approx(1.5)
    assert s.spl == 0.0


def test_a_false_arrival_is_wrong_place_or_wrong_floor() -> None:
    assert score(_walk(10.0), RULES, (7.0, 1.0, 0.0)).outcome == "wrong_place"
    assert score(_walk(10.0), RULES, (5.0, 1.0, 1.0)).outcome == "wrong_floor"


def test_tipping_over_is_a_fall_even_after_arriving() -> None:
    rec = _walk(10.0)
    rpy = rec.pose.rpy.copy()
    rpy[-10:, 1] = 1.4
    assert score(replace(rec, pose=replace(rec.pose, rpy=rpy)), RULES, GOAL).outcome == "fall"
    rec = replace(rec, contacts=[ContactSample(T0 + 5.0, [Contact("trunk", "floor")])])
    s = score(rec, RULES, GOAL)
    assert s.outcome == "fall"
    assert s.trunk_floor_s == pytest.approx(4.9)


def test_no_path_ever_is_no_plan() -> None:
    rec = replace(_walk(30.0, arrive=False), planner_paths=[PathSample(T0 + 0.5, np.zeros((0, 3)))])
    s = score(rec, RULES, GOAL)
    assert s.outcome == "no_plan"
    assert s.empty_path_s == pytest.approx(30.0)


def test_commanding_without_moving_is_stuck() -> None:
    s = score(_walk(30.0, arrive=False, hold_last=15.0), RULES, GOAL)
    assert s.outcome == "stuck"


def test_commanding_nothing_is_stalled() -> None:
    s = score(_walk(30.0, arrive=False, hold_last=15.0, command_last=False), RULES, GOAL)
    assert s.outcome == "stalled"


def test_running_out_of_time_is_timeout_and_caps_the_window() -> None:
    far = (40.0, 1.0, 0.0)
    rec = _walk(60.0, arrive=False, goal=far)
    s = score(rec, RULES, far, route_length=4.0)
    assert s.outcome == "timeout"
    assert s.window == (T0, T0 + RULES.timeout_s(4.0))
    late = replace(rec, arrivals=[T0 + 55.0])
    assert score(late, RULES, far, route_length=4.0).outcome == "timeout"


def test_reroutes_and_path_change_come_from_consecutive_paths() -> None:
    rec = _walk(10.0)
    straight = np.array([START, [*GOAL]])
    bent = np.array([START, [3.0, 2.0, 0.3], [*GOAL]])
    rec = replace(
        rec,
        planner_paths=[
            PathSample(T0 + 0.5, straight),
            PathSample(T0 + 2.0, bent),
            PathSample(T0 + 3.0, bent),
            PathSample(T0 + 4.0, np.zeros((0, 3))),
            PathSample(T0 + 5.0, straight),
        ],
    )
    s = score(rec, RULES, GOAL)
    assert s.reroutes == 1
    assert s.path_change_max == pytest.approx(1.0)
    assert s.empty_path_s == pytest.approx(1.5)


def test_a_repeated_vertex_does_not_poison_the_path_change() -> None:
    doubled = np.array([START, START, [*GOAL]])
    bent = np.array([START, [3.0, 2.0, 0.3], [*GOAL]])
    rec = replace(
        _walk(10.0), planner_paths=[PathSample(T0 + 0.5, doubled), PathSample(T0 + 2.0, bent)]
    )
    s = score(rec, RULES, GOAL)
    assert s.path_change_max == pytest.approx(1.0)
    assert s.reroutes == 1


def test_yaw_reversals_count_sign_flips_of_the_turn_command() -> None:
    rec = _walk(10.0)
    v = rec.commands.v.copy()
    v[:, 2] = np.where(np.arange(len(v)) // 10 % 2 == 0, 0.5, -0.5)
    s = score(replace(rec, commands=Commands(rec.commands.t, v)), RULES, GOAL)
    assert s.yaw_reversals == 9


def test_missing_streams_are_not_measured() -> None:
    rec = replace(_walk(10.0), contacts=None, commands=None, planner_paths=None)
    s = score(rec, RULES, GOAL)
    assert s.outcome == "success"
    assert s.collisions is None and s.collision_s is None and s.fell is False
    assert s.reroutes is None and s.empty_path_s is None and s.yaw_reversals is None
    assert s.missing == ["contacts", "planner_path", "cmd_vel"]
    assert score(replace(rec, arrivals=None), RULES, GOAL).outcome == "timeout"


def test_recording_reads_every_stream_from_a_store(tmp_path: Path) -> None:
    store = SqliteStore(path=str(tmp_path / "memory.db"))
    store.start()
    poses = store.stream("ground_truth", PoseStamped)
    for i in range(3):
        poses.append(
            PoseStamped(1.0 + i, 1.0, 0.3, 0, 0, 0, 1, ts=T0 + i, frame_id="odom"), ts=T0 + i
        )
    store.stream("goal", PointStamped).append(PointStamped(*GOAL, ts=T0, frame_id="odom"), ts=T0)
    path = PathMsg(ts=T0, frame_id="odom", poses=[PoseStamped(*START, 0, 0, 0, 1, ts=T0)])
    store.stream("planner_path", PathMsg).append(path, ts=T0)
    store.stream("cmd_vel", Twist).append(Twist(Vector3(0.5, 0.0, 0.0), Vector3(0, 0, 0.1)), ts=T0)
    store.stream("goal_reached", Bool).append(Bool(True), ts=T0 + 2)
    store.stream("contacts", Contacts).append(Contacts([Contact("foot", "floor")], ts=T0), ts=T0)
    store.stop()
    rec = Recording.from_store(tmp_path / "memory.db")
    assert rec.end == T0 + 2
    assert rec.pose.xyz[-1].tolist() == [3.0, 1.0, 0.3]
    assert rec.goals == [(T0, GOAL)]
    assert rec.planner_paths[0].points.tolist() == [list(START)]
    assert rec.commands.v.tolist() == [[0.5, 0.0, 0.1]]
    assert rec.arrivals == [T0 + 2]
    assert rec.contacts[0].contacts == [Contact("foot", "floor")]
    assert score(rec, RULES, GOAL).outcome == "wrong_place"
