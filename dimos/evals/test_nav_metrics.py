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

import math

import pytest

from dimos.evals.agents.topic import point_in
from dimos.evals.nav_metrics import (
    NavParams,
    box_of,
    bumps,
    read_cmds,
    read_declared,
    read_poses,
    score_navigation,
    turn_reversals,
)

BOX = box_of((5.0, 0.0), (1.0, 1.0))
END = (4.0, 0.0)  # beside the box, facing +x looks at it


def _straight(
    n: int, speed: float = 0.5, dt: float = 0.1
) -> list[tuple[float, float, float, float]]:
    return [(i * dt, i * dt * speed, 0.0, 0.0) for i in range(n + 1)]


def test_straight_run_reaches_and_faces() -> None:
    poses = _straight(80)  # ends at x = 4.0
    cmds = [(t, 0.5, 0.0, 0.0) for t, *_ in poses]
    m = score_navigation(poses, cmds, END, BOX, NavParams(success_radius_m=1.0))
    assert m.reached and m.facing and m.bumps == 0 and m.turn_reversals == 0
    assert m.time_to_object_s == pytest.approx(6.0)  # x = 3.0 at t = 6
    assert m.path_length_m == pytest.approx(4.0)
    assert m.straightness == pytest.approx(1.0)
    assert m.score() == pytest.approx(1.0)


def test_not_reached_scores_zero_and_reports_duration() -> None:
    m = score_navigation(_straight(20), [], END, BOX)
    assert not m.reached and m.score() == 0.0
    assert m.time_to_object_s == pytest.approx(m.duration_s)
    assert m.declared_at_s is None
    assert score_navigation(_straight(20), [], END, BOX, declared_at=1.5).declared_at_s == 1.5


def test_bump_is_a_held_command_without_motion() -> None:
    pinned = [(i * 0.1, 0.0, 0.0, 0.0) for i in range(20)]
    cmds = [(0.0, 0.5, 0.0, 0.0)]
    assert bumps(pinned, cmds) == 1
    assert bumps(pinned, []) == 0
    assert bumps([(i * 0.1, i * 0.05, 0.0, 0.0) for i in range(20)], cmds) == 0


def test_turn_reversals_need_hysteresis() -> None:
    yaws = [0.0, 0.1, 0.2, 0.3, 0.2, 0.1, 0.0, 0.1, 0.2, 0.3]  # left, right, left
    poses = [(i * 0.1, 0.0, 0.0, y) for i, y in enumerate(yaws)]
    reversals, total = turn_reversals(poses, hysteresis_deg=5.0)
    assert reversals == 2 and total == pytest.approx(0.9)
    steady = [(i * 0.1, 0.0, 0.0, 0.05 * i) for i in range(10)]
    assert turn_reversals(steady, hysteresis_deg=5.0)[0] == 0


def test_end_point_on_the_object_measures_to_its_edge() -> None:
    poses = _straight(76)  # ends at x = 3.8, 0.7 m from the box edge, 1.2 m from its centre
    m = score_navigation(poses, [], (5.0, 0.0), BOX)
    assert m.reached and m.final_distance_m == pytest.approx(0.7)
    assert m.straight_line_m == pytest.approx(4.5)


def test_facing_away_loses_facing_credit() -> None:
    m = score_navigation([(0.0, 4.0, 0.0, math.pi)], [], END, BOX)
    assert m.reached and not m.facing and m.facing_error_deg == pytest.approx(180.0)
    assert m.score() == pytest.approx(0.5 + 0.15)  # no facing, no path, no bumps


def test_point_in_instruction() -> None:
    assert point_in("go to the chair at (1.5, -2)") == (1.5, -2.0)
    with pytest.raises(ValueError):
        point_in("go to the chair")


def test_read_recording_streams() -> None:
    from dimos.memory.store.memory import MemoryStore
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.Twist import Twist
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.nav_msgs.Odometry import Odometry
    from dimos.msgs.std_msgs.Bool import Bool

    with MemoryStore() as store:
        with pytest.raises(LookupError):
            read_poses(store)
        store.stream("odometry", Odometry).append(
            Odometry(frame_id="world", pose=Pose(position=Vector3(1, 2, 0))), ts=10.0
        )
        store.stream("cmd_vel", Twist).append(Twist(linear=(0.3, 0.0, 0.0)), ts=10.0)
        store.stream("finished", Bool).append(Bool(False), ts=11.0)
        store.stream("finished", Bool).append(Bool(True), ts=12.0)
        assert read_poses(store) == [(10.0, 1.0, 2.0, 0.0)]
        assert read_cmds(store) == [(10.0, 0.3, 0.0, 0.0)]
        assert read_declared(store) == 12.0
