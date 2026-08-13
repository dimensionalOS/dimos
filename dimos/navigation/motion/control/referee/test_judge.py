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

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped as RefereePose
from dimos.msgs.nav_msgs.Path import Path as RefereePath
from dimos.navigation.motion.control.referee import world
from dimos.navigation.motion.control.referee.episode import EpisodeConfig, EpisodeResult
from dimos.navigation.motion.control.referee.judge import (
    cross_track,
    executed_clearance,
    score_episode,
    summarize,
)
from dimos.navigation.motion.scenarios import Box, Scenario

CLEAR = Scenario("t_clear", [], goal=(4.0, 0.0))
REFUSE = Scenario("t_refuse", [], goal=(4.0, 0.0), expect="refuse")
# a wall along y=0.5: the path at y=0 has ~0.19 m of body clearance
WALLED = Scenario("t_walled", [Box(2.0, 0.75, 4.0, 0.3)], goal=(4.0, 0.0))


def _plan(n: int = 41) -> RefereePath:
    return RefereePath(
        frame_id="world",
        poses=[RefereePose(frame_id="world", position=[i * 0.1, 0.0, 0.0]) for i in range(n)],
    )


def _result(
    sc: Scenario = CLEAR,
    outcome: str = "goal",
    time_to_goal: float | None = 13.0,
    lateral: float = 0.0,
    tilt: float = 0.05,
    n: int = 200,
) -> EpisodeResult:
    t = np.linspace(0.5, 13.0, n)
    pos = np.column_stack([np.linspace(0, 4, n), np.full(n, lateral), np.full(n, 0.3)])
    cmd = np.tile([0.4, 0.0, 0.0], (n, 1))
    return EpisodeResult(
        scenario=sc,
        outcome=outcome,
        t=t,
        pos=pos,
        yaw=np.zeros(n),
        tilt=np.full(n, tilt),
        twist_cmd=cmd,
        used_cmd=cmd.copy(),
        contact=np.zeros(n, dtype=bool),
        plan=world.to_nav_path(_plan()),
        plans=[_plan()],
        plan_t=[0.5],
        plan_min_clear=[0.29],
        plan_ms=[2.0],
        time_to_goal=time_to_goal,
        cfg=EpisodeConfig(),
    )


def test_cross_track_on_line_is_zero() -> None:
    path = np.array([[0.0, 0.0], [4.0, 0.0]])
    pos = np.array([[1.0, 0.0], [2.5, 0.0]])
    np.testing.assert_allclose(cross_track(pos, path), 0.0, atol=1e-12)


def test_cross_track_offset_and_beyond_ends() -> None:
    path = np.array([[0.0, 0.0], [4.0, 0.0]])
    pos = np.array([[2.0, 0.3], [-1.0, 0.0], [5.0, 0.4]])
    np.testing.assert_allclose(cross_track(pos, path), [0.3, 1.0, np.hypot(1.0, 0.4)], atol=1e-12)


def test_executed_clearance_open_world_is_inf() -> None:
    assert np.all(np.isinf(executed_clearance(_result())))


def test_executed_clearance_beside_wall() -> None:
    r = _result(sc=WALLED)
    clear = executed_clearance(r)
    # wall inner face at y=0.6, body edge at y=0.2965 (the GO2 union
    # re-baselined from 0.50 to 0.593 wide by planner/revision.md): ~0.30 of
    # true body clearance riding the corridor centre. The footprint sampler
    # steps past the box edge by up to half a step, so it reads a shade under.
    assert 0.25 < float(np.min(clear)) < 0.35


def test_clean_run_scores_high() -> None:
    row = score_episode(_result())
    assert row["total"] > 110.0
    assert not row["dq"]
    assert row["arrived"] == 1.0 and row["precision"] == 1.0


def test_collision_gates_to_zero() -> None:
    row = score_episode(_result(outcome="collision", time_to_goal=None))
    assert row["dq"] and row["total"] == 0.0


def test_fall_gates_to_zero() -> None:
    assert score_episode(_result(outcome="fall", time_to_goal=None))["total"] == 0.0


def test_below_floor_costs_precision_not_arrival() -> None:
    # ride at y=0.42: body edge 0.155 from centre -> ~0.03 clearance to the
    # wall face at 0.6 - wait, face at 0.6; edge at 0.575 -> 0.025 < floor
    row = score_episode(_result(sc=WALLED, lateral=0.42))
    assert row["arrived"] == 1.0
    assert row["below_floor"] > 0.9
    assert row["precision"] < 0.1


def test_open_space_deviation_is_free() -> None:
    # same deviation, no walls: precision untouched (context-specific for free)
    row = score_episode(_result(lateral=0.42))
    assert row["precision"] == 1.0
    assert row["xtrack_p95"] > 0.4  # still visible as a diagnostic


def test_slow_run_costs_pace() -> None:
    fast = score_episode(_result(time_to_goal=13.0))
    slow = score_episode(_result(time_to_goal=40.0))
    assert slow["pace"] < fast["pace"]
    assert slow["arrived"] == fast["arrived"] == 1.0


def test_timeout_on_clear_world_scores_low() -> None:
    row = score_episode(_result(outcome="timeout", time_to_goal=None))
    assert row["arrived"] == 0.0
    assert row["total"] < 12.0


def test_refusal_on_sealed_world_is_success() -> None:
    row = score_episode(_result(sc=REFUSE, outcome="refused", time_to_goal=None, n=5))
    assert row["arrived"] == 1.0 and row["precision"] == 1.0
    assert row["total"] > 110.0


def test_tilt_tail_costs_composure() -> None:
    calm = score_episode(_result(tilt=0.05))
    shaky = score_episode(_result(tilt=0.30))
    assert shaky["composure"] < calm["composure"]
    assert shaky["total"] < calm["total"]


def test_not_reaching_a_clear_world_is_a_categorical_failure() -> None:
    # A mean over enough worlds absorbs a stuck robot; the failed flag may not.
    stuck = score_episode(_result(outcome="timeout", time_to_goal=None))
    assert stuck["failed"]
    refused_ok = score_episode(_result(sc=REFUSE, outcome="refused", time_to_goal=None, n=5))
    assert not refused_ok["failed"]
    s = summarize([score_episode(_result()), stuck])
    assert s["failed"] == [stuck["name"]]


def test_summarize_counts() -> None:
    rows = [
        score_episode(_result()),
        score_episode(_result(outcome="collision", time_to_goal=None)),
    ]
    s = summarize(rows)
    assert s["worlds"] == 2 and s["dq"] == 1
    assert s["outcomes"] == {"goal": 1, "collision": 1}
    assert s["worst"]["total"] == 0.0
    assert s["score"] == pytest.approx((rows[0]["total"] + 0.0) / 2, abs=0.01)


def test_active_cross_track_scores_against_the_plan_of_the_time() -> None:
    from dimos.navigation.motion.control.referee.judge import active_cross_track

    r = _result(lateral=0.3)
    # a replan at t=7 that starts at the (drifted) robot: y=0.3
    late = RefereePath(
        frame_id="world",
        poses=[RefereePose(frame_id="world", position=[i * 0.1, 0.3, 0.0]) for i in range(41)],
    )
    r.plans = [_plan(), late]
    r.plan_t = [0.5, 7.0]
    xt = active_cross_track(r)
    early, after = xt[r.t < 7.0], xt[r.t >= 7.0]
    assert float(np.min(early)) > 0.25  # drift vs the original plan stays scored
    assert float(np.max(after)) < 0.05  # the new plan is followed cleanly


def test_plan_churn_measures_forced_replans() -> None:
    from dimos.navigation.motion.control.referee.judge import plan_churn

    r = _result()
    straight = _plan()
    detour = RefereePath(
        frame_id="world",
        poses=[RefereePose(frame_id="world", position=[i * 0.1, 0.2, 0.0]) for i in range(41)],
    )
    r.plans, r.plan_t = [straight, straight], [0.5, 7.0]
    assert plan_churn(r) < 1e-9  # identical replans: follower held the line
    r.plans = [straight, detour]
    assert 0.15 < plan_churn(r) < 0.25  # follower forced a 0.2 m re-route
