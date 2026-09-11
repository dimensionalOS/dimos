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

from copy import copy

import mujoco
import numpy as np
import pytest
from dimos.robot.pollen.microduck.places import add_ball_body
from microduck_world.ball_physics import BALL_RADIUS
from microduck_world.football import BALL_NAMES, FootballMatch, Scoreboard, add_footballs
from microduck_world.scene import load_world


@pytest.fixture
def pitch():
    spec = mujoco.MjSpec.from_file(str(load_world()[0].mujoco_scene_path))
    spec.option.timestep = 0.005
    add_ball_body(spec)
    add_footballs(spec)
    model = spec.compile()
    return model, mujoco.MjData(model), FootballMatch(model)


def put(pitch, x, y=4.3, z=BALL_RADIUS, ball="football_ball_1"):
    model, data, match = pitch
    adr = int(model.joint(ball + "_freejoint").qposadr[0])
    data.qpos[adr : adr + 3] = (x, y, z)
    match.update(data)


@pytest.mark.parametrize("sign, expected", [(1, [1, 0]), (-1, [0, 1])])
def test_whole_ball_must_cross_and_only_once_until_it_returns_to_field(pitch, sign, expected):
    _, data, match = pitch
    put(pitch, sign * 1.2)
    put(pitch, sign * 1.32)
    assert match.scores == [0, 0]  # Center over line, back of ball not yet over.
    put(pitch, sign * (1.306 + BALL_RADIUS - 0.001))
    assert match.scores == [0, 0]  # Would have scored with the old 3.5 cm radius.
    put(pitch, sign * (1.306 + BALL_RADIUS + 0.001))
    assert match.scores == expected
    before = data.qpos.copy()
    for _ in range(20):
        match.update(data)
    np.testing.assert_array_equal(data.qpos, before)
    put(pitch, sign * 1.335)
    put(pitch, sign * 1.36)
    assert match.scores == expected
    put(pitch, sign * 1.2)
    put(pitch, sign * 1.37)
    assert match.scores == [n * 2 for n in expected]


@pytest.mark.parametrize("sign", [-1, 1])
def test_ball_starting_in_net_or_crossing_backwards_does_not_score(pitch, sign):
    put(pitch, sign * 1.6)
    put(pitch, sign * 1.4)
    put(pitch, sign * 1.1)
    assert pitch[2].scores == [0, 0]


@pytest.mark.parametrize(
    "y, z", [(4.9, BALL_RADIUS), (3.7, BALL_RADIUS), (4.3, 0.56), (4.79, BALL_RADIUS), (4.3, 0.47)]
)
def test_wide_shots_and_balls_overlapping_posts_or_crossbar_do_not_score(pitch, y, z):
    put(pitch, 1.1, y, z)
    put(pitch, 1.6, y, z)
    assert pitch[2].scores == [0, 0]


def test_entering_from_side_behind_goal_line_does_not_score(pitch):
    put(pitch, 1.1, 5.1)
    put(pitch, 1.4, 5.1)
    put(pitch, 1.4, 4.3)
    assert pitch[2].scores == [0, 0]


def test_a_ball_landing_in_the_net_from_above_does_not_score(pitch):
    put(pitch, 1.1, z=0.7)
    put(pitch, 1.32, z=0.6)
    put(pitch, 1.32, z=0.1)
    put(pitch, 1.4, z=0.1)
    assert pitch[2].scores == [0, 0]


def test_simultaneous_balls_score_independently_and_credit_opposite_goal(pitch):
    model, data, match = pitch
    for ball in BALL_NAMES:
        adr = int(model.joint(ball + "_freejoint").qposadr[0])
        data.qpos[adr : adr + 3] = [1.1, 4.3, BALL_RADIUS]
    match.update(data)
    for ball in BALL_NAMES:
        adr = int(model.joint(ball + "_freejoint").qposadr[0])
        data.qpos[adr] = 1.5
    match.update(data)
    assert match.scores == [4, 0]
    assert match.last_goal["team"] == "blue"


@pytest.mark.parametrize("sign, expected", [(1, [1, 0]), (-1, [0, 1])])
def test_real_mujoco_ball_rolls_into_net_scores_and_is_retained(pitch, sign, expected):
    model, data, match = pitch
    ball = model.joint("football_ball_1_freejoint")
    adr, vel = int(ball.qposadr[0]), int(ball.dofadr[0])
    put(pitch, sign * 0.85)
    data.qvel[vel] = sign * 1.0
    data.qvel[vel + 4] = sign / BALL_RADIUS
    for _ in range(400):
        mujoco.mj_step(model, data)
        match.update(data)
    assert match.scores == expected
    assert 1.34 < sign * data.qpos[adr] < 1.74
    assert abs(data.qvel[vel]) < 0.2
    np.testing.assert_allclose(model.body("football_ball_1").mass, [0.03])
    np.testing.assert_allclose(model.body("football_ball_1").inertia, [0.00003] * 3)


def test_real_post_contact_deflects_ball_without_a_phantom_goal(pitch):
    model, data, match = pitch
    ball = model.joint("football_ball_1_freejoint")
    adr, vel = int(ball.qposadr[0]), int(ball.dofadr[0])
    put(pitch, 0.85, y=4.825)
    data.qvel[vel] = 1
    data.qvel[vel + 4] = 1 / BALL_RADIUS
    touched = False
    post = model.geom("football_coral_post_1").id
    for _ in range(300):
        mujoco.mj_step(model, data)
        match.update(data)
        touched |= bool(np.any(data.contact.geom[: data.ncon] == post))
    assert touched
    assert match.scores == [0, 0]
    assert data.qpos[adr] < 1.3


def test_player_tunnel_has_no_collision_and_side_is_still_a_wall(pitch):
    model, data, _ = pitch
    mujoco.mj_forward(model, data)
    group = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
    geom_id = np.zeros(1, dtype=np.int32)
    through = mujoco.mj_ray(
        model, data, np.array([0.0, 1.9, 0.2]), np.array([0.0, 1.0, 0.0]), group, 1, -1, geom_id
    )
    assert through > 0.7
    blocked = mujoco.mj_ray(
        model, data, np.array([1.8, 1.9, 0.2]), np.array([0.0, 1.0, 0.0]), group, 1, -1, geom_id
    )
    assert blocked == pytest.approx(0.1)


def test_native_camera_copy_and_browser_receive_the_same_lamp_ids(pitch):
    model, _, match = pitch
    native = copy(model)
    board = Scoreboard(native)
    put(pitch, 1.1)
    put(pitch, 1.5)
    snapshot = match.snapshot()
    board.apply(native, snapshot["lit"])
    assert snapshot["scores"] == [1, 0]
    on = model.geom("score_blue_2_b").id
    off = model.geom("score_blue_2_a").id
    assert on in snapshot["lit"] and off not in snapshot["lit"]
    np.testing.assert_array_equal(native.geom_rgba[on], model.geom_rgba[on])
    np.testing.assert_allclose(native.geom_rgba[off, :3], [0.018, 0.025, 0.024])
    np.testing.assert_allclose(model.geom_rgba[off, :3], [0.20, 0.53, 0.75])


def test_large_scores_keep_full_count_and_light_overflow_indicator(pitch):
    model, _, match = pitch
    match.scores[:] = [999, 0]
    put(pitch, 1.1)
    put(pitch, 1.5)
    assert match.snapshot()["scores"] == [1000, 0]
    assert model.geom("score_blue_overflow_0").id in match.lit


def test_fast_diagonal_shot_checks_sphere_edge_between_samples(pitch):
    # The centre fits at x=0, but the swept edge clips the right post.
    put(pitch, 1.1, y=4.47)
    put(pitch, 1.5, y=5.03)
    assert pitch[2].scores == [0, 0]


def test_glancing_entry_that_fits_the_swept_opening_is_a_goal(pitch):
    # Its leading tip enters near a post, then the ball travels inward.
    # Requiring the full radius at the leading tip would reject this valid path.
    put(pitch, 1.306 - BALL_RADIUS - 0.005, y=4.78)
    put(pitch, 1.306 + BALL_RADIUS + 0.005, y=4.67)
    assert pitch[2].scores == [1, 0]


@pytest.mark.parametrize(
    "team, generation, expected",
    [("duck4", "session", 1), ("duck1", "session", 0), ("duck4", "old-session", 0)],
)
def test_scorer_credit_uses_last_touch_owner_excludes_own_goals_and_stale_identity(
    pitch, tmp_path, team, generation, expected
):
    from types import SimpleNamespace

    from microduck_world.scorers import ScorerLedger

    model, _, match = pitch
    ledger = ScorerLedger(tmp_path / "scores.sqlite3")
    match.ledger = ledger
    robot = SimpleNamespace(id=team, active=True, generation="session", geoms=[99999])
    contact = SimpleNamespace(geom=[model.geom("football_ball_1_geom").id, 99999], dist=0)
    sensed = SimpleNamespace(model=model, contact=[contact], ncon=1)
    match.touches(
        sensed,
        {team: robot},
        {team: {"generation": generation, "userId": "123", "handle": "player"}},
    )
    put(pitch, 1.0)
    put(pitch, 1.5)
    assert match.scores == [1, 0]
    ledger.flush()
    assert ledger.rows == ([{"handle": "player", "goals": 1}] if expected else [])
    assert ScorerLedger(tmp_path / "scores.sqlite3").rows == ledger.rows
    match.scores[:] = [0, 0]
    assert ScorerLedger(tmp_path / "scores.sqlite3").rows == ledger.rows


@pytest.mark.parametrize("ball", BALL_NAMES)
def test_manual_drop_resets_one_ball_at_two_metres_without_score_or_old_credit(pitch, ball):
    model, data, match = pitch
    other_positions = data.qpos.copy()
    adr = match.balls[ball][0]
    match._last_touch[ball] = {"handle": "old"}
    assert match.drop_ball(data, ball)
    np.testing.assert_allclose(data.qpos[adr : adr + 7], [0, 4.3, 2, 1, 0, 0, 0])
    assert ball not in match._last_touch
    for other, (other_adr, _) in match.balls.items():
        if other != ball:
            np.testing.assert_array_equal(
                data.qpos[other_adr : other_adr + 7], other_positions[other_adr : other_adr + 7]
            )
    match.update(data)
    assert match.scores == [0, 0]
    assert match.drops[ball] == 1
    for _ in range(60):
        mujoco.mj_step(model, data)
    assert data.qpos[adr + 2] < 1.8


def test_manual_drops_wait_for_clear_space(pitch):
    _, data, match = pitch
    assert match.drop_ball(data, BALL_NAMES[1])
    assert not match.drop_ball(data, BALL_NAMES[2])
    assert match.drops[BALL_NAMES[2]] == 0
