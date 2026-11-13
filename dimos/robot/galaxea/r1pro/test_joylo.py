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

from dimos.robot.galaxea.r1pro.joylo import (
    ARMS,
    JAWS,
    TORSO,
    UPPER,
    JoyloConfig,
    JoyloController,
    torso_step,
)
from dimos.teleop.joycon.mapping import JoyConReading


@pytest.fixture
def controller():
    limits = dict.fromkeys(UPPER, (-2.0, 2.0))
    limits.update(zip(TORSO, ((0, 1.7), (-2.6, 0), (-2.1, 0.5), (-3, 3)), strict=True))
    return JoyloController(limits, JoyloConfig())


def feed(
    controller, now=0.0, *, leader=None, left=JoyConReading(), right=JoyConReading(), feedback=None
):
    controller.set_leader(dict.fromkeys(ARMS, 0.0) if leader is None else leader, now)
    controller.set_feedback(
        {**dict.fromkeys(UPPER, 0), **dict.fromkeys(JAWS, 0.05)} if feedback is None else feedback,
        now,
    )
    controller.set_joycon(left, right, now)
    return controller.step(now, 0.02)


def engage(controller):
    feed(controller)
    return feed(controller, 0.02, right=JoyConReading(frozenset({"X"})))


def test_engagement_follows_calibrated_pose_without_rezeroing_and_keeps_jaws(controller):
    leader = dict.fromkeys(ARMS, 0.2)
    feed(controller, leader=leader)
    output = feed(controller, 0.02, leader=leader, right=JoyConReading(frozenset({"X"})))
    assert controller.active
    assert output.openings == (0.5, 0.5)
    assert [output.positions[n] for n in ARMS] == pytest.approx([0.005] * 14)
    for i in range(2, 80):
        output = feed(controller, i * 0.02, leader=leader)
    assert {n: output.positions[n] for n in ARMS} == pytest.approx(leader)
    assert [output.positions[n] for n in TORSO] == [0] * 4


def test_both_sticks_match_r1pro_layout_and_right_click_suppresses_yaw(controller):
    engage(controller)
    output = feed(
        controller, 0.04, left=JoyConReading(stick=(1, -1)), right=JoyConReading(stick=(-1, 0))
    )
    assert output.velocity == (0.1, -0.1, 0.25)
    output = feed(
        controller,
        0.06,
        left=JoyConReading(frozenset({"stick"}), (0, -1)),
        right=JoyConReading(frozenset({"stick"}), (-1, -1)),
    )
    assert output.velocity == (0.2, 0.0, 0.0)
    assert [output.positions[n] for n in TORSO] == [0, 0, 0, 0]


def test_disconnect_pauses_all_channels_and_held_x_cannot_rearm(controller):
    engage(controller)
    output = feed(controller, 0.04, left=None, right=JoyConReading(frozenset({"X", "ZR"}), (1, 1)))
    assert not controller.active
    assert output.velocity == (0, 0, 0)
    assert output.openings == (0.5, 0.5)
    feed(controller, 0.06, right=JoyConReading(frozenset({"X"})))
    assert not controller.active
    feed(controller, 0.08)
    output = feed(controller, 0.10, right=JoyConReading(frozenset({"X"})))
    assert controller.active and output.positions == dict.fromkeys(UPPER, 0)


def test_stale_source_latches_pause_even_when_other_inputs_continue(controller):
    engage(controller)
    controller.set_feedback({**dict.fromkeys(UPPER, 0), **dict.fromkeys(JAWS, 0.05)}, 1)
    controller.set_joycon(JoyConReading(stick=(0, -1)), JoyConReading(), 1)
    output = controller.step(1, 0.02)
    assert not controller.active and "leader" in controller.reason
    assert output.velocity == (0, 0, 0)


def test_wrist_lock_release_returns_to_absolute_leader_target(controller):
    engage(controller)
    leader = dict.fromkeys(ARMS, 0.0)
    leader[ARMS[6]] = 0.3
    output = feed(controller, 0.04, leader=leader, left=JoyConReading(frozenset({"L"})))
    assert output.positions[ARMS[6]] == 0
    output = feed(controller, 0.06, leader=leader)
    assert output.positions[ARMS[6]] == pytest.approx(0.005)
    leader[ARMS[6]] += 0.1
    output = feed(controller, 0.08, leader=leader)
    assert output.positions[ARMS[6]] == pytest.approx(0.01)
    for i in range(5, 100):
        output = feed(controller, i * 0.02, leader=leader)
    assert output.positions[ARMS[6]] == pytest.approx(0.4)


def test_trigger_toggles_once_and_release_keeps_closing(controller):
    engage(controller)
    a = feed(controller, 0.04, left=JoyConReading(frozenset({"ZL"})))
    b = feed(controller, 0.06, left=JoyConReading(frozenset({"ZL"})))
    c = feed(controller, 0.08)
    assert (a.openings[0], b.openings[0], c.openings[0]) == pytest.approx((0.44, 0.38, 0.32))
    assert c.openings[1] == 0.5


def test_home_pauses_without_creeping_toward_changing_feedback(controller):
    engage(controller)
    feed(controller, 0.04, right=JoyConReading(frozenset({"home"})))
    feedback = {**dict.fromkeys(UPPER, 0.001), **dict.fromkeys(JAWS, 0.06)}
    output = feed(controller, 0.06, feedback=feedback, right=JoyConReading(frozenset({"home"})))
    assert not controller.active
    assert output.positions == dict.fromkeys(UPPER, 0)
    assert output.openings == (0.5, 0.5)


def test_arm_lock_release_returns_to_absolute_leader_targets(controller):
    engage(controller)
    feed(controller, 0.04, left=JoyConReading(frozenset({"minus"})))
    leader = dict.fromkeys(ARMS, 0.3)
    locked = feed(controller, 0.06, leader=leader)
    assert [locked.positions[n] for n in ARMS[:7]] == [0] * 7
    assert locked.positions[ARMS[7]] > 0
    unlocked = feed(controller, 0.08, leader=leader, left=JoyConReading(frozenset({"minus"})))
    assert [unlocked.positions[n] for n in ARMS[:7]] == pytest.approx([0.005] * 7)
    leader[ARMS[0]] += 0.1
    moved = feed(controller, 0.10, leader=leader)
    assert moved.positions[ARMS[0]] == pytest.approx(0.01)
    for i in range(6, 100):
        moved = feed(controller, i * 0.02, leader=leader)
    assert {n: moved.positions[n] for n in ARMS} == pytest.approx(leader)


def test_torso_height_changes_without_pitch_or_yaw_drift(controller):
    initial = [0.4, -0.8, -0.4, 0.2]
    q = initial.copy()
    limits = [controller.limits[n] for n in TORSO]
    for _ in range(100):
        q = torso_step(q, 1, 0, 0.02, limits, controller.config)
    assert q[0] < initial[0]
    assert q[0] + q[1] - q[2] == pytest.approx(0)
    assert math.sin(q[0]) + math.sin(q[0] + q[1]) == pytest.approx(0)
    assert q[3] == 0.2
    tilted = torso_step(q, 0, 1, 0.02, limits, controller.config)
    assert tilted[0:2] == q[0:2] and tilted[2] < q[2]


def test_missing_feedback_and_leader_discontinuity_cannot_command(controller):
    assert controller.step(0, 0.02) is None
    engage(controller)
    feed(controller, 0.04, leader=dict.fromkeys(ARMS, 2.0))
    assert not controller.active and "discontinuity" in controller.reason


def test_left_only_needs_no_right_input_and_holds_every_other_robot_joint(controller):
    solo = JoyloController(controller.limits, JoyloConfig(enabled_sides=("left",), arm_only=True))
    leader = dict.fromkeys(ARMS[:7], 0.0)
    feed(solo, leader=leader, right=None)
    feed(solo, 0.02, leader=leader, left=JoyConReading(frozenset({"minus"})), right=None)
    assert solo.active
    leader[ARMS[0]] += 0.1
    output = feed(
        solo,
        0.04,
        leader=leader,
        left=JoyConReading(frozenset({"down", "ZL"}), (1, -1)),
        right=None,
    )
    assert output.positions[ARMS[0]] == pytest.approx(0.005)
    assert [output.positions[n] for n in (*TORSO, *ARMS[7:])] == [0] * 11
    assert output.openings[0] < 0.5 and output.openings[1] == 0.5
    assert output.velocity == (0, 0, 0)
    feed(solo, 0.06, leader=leader, left=JoyConReading(frozenset({"minus"})), right=None)
    assert not solo.active


def test_left_only_disconnect_or_incomplete_leader_requires_released_minus(controller):
    solo = JoyloController(controller.limits, JoyloConfig(enabled_sides=("left",), arm_only=True))
    leader = dict.fromkeys(ARMS[:7], 0.0)
    feed(solo, leader=leader, right=None)
    feed(solo, 0.02, leader=leader, left=JoyConReading(frozenset({"minus"})), right=None)
    feed(solo, 0.04, leader=leader, left=None, right=None)
    assert not solo.active
    feed(solo, 0.06, leader=leader, left=JoyConReading(frozenset({"minus"})), right=None)
    assert not solo.active
    feed(solo, 0.08, leader=leader, right=None)
    feed(solo, 0.10, leader=leader, left=JoyConReading(frozenset({"minus"})), right=None)
    assert solo.active
    feed(solo, 0.12, leader=dict.fromkeys(ARMS[:3], 0.8), right=None)
    assert not solo.active


def test_left_only_capture_pauses_even_with_minus_held(controller):
    solo = JoyloController(controller.limits, JoyloConfig(enabled_sides=("left",), arm_only=True))
    leader = dict.fromkeys(ARMS[:7], 0.0)
    feed(solo, leader=leader, right=None)
    feed(solo, 0.02, leader=leader, left=JoyConReading(frozenset({"minus"})), right=None)
    feed(solo, 0.04, leader=leader, left=JoyConReading(frozenset({"capture", "minus"})), right=None)
    assert not solo.active


def test_2025_torso_height_preserves_reach_and_pitch():
    config = JoyloConfig(torso_link_lengths=(0.4, 0.3), torso_rate=0.5)
    q = [0.0, 0.0, 0.0, 0.0]
    for _ in range(1000):
        q = torso_step(q, -1, 0, 0.01, [(-3.0, 3.0)] * 4, config)
    assert 0.4 * math.sin(q[0]) + 0.3 * math.sin(q[0] + q[1]) == pytest.approx(0, abs=2e-4)
    assert q[0] + q[1] - q[2] == pytest.approx(0, abs=1e-10)
    assert 0.4 * math.cos(q[0]) + 0.3 * math.cos(q[0] + q[1]) == pytest.approx(0.45, abs=1e-3)


@pytest.mark.parametrize("paused_feedback", [-1.59, 0.1, 0.8])
def test_reenable_converges_to_same_leader_pose_regardless_of_paused_robot_pose(
    controller, paused_feedback
):
    leader = dict.fromkeys(ARMS, 0.24)
    feedback = {**dict.fromkeys(UPPER, 0.0), **dict.fromkeys(JAWS, 0.05), ARMS[10]: paused_feedback}
    feed(controller, leader=leader, feedback=feedback)
    feed(controller, 0.02, leader=leader, feedback=feedback, right=JoyConReading(frozenset({"X"})))
    feed(
        controller, 0.04, leader=leader, feedback=feedback, right=JoyConReading(frozenset({"home"}))
    )
    feed(controller, 0.06, leader=leader, feedback=feedback)
    output = feed(
        controller, 0.08, leader=leader, feedback=feedback, right=JoyConReading(frozenset({"X"}))
    )
    first = output.positions[ARMS[10]]
    assert abs(first - paused_feedback) <= 0.25 * 0.02 + 1e-12
    for i in range(5, 500):
        output = feed(controller, i * 0.02, leader=leader, feedback=feedback)
    assert controller.active
    assert {n: output.positions[n] for n in ARMS} == pytest.approx(leader)


def test_repeated_pause_enable_cycles_never_accumulate_an_offset(controller):
    leader = dict.fromkeys(ARMS, 0.2)
    for cycle in range(4):
        t = cycle * 2
        feed(controller, t, leader=leader)
        feed(controller, t + 0.02, leader=leader, right=JoyConReading(frozenset({"X"})))
        for i in range(2, 80):
            output = feed(controller, t + i * 0.02, leader=leader)
        assert {n: output.positions[n] for n in ARMS} == pytest.approx(leader)
        feed(controller, t + 1.6, leader=leader, right=JoyConReading(frozenset({"home"})))
        assert not controller.active
