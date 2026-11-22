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

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.galaxea.r1pro.joylo import (
    _WRIST_TO_GRIPPER,
    ARMS,
    JAWS,
    ROBOT_EYES,
    TORSO,
    UPPER,
    HandsOutput,
    JoyloConfig,
    JoyloController,
    _turned,
    torso_step,
)
from dimos.teleop.joycon.mapping import JoyConReading
from dimos.teleop.utils.teleop_transforms import webxr_to_robot


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
    assert [output.positions[n] for n in ARMS] == pytest.approx([0.2] * 14)
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
    assert output.velocity == (0.1, 0.0, 0.0)
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


@pytest.mark.parametrize("source", ["leader", "joycon", "robot"])
def test_brief_gap_holds_motion_then_resumes_only_on_fresh_input(controller, source):
    engage(controller)
    feed(controller, 0.04, leader=dict.fromkeys(ARMS, 0.4))
    controller.receipts.update(dict.fromkeys(controller.receipts, 0.34))
    controller.receipts[source] = 0.04

    held = controller.step(0.34, 0.01)

    assert not controller.active
    assert controller.recovery_deadline == pytest.approx(1.04)
    if source == "robot":
        assert held is None
    else:
        assert held.positions == dict.fromkeys(UPPER, 0)
        assert held.velocity == (0, 0, 0)
    resumed = feed(controller, 0.36, leader=dict.fromkeys(ARMS, 0.2))
    assert controller.active
    assert controller.recovery_deadline is None
    assert {name: resumed.positions[name] for name in ARMS} == dict.fromkeys(ARMS, 0.2)
    assert controller.hold_count == 1
    assert controller.pause_count == 0


def test_input_returning_before_watchdog_tick_still_checks_gap_and_jump(controller):
    engage(controller)
    feed(controller, 0.35, leader=dict.fromkeys(ARMS, 0.2))
    assert controller.active
    assert controller.hold_count == 1
    feed(controller, 0.7, leader=dict.fromkeys(ARMS, 1.5))
    assert not controller.active
    assert controller.recovery_deadline is None
    assert "discontinuity" in controller.reason


def test_long_gap_requires_reenable_even_if_next_tick_has_fresh_inputs(controller):
    engage(controller)
    controller.step(0.35, 0.01)
    feed(controller, 1.1)
    assert not controller.active
    assert controller.recovery_deadline is None
    feed(controller, 1.12, right=JoyConReading(frozenset({"X"})))
    assert controller.active


@pytest.mark.parametrize("reason", ["operator", "disconnect", "home"])
def test_explicit_pause_or_controller_loss_cancels_pending_recovery(controller, reason):
    engage(controller)
    controller.step(0.35, 0.01)
    if reason == "operator":
        controller.pause("Paused by operator")
    elif reason == "disconnect":
        feed(controller, 0.36, left=None)
    else:
        # Home must work even while the leader and robot streams are stale.
        controller.set_joycon(JoyConReading(), JoyConReading(frozenset({"home"})), 0.36)
        controller.step(0.36, 0.01)
    feed(controller, 0.38)
    assert not controller.active
    assert controller.recovery_deadline is None


def test_recovery_does_not_replay_gripper_or_arm_lock_button_edges(controller):
    engage(controller)
    controller.step(0.35, 0.01)
    held_buttons = JoyConReading(frozenset({"ZL", "minus"}))
    resumed = feed(controller, 0.36, left=held_buttons)
    assert controller.active
    assert resumed.openings == (0.5, 0.5)
    assert controller.arm_locked == [False, False]


def test_wrist_lock_release_returns_to_absolute_leader_target(controller):
    engage(controller)
    leader = dict.fromkeys(ARMS, 0.0)
    leader[ARMS[6]] = 0.3
    output = feed(controller, 0.04, leader=leader, left=JoyConReading(frozenset({"L"})))
    assert output.positions[ARMS[6]] == 0
    output = feed(controller, 0.06, leader=leader)
    assert output.positions[ARMS[6]] == pytest.approx(0.3)
    leader[ARMS[6]] += 0.1
    output = feed(controller, 0.08, leader=leader)
    assert output.positions[ARMS[6]] == pytest.approx(0.4)
    for i in range(5, 100):
        output = feed(controller, i * 0.02, leader=leader)
    assert output.positions[ARMS[6]] == pytest.approx(0.4)


def test_trigger_toggles_once_and_release_keeps_closed_target(controller):
    engage(controller)
    a = feed(controller, 0.04, left=JoyConReading(frozenset({"ZL"})))
    b = feed(controller, 0.06, left=JoyConReading(frozenset({"ZL"})))
    c = feed(controller, 0.08)
    assert (a.openings[0], b.openings[0], c.openings[0]) == pytest.approx((0, 0, 0))
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
    assert [unlocked.positions[n] for n in ARMS[:7]] == pytest.approx([0.3] * 7)
    leader[ARMS[0]] += 0.1
    moved = feed(controller, 0.10, leader=leader)
    assert moved.positions[ARMS[0]] == pytest.approx(0.4)
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
    assert output.positions[ARMS[0]] == pytest.approx(0.1)
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
    config = JoyloConfig(torso_link_lengths=(0.4, 0.3))
    q = [0.0, 0.0, 0.0, 0.0]
    for _ in range(1000):
        q = torso_step(q, -1, 0, 0.01, [(-3.0, 3.0)] * 4, config)
    assert 0.4 * math.sin(q[0]) + 0.3 * math.sin(q[0] + q[1]) == pytest.approx(0, abs=2e-4)
    assert q[0] + q[1] - q[2] == pytest.approx(0, abs=1e-10)
    assert 0.4 * math.cos(q[0]) + 0.3 * math.cos(q[0] + q[1]) == pytest.approx(0.4, abs=1e-3)


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
    assert first == leader[ARMS[10]]  # The coordinator owns physical slew.
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


@pytest.mark.parametrize("paused", [False, True])
@pytest.mark.parametrize("direction, neutral", [(1, -0.050621366), (-1, 0.214757310)])
@pytest.mark.parametrize("joint", [ARMS[2], ARMS[4]])
def test_returning_leader_to_neutral_after_a_turn_does_not_pin_robot_at_limit(
    controller, paused, direction, neutral, joint
):
    controller.config.leader_angle_period = math.tau
    controller.limits[joint] = (-2.321289415, 2.321289415)
    engage(controller)
    leader = dict.fromkeys(ARMS, 0.0)
    for i in range(1, 41):
        leader[joint] = math.remainder((direction * math.tau + neutral) * i / 40, math.tau)
        buttons = frozenset({"home"}) if paused and i == 10 else frozenset()
        output = feed(controller, 0.02 + i * 0.02, leader=leader, right=JoyConReading(buttons))
    if paused:
        output = feed(controller, 0.84, leader=leader, right=JoyConReading(frozenset({"X"})))

    assert controller.active
    assert controller.leader[joint] == pytest.approx(neutral)
    assert output.positions[joint] == pytest.approx(neutral)


def test_periodic_leader_still_rejects_a_real_discontinuity(controller):
    controller.config.leader_angle_period = math.tau
    engage(controller)
    feed(controller, 0.04, leader={**dict.fromkeys(ARMS, 0.0), ARMS[2]: 2.0})

    assert not controller.active
    assert "Leader discontinuity" in controller.reason


# The hands of a headset as the arm source: the Joy-Cons keep their functions.
LEFT_HAND = ((0.3, 0.2, 1.0), (0.0, 0.0, 0.0, 1.0))
RIGHT_HAND = ((0.3, -0.2, 1.0), (0.0, 0.0, 0.0, 1.0))
TURNED = (0.0, 0.0, math.sin(0.3), math.cos(0.3))
# The operator's eyes: 0.1 m behind the hands and 0.4 m above them, facing forward.
HEAD = ((-0.1, 0.0, 1.4), 0.0)
EYES = ROBOT_EYES


def feed_hands(
    controller,
    now=0.0,
    *,
    left_hand=LEFT_HAND,
    right_hand=RIGHT_HAND,
    head=HEAD,
    left=JoyConReading(),
    right=JoyConReading(),
):
    for side, hand in (("left", left_hand), ("right", right_hand)):
        if hand is not None:
            controller.set_hand(side, hand, now)
    if head is not None:
        controller.set_head(*head, now)
    controller.set_feedback({**dict.fromkeys(UPPER, 0), **dict.fromkeys(JAWS, 0.05)}, now)
    controller.set_joycon(left, right, now)
    return controller.step(now, 0.02)


@pytest.fixture
def hands(controller):
    controller.set_arm_source("hands")
    feed_hands(controller)
    output = feed_hands(controller, 0.02, right=JoyConReading(frozenset({"X"})))
    assert controller.active and output.hands.engaged
    return controller


def test_hands_need_no_leader_and_start_only_on_the_enable_press(controller):
    controller.set_arm_source("hands")
    output = feed_hands(controller)
    assert not controller.active and output.hands == HandsOutput(False, None)
    output = feed_hands(controller, 0.02, right=JoyConReading(frozenset({"X"})))
    assert controller.active and controller.reason == "Following the headset's hands"
    assert output.hands.engaged and output.hands.poses is not None
    # The leader arms' joint targets stay where the robot stood.
    assert [output.positions[n] for n in ARMS] == [0] * 14


def test_a_gripper_goes_where_its_hand_is_from_the_eyes(hands):
    # Each hand is 0.4 m in front of the operator's eyes, 0.2 m to its side and 0.4 m below.
    left, right = feed_hands(hands, 0.04).hands.poses
    assert left[0] == pytest.approx((EYES[0] + 0.4, 0.2, EYES[2] - 0.4))
    assert right[0] == pytest.approx((EYES[0] + 0.4, -0.2, EYES[2] - 0.4))
    # The hand moves 0.1 m up and 0.05 m to the right: so does the gripper, from the same place.
    moved = feed_hands(hands, 0.06, right_hand=((0.3, -0.25, 1.1), RIGHT_HAND[1])).hands.poses[1]
    assert moved[0] == pytest.approx((EYES[0] + 0.4, -0.25, EYES[2] - 0.3))
    # The operator's head moves after the enable press: the hands are still measured from where it was.
    after = feed_hands(hands, 0.08, head=((0.2, 0.3, 1.5), 0.4)).hands.poses
    assert after[0][0] == pytest.approx(left[0]) and after[0][1] == pytest.approx(left[1])


def test_hands_are_measured_in_the_directions_the_operator_faced_at_the_enable_press(controller):
    # The operator faces left in the headset's space: forward is +y there, and right is +x.
    controller.set_arm_source("hands")
    head = ((0.0, -0.1, 1.4), math.pi / 2)
    quarter = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    left_hand, right_hand = ((-0.2, 0.3, 1.0), quarter), ((0.2, 0.3, 1.0), quarter)
    feed_hands(controller, left_hand=left_hand, right_hand=right_hand, head=head)
    output = feed_hands(
        controller,
        0.02,
        left_hand=left_hand,
        right_hand=right_hand,
        head=head,
        right=JoyConReading(frozenset({"X"})),
    )
    left, right = output.hands.poses
    assert left[0] == pytest.approx((EYES[0] + 0.4, 0.2, EYES[2] - 0.4))
    assert right[0] == pytest.approx((EYES[0] + 0.4, -0.2, EYES[2] - 0.4))
    # A hand that is turned as the operator is, is a hand that is not turned.
    assert left[1] == pytest.approx(_turned(LEFT_HAND[1], _WRIST_TO_GRIPPER))


def test_the_robots_eyes_are_where_the_model_has_the_head_cameras():
    # With every joint at zero the frames along the torso are not turned: the places add up.
    joints = {joint.name: joint.origin_xyz for joint in JoyloConfig().model.load().joints}
    cameras = [joints[f"camera_head_{side}_joint"] for side in ("left", "right")]
    eyes = [
        sum(joints[name][i] for name in (*TORSO, "head_joint"))
        + (cameras[0][i] + cameras[1][i]) / 2
        for i in range(3)
    ]
    assert eyes == pytest.approx(ROBOT_EYES, abs=2e-3)


def test_a_gripper_points_where_the_fingers_point_and_closes_across_the_palm(hands):
    # The wrist joint of a headset: -z out along the fingers, +y out of the back of the hand.
    # Each hand holds a Joy-Con: the fingers point forward and the back of the hand outward.
    def wrist(back_of_hand):  # in the headset's axes: x right, y up, z back
        z = np.array([0.0, 0.0, 1.0])
        y = np.array(back_of_hand)
        return Rotation.from_matrix(np.column_stack([np.cross(y, z), y, z]))

    for side, back in (("left", (-1.0, 0.0, 0.0)), ("right", (1.0, 0.0, 0.0))):
        q = wrist(back).as_quat()
        hand = webxr_to_robot(
            PoseStamped(position=[0.0, 1.0, -0.4], orientation=list(q)),
            is_left_controller=side == "left",
        )
        o = hand.orientation
        target = hands.gripper_target(((0.3, 0.0, 1.0), (o.x, o.y, o.z, o.w)))
        link = Rotation.from_quat(target[1]).as_matrix()
        # The gripper link: -z is its approach, y the line its jaws close on.
        assert -link[:, 2] == pytest.approx([1, 0, 0], abs=1e-6), side
        # Both arms reach forward with the link's y to the robot's left (their natural roll).
        assert link[:, 1] == pytest.approx([0, 1, 0], abs=1e-6), side
    # A flat right hand, palm down: the jaws close up and down.
    flat = webxr_to_robot(
        PoseStamped(position=[0.0, 1.0, -0.4], orientation=[0, 0, 0, 1]), is_left_controller=False
    ).orientation
    link = Rotation.from_quat(
        hands.gripper_target(((0.3, 0.0, 1.0), (flat.x, flat.y, flat.z, flat.w)))[1]
    ).as_matrix()
    assert -link[:, 2] == pytest.approx([1, 0, 0], abs=1e-6)
    assert abs(link[2, 1]) == pytest.approx(1)


def test_joycons_drive_lift_grip_and_pause_as_with_the_leader_arms(hands):
    output = feed_hands(
        hands,
        0.04,
        left=JoyConReading(frozenset({"up", "ZL"}), stick=(0, -1)),
        right=JoyConReading(stick=(-1, 0)),
    )
    assert output.velocity == (0.1, 0.0, 0.25)
    assert output.openings == (0.0, 0.5)
    output = feed_hands(hands, 0.06, right=JoyConReading(frozenset({"home"})))
    assert not hands.active and output.hands == HandsOutput(False, None)
    assert output.velocity == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("source", ["leader", "hands"])
def test_x_toggles_tracking_off_and_requires_a_new_press_to_reenable(controller, source):
    controller.set_arm_source(source)
    send = feed_hands if source == "hands" else feed
    x = JoyConReading(frozenset({"X"}))
    send(controller, 0.0)
    send(controller, 0.02, right=x)
    assert controller.active
    send(controller, 0.04, right=x)
    assert controller.active  # A held button is one press.
    send(controller, 0.06)

    output = send(controller, 0.08, right=x)

    assert not controller.active
    assert controller.recovery_deadline is None
    assert output.velocity == (0.0, 0.0, 0.0)
    assert output.hands == (HandsOutput(False, None) if source == "hands" else None)
    send(controller, 0.10, right=x)
    assert not controller.active
    send(controller, 0.12)
    assert not controller.active
    send(controller, 0.14, right=x)
    assert controller.active


def test_x_cancels_hand_tracking_recovery_without_waiting_for_poses(hands):
    feed_hands(hands, 0.04)  # Release the initial enable press.
    feed_hands(hands, 0.40, left_hand=None)
    assert hands.recovery_deadline is not None

    output = feed_hands(hands, 0.42, left_hand=None, right=JoyConReading(frozenset({"X"})))

    assert not hands.active
    assert hands.recovery_deadline is None
    assert output.hands == HandsOutput(False, None)
    feed_hands(hands, 0.44)
    assert not hands.active  # Fresh poses cannot undo the operator's stop.


def test_a_lost_hand_or_head_stops_following_and_a_brief_loss_recovers_in_place(hands):
    before = feed_hands(hands, 0.04).hands.poses
    output = feed_hands(hands, 0.4, left_hand=None)
    assert not hands.active and not output.hands.engaged
    assert hands.reason == "Holding for fresh hands"
    # The operator moved in the gap: the hands are still measured from the enable press.
    output = feed_hands(hands, 0.5, head=((0.3, 0.0, 1.4), 0.2))
    assert hands.active and output.hands.poses == before
    # A long loss needs the enable button again.
    feed_hands(hands, 2.0, left_hand=None, right_hand=None, head=None)
    output = feed_hands(hands, 2.02)
    assert not hands.active and not output.hands.engaged
    assert "hands, head" in hands.last_pause_reason or "recovery" in hands.last_pause_reason


def test_without_the_operators_head_the_hands_cannot_be_enabled(controller):
    controller.set_arm_source("hands")
    feed_hands(controller, head=None)
    output = feed_hands(controller, 0.02, head=None, right=JoyConReading(frozenset({"X"})))
    assert not controller.active and not output.hands.engaged
    assert controller.reason == "Waiting for fresh head"
    controller.set_head((0.0, 0.0, math.inf), 0.0, 0.04)
    assert controller.reason == "Malformed head pose"


def test_a_locked_arm_keeps_its_pose_and_a_held_wrist_its_orientation(hands):
    target = hands.gripper_target
    moved = ((0.4, 0.2, 1.1), TURNED)
    # Minus locks the left arm at the pose its hand has then.
    output = feed_hands(hands, 0.04, left=JoyConReading(frozenset({"minus"})))
    assert output.hands.poses == (target(LEFT_HAND), target(RIGHT_HAND))
    output = feed_hands(hands, 0.06, left_hand=moved, right_hand=((0.3, -0.2, 1.2), TURNED))
    assert output.hands.poses == (target(LEFT_HAND), target(((0.3, -0.2, 1.2), TURNED)))
    # Minus again: the arm goes to where the hand is now.
    feed_hands(hands, 0.08, left_hand=moved, left=JoyConReading(frozenset({"minus"})))
    output = feed_hands(hands, 0.10, left_hand=moved)
    assert output.hands.poses[0] == target(moved)
    # R holds the right wrist: the position follows, the orientation stays.
    output = feed_hands(hands, 0.12, left_hand=moved, right=JoyConReading(frozenset({"R"})))
    output = feed_hands(
        hands,
        0.14,
        left_hand=moved,
        right_hand=((0.35, -0.2, 1.0), TURNED),
        right=JoyConReading(frozenset({"R"})),
    )
    assert output.hands.poses[1] == target(((0.35, -0.2, 1.0), RIGHT_HAND[1]))
    output = feed_hands(hands, 0.16, left_hand=moved, right_hand=((0.35, -0.2, 1.0), TURNED))
    assert output.hands.poses[1] == target(((0.35, -0.2, 1.0), TURNED))


def test_a_hand_that_jumps_or_is_malformed_stops_following(hands):
    feed_hands(hands, 0.04, left_hand=((0.9, 0.2, 1.0), LEFT_HAND[1]))
    assert not hands.active and "Left hand jumped 0.60 m" in hands.reason
    hands.set_hand("right", ((0.3, -0.2, math.nan), RIGHT_HAND[1]), 0.06)
    assert hands.reason == "Malformed hand pose"


def test_changing_the_arm_source_stops_following_and_small_profiles_refuse_hands(hands):
    hands.set_arm_source("leader")
    assert not hands.active and hands.arm_source == "leader"
    output = feed(hands, 0.04)
    assert output.hands is None
    limits = dict.fromkeys(UPPER, (-2.0, 2.0))
    with pytest.raises(ValueError, match="both Joy-Cons"):
        JoyloController(limits, JoyloConfig(enabled_sides=("left",))).set_arm_source("hands")
    with pytest.raises(ValueError, match="'leader' or 'hands'"):
        hands.set_arm_source("feet")


@pytest.mark.parametrize(
    ("left", "right"),
    [
        (JoyConReading(), JoyConReading()),
        (JoyConReading(frozenset({"down"})), JoyConReading()),
        (JoyConReading(), JoyConReading(stick=(0, 1))),
        (JoyConReading(), JoyConReading(frozenset({"stick"}), (0, 1))),
    ],
)
def test_hands_use_exactly_the_leader_torso_controls(controller, hands, left, right):
    leader = JoyloController(controller.limits, controller.config)
    engage(leader)
    expected = feed(leader, 0.04, left=left, right=right)
    actual = feed_hands(hands, 0.04, left=left, right=right)
    assert [actual.positions[n] for n in TORSO] == pytest.approx(
        [expected.positions[n] for n in TORSO]
    )


def test_moving_hands_does_not_move_the_torso(hands):
    before = feed_hands(hands, 0.04)
    after = feed_hands(hands, 0.06, right_hand=((0.3, -0.25, 1.1), RIGHT_HAND[1]))
    assert before.hands.poses != after.hands.poses
    assert [after.positions[n] for n in TORSO] == [before.positions[n] for n in TORSO]


def test_torso_lowering_reaches_joint_boundary_and_can_raise_again():
    config = JoyloConfig(torso_link_lengths=(0.4, 0.3))
    limits = [(-1.1345, 1.8326), (-2.7925, 2.5307), (-1.8326, 1.5708), (-3.0543, 3.0543)]
    q = [0.0] * 4
    for _ in range(3000):
        q = torso_step(q, -1, 0, 0.02, limits, config)
    x = 0.4 * math.sin(q[0]) + 0.3 * math.sin(q[0] + q[1])
    height = 0.4 * math.cos(q[0]) + 0.3 * math.cos(q[0] + q[1])
    assert height < 0.21
    assert q[2] == pytest.approx(limits[2][0], abs=1e-7)
    assert x == pytest.approx(0.0, abs=1e-8)
    assert q[0] + q[1] - q[2] == pytest.approx(0.0, abs=1e-8)
    assert all(lo <= value <= hi for value, (lo, hi) in zip(q, limits, strict=True))
    raised = torso_step(q, 1, 0, 0.02, limits, config)
    assert 0.4 * math.cos(raised[0]) + 0.3 * math.cos(raised[0] + raised[1]) > height


@pytest.mark.parametrize("torso", [(0.4, -0.8, -0.4, 0), (0.4, -0.8, -0.2, 0.3)])
def test_hand_targets_ride_with_measured_torso(hands, torso):
    before = hands.gripper_target(LEFT_HAND)
    hands.feedback.update(zip(TORSO, torso, strict=True))
    after = hands.gripper_target(LEFT_HAND)
    # Transform the new target back through the torso: the arm-relative pose is unchanged.
    neutral_body = np.eye(4)
    current_body = np.eye(4)
    joints = {joint.name: joint for joint in hands.config.model.load().joints}
    for name, angle in zip(TORSO, torso, strict=True):
        neutral_body = neutral_body @ joints[name].parent_from_child(0)
        current_body = current_body @ joints[name].parent_from_child(angle)

    def relative_target(pose, body):
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat(pose[1]).as_matrix()
        transform[:3, 3] = pose[0]
        return np.linalg.inv(body) @ transform

    np.testing.assert_allclose(
        relative_target(after, current_body), relative_target(before, neutral_body), atol=1e-10
    )
    assert after != before


def test_lowering_level_torso_lowers_both_hand_targets(hands):
    before = [hands.gripper_target(hand) for hand in (LEFT_HAND, RIGHT_HAND)]
    hands.feedback.update(zip(TORSO, (0.4, -0.8, -0.4, 0), strict=True))
    after = [hands.gripper_target(hand) for hand in (LEFT_HAND, RIGHT_HAND)]
    for old, new in zip(before, after, strict=True):
        assert new[0][2] < old[0][2] - 0.05
        assert new[1] == pytest.approx(old[1])
