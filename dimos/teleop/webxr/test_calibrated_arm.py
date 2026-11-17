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

"""The generic calibration adapter carries targets with feedback and gates stale input."""

import asyncio

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.teleop.webxr.arm_calibration import ArmAnchors, CalibrationPose, CalibrationProfile
from dimos.teleop.webxr.calibrated_arm import CalibratedArmTeleopModule
from dimos.teleop.webxr.controller_types import Hand, WebXRControllerState
from dimos.teleop.webxr.extensions import ArmTeleopModule


@pytest.fixture
def module(tmp_path, mocker):
    path = tmp_path / "robot.urdf"
    path.write_text("""<robot name="test"><link name="base_link"/><link name="body"/>
      <link name="left"/><link name="right"/><link name="left_shoulder"/><link name="right_shoulder"/>
      <joint name="sl" type="fixed"><parent link="body"/><child link="left_shoulder"/><origin xyz="0 .2 0"/></joint>
      <joint name="sr" type="fixed"><parent link="body"/><child link="right_shoulder"/><origin xyz="0 -.2 0"/></joint>
      <joint name="lift" type="prismatic"><parent link="base_link"/><child link="body"/>
      <axis xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint>
      <joint name="left_arm" type="prismatic"><parent link="body"/><child link="left"/>
      <origin xyz="0.3 0.2 0"/><axis xyz="1 0 0"/><limit lower="0" upper="0.5" effort="1" velocity="1"/></joint>
      <joint name="right_arm" type="prismatic"><parent link="body"/><child link="right"/>
      <origin xyz="0.3 -0.2 0"/><axis xyz="1 0 0"/><limit lower="0" upper="0.5" effort="1" velocity="1"/></joint></robot>""")
    mocker.patch.object(ArmTeleopModule, "start")
    mocker.patch("dimos.teleop.webxr.calibrated_arm.time.monotonic", return_value=10)
    mocker.patch("dimos.teleop.webxr.calibrated_arm.time.time", return_value=100)
    m = CalibratedArmTeleopModule(
        model=RobotModel.from_file(path),
        body_frame="body",
        target_frames={"left": "left", "right": "right"},
        shoulder_frames={"left": "left_shoulder", "right": "right_shoulder"},
        wrist_frames={"left": "left", "right": "right"},
        neutral_joints={},
        reach_joints={"left_arm": 0.3, "right_arm": 0.3},
        calibration_directory=tmp_path / "profiles",
    )
    m._calibration_control = mocker.Mock()
    mocker.patch.object(m.robot_feedback, "subscribe", return_value=lambda: None)
    for name in (
        "arm_calibration_state",
        "arm_retarget_state",
        "webxr_pose_observed",
        "webxr_head_observed",
        "webxr_joy_observed",
    ):
        mocker.patch.object(getattr(m, name), "publish")
    m.start()
    yield m
    m.dispose()


def ready(m):
    c = m._calibration
    q = (0, 0, 0, 1)
    c.profile = CalibrationProfile(
        name="test",
        input_kind="hands",
        device="hand",
        robot=c.robot,
        saved_at=100,
        arms={
            s: ArmAnchors(
                shoulder=(0, 0, 0),
                neutral=CalibrationPose(position=(0.3, 0, 0), orientation=q),
                reach=CalibrationPose(position=(0.6, 0, 0), orientation=q),
            )
            for s in ("left", "right")
        },
    )
    c.active = True
    return c


def test_calibrated_targets_follow_body_height_and_stop_with_stale_feedback(module, mocker):
    m = module
    c = ready(m)
    raw = PoseStamped(ts=1, frame_id="left", position=(0.6, 0, 0), orientation=(0, 0, 0, 1))
    m._current_poses[Hand.LEFT] = raw
    m._head_receipt = 10
    c.observe("left", m._pose(raw), np.eye(4), 10, 1)
    m._on_calibration_feedback(JointState(ts=100, name=["lift"], position=[0.4]))
    target = m._get_output_pose(Hand.LEFT)
    assert target.frame_id == "base_link"
    assert target.position.to_list() == pytest.approx([0.6, 0.2, 0.4])
    mocker.patch("dimos.teleop.webxr.calibrated_arm.time.time", return_value=101)
    assert m._get_output_pose(Hand.LEFT) is None


def test_replayed_hand_input_does_not_keep_target_alive(module, mocker):
    m = module
    c = ready(m)
    raw = PoseStamped(ts=1, frame_id="left", position=(0.6, 0, 0), orientation=(0, 0, 0, 1))
    m._current_poses[Hand.LEFT] = raw
    m._head_receipt = 11
    c.observe("left", m._pose(raw), np.eye(4), 10, 1)
    m._on_calibration_feedback(JointState(ts=100, name=["lift"], position=[0.4]))
    mocker.patch("dimos.teleop.webxr.calibrated_arm.time.monotonic", return_value=11)
    assert m._get_output_pose(Hand.LEFT) is None


def test_capture_holds_and_cancel_never_restores_engaged_buttons(module, mocker):
    m = module
    forward = mocker.patch.object(ArmTeleopModule, "_publish_button_state")
    m.begin_arm_calibration("Alice", "controllers", "grips", "session")
    m._calibration_control.hold_for_arm_calibration.assert_called_once_with(True)
    m._publish_button_state(WebXRControllerState(grip=1), WebXRControllerState(grip=1))
    forward.assert_called_with(None, None)
    m.cancel_arm_calibration("session")
    assert not m._calibration.active
    m._calibration_control.hold_for_arm_calibration.assert_called_with(False)


def test_head_samples_convert_to_the_same_operator_frame_and_reject_bad_quaternions(module):
    asyncio.run(
        module.handle_webxr_head(
            PoseStamped(ts=1, position=[0.2, 1.6, -0.4], orientation=[0, 0, 0, 1])
        )
    )
    assert module._head[:3, 3] == pytest.approx([0.4, -0.2, 1.6])
    asyncio.run(
        module.handle_webxr_head(PoseStamped(ts=2, position=[0, 0, 0], orientation=[0, 0, 0, 0]))
    )
    assert module._head_stamp == 1


def test_looking_straight_down_keeps_tracking_with_last_observable_heading(module):
    asyncio.run(
        module.handle_webxr_head(PoseStamped(ts=1, position=[0, 1.6, 0], orientation=[0, 0, 0, 1]))
    )
    # The headset's forward direction is now vertical, with no observable yaw.
    asyncio.run(
        module.handle_webxr_head(
            PoseStamped(ts=2, position=[0.2, 1.5, 0], orientation=[2**-0.5, 0, 0, 2**-0.5])
        )
    )
    assert module._head_stamp == 2
    assert module._head[:3, 3] == pytest.approx([0, -0.2, 1.5])
    assert module._head[:3, :3] == pytest.approx(np.eye(3))


@pytest.mark.parametrize("head_first", [True, False])
def test_turns_pair_head_and_hands_from_same_frame_despite_delivery_order(module, head_first):
    m = module
    c = ready(m)
    head_position = np.array([0, 1.6, 0])
    hands = {"left": np.array([-0.2, 1.2, -0.3]), "right": np.array([0.2, 1.2, -0.3])}
    asyncio.run(
        m.handle_webxr_head(PoseStamped(ts=1, position=head_position, orientation=[0, 0, 0, 1]))
    )
    for side, position in hands.items():
        m._on_pose(PoseStamped(ts=1, frame_id=side, position=position, orientation=[0, 0, 0, 1]))
    expected = {side: c.map(side) for side in hands}
    turn = Rotation.from_euler("y", 1.2)
    translation = np.array([2, 0.2, -0.4])
    head = PoseStamped(
        ts=2, position=turn.apply(head_position) + translation, orientation=turn.as_quat()
    )
    if head_first:
        asyncio.run(m.handle_webxr_head(head))
        # A newer head frame must not reinterpret a delayed hand from frame 2.
        asyncio.run(
            m.handle_webxr_head(PoseStamped(ts=3, position=[4, 1, 2], orientation=[0, 0, 0, 1]))
        )
    for side, position in hands.items():
        m._on_pose(
            PoseStamped(
                ts=2,
                frame_id=side,
                position=turn.apply(position) + translation,
                orientation=turn.as_quat(),
            )
        )
    if not head_first:
        assert {side: c.map(side) for side in hands} == expected
        asyncio.run(m.handle_webxr_head(head))
    for side in hands:
        actual = c.map(side)
        assert actual.position == pytest.approx(expected[side].position)
        rotation_error = (
            Rotation.from_quat(actual.orientation)
            * Rotation.from_quat(expected[side].orientation).inv()
        )
        assert rotation_error.magnitude() < 1e-10
    assert not m._pending_hands


def test_unpaired_input_expires_without_late_head_reviving_it(module, mocker):
    m = module
    c = ready(m)
    m._on_pose(
        PoseStamped(ts=1, frame_id="left", position=[-0.2, 1.2, -0.3], orientation=[0, 0, 0, 1])
    )
    assert c.map("left") is None
    mocker.patch("dimos.teleop.webxr.calibrated_arm.time.monotonic", return_value=11)
    asyncio.run(
        m.handle_webxr_head(PoseStamped(ts=1, position=[0, 1.6, 0], orientation=[0, 0, 0, 1]))
    )
    assert c.map("left") is None
    assert not m._pending_hands
