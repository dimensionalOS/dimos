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

import pytest
from scipy.spatial.transform import Rotation

from dimos.calibration.hand_eye_motion import CalibrationArm, connect_calibration_arm
from dimos.core.introspection.module.info import ModuleInfo, StreamInfo
from dimos.manipulation.manipulation_spec import (
    ExecutionResult,
    ExecutionStatus,
    PlanningGroupInfo,
    PlanningGroupState,
    PlanResult,
    PlanStatus,
)
from dimos.manipulation.sdk import Arm, MotionError
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState

# A seven-joint group whose measured stream includes its gripper and uses a different order.
_JOINTS = tuple(f"left/J{i}" for i in range(1, 8))
_HOME = (0.1, 0.7, -0.3, 0.0, 0.2, -0.1, 1.0)


def arm_state(positions=_HOME, ts=100.0):
    return JointState(
        name=["left/gripper", *reversed(_JOINTS)],
        position=[0.015, *reversed(positions)],
        ts=ts,
    )


def planner_state(pose, positions=_HOME):
    return PlanningGroupState(
        joints=JointState(name=list(_JOINTS), position=list(positions), ts=pose.ts),
        end_effector_pose=pose,
        gripper_position=None,
    )


@pytest.fixture
def calibration_arm(mocker):
    mocker.patch("dimos.calibration.hand_eye_motion.time", return_value=100.0)
    planner = mocker.Mock(spec=Arm)
    planner.info = PlanningGroupInfo("left", _JOINTS, "pedestal", "tcp", True)
    planner.state.return_value = planner_state(PoseStamped(frame_id="world", ts=100.0))
    feedback = mocker.Mock()
    feedback.get_next.return_value = arm_state()
    return CalibrationArm(planner, feedback), planner, feedback


def test_reads_fresh_arm_joints_in_planner_order(calibration_arm):
    arm, planner, feedback = calibration_arm
    feedback.get_next.side_effect = [arm_state((0.0,) * 7, ts=95.0), arm_state()]

    assert arm.read_joints() == _HOME
    planner.move_joints.assert_not_called()


def test_home_accepts_planner_joint_order_and_returns_measured_tcp(calibration_arm):
    arm, planner, feedback = calibration_arm
    feedback.get_next.side_effect = [
        JointState(name=["left/gripper"], position=[0.015], ts=100.0),
        arm_state(ts=95.0),
        arm_state((0.0,) * 7),
        arm_state(),
    ]

    home_tcp = arm.move_joints(_HOME)

    planner.move_joints.assert_called_once_with(_HOME, speed_scale=0.15, timeout=60.0)
    assert home_tcp is planner.state.return_value.end_effector_pose


@pytest.mark.parametrize("home", [[0.0] * 6, [float("nan")] * 7])
def test_unusable_home_is_rejected_before_motion(calibration_arm, home):
    arm, planner, _ = calibration_arm
    with pytest.raises(ValueError, match="7 finite joint positions"):
        arm.move_joints(home)
    planner.move_joints.assert_not_called()


def test_delayed_image_matches_raw_feedback_at_exposure(calibration_arm, marker_image):
    arm, planner, feedback = calibration_arm
    arm.move_joints(_HOME)
    image, _, _ = marker_image
    image.ts = 99.4
    pose = Transform(ts=image.ts)

    def capture():
        observe = feedback.subscribe.call_args.args[0]
        observe(arm_state(ts=99.38))
        # A high-rate arm can publish hundreds more readings before image delivery.
        for i in range(350):
            observe(arm_state((0.0,) * 7, ts=99.65 + i / 1000))
        observe(JointState(name=["left/gripper"], position=[0.015], ts=99.4))
        return image, pose

    reviewed, measured = arm.capture_at_view(capture)

    assert reviewed is image and measured is pose
    feedback.subscribe.return_value.assert_called_once_with()
    planner.move_joints.assert_not_called()


@pytest.mark.parametrize(
    ("reading", "message"),
    [
        (arm_state(ts=98.0), "No fresh"),
        (arm_state((0.0,) * 7), "outside the 2 degree"),
    ],
)
def test_unusable_raw_feedback_prevents_image_review(
    calibration_arm, marker_image, reading, message
):
    arm, _, feedback = calibration_arm
    arm.move_joints(_HOME)
    image, _, _ = marker_image
    image.ts = 100.0

    def capture():
        feedback.subscribe.call_args.args[0](reading)
        return image, Transform(ts=image.ts)

    with pytest.raises(RuntimeError, match=message):
        arm.capture_at_view(capture)
    feedback.subscribe.return_value.assert_called_once_with()


def test_missed_zero_reports_worst_joint_and_aborts_the_move(calibration_arm, mocker):
    arm, planner, feedback = calibration_arm
    feedback.get_next.side_effect = [
        arm_state(),
        arm_state((0.01, 0.0, -0.07, 0.0, 0.0, 0.0, 0.0)),
        TimeoutError(),
    ]

    with pytest.raises(
        RuntimeError,
        match=r"left/J3 measured -0\.0700 rad, requested 0\.0000 rad, error 4\.011° \(limit 2°\)",
    ):
        arm.move_joints((0.0,) * 7)
    planner.cancel.assert_called_once()
    with pytest.raises(RuntimeError, match="before capturing"):
        arm.capture_at_view(mocker.Mock())


def test_cartesian_view_sends_world_pose_and_accepts_measured_ik_solution(
    calibration_arm, marker_image
):
    arm, planner, feedback = calibration_arm
    target = PoseStamped(position=(0.1, 0.2, 0.3), frame_id="world", ts=100.0)
    reached = (0.2,) * 7
    # A measured 5.302 mm and 0.630° offset still gives a usable ring view.
    measured_tcp = PoseStamped(
        position=(0.105302, 0.2, 0.3),
        orientation=Rotation.from_euler("z", 0.630, degrees=True).as_quat(),
        frame_id="world",
        ts=100.0,
    )
    planner.state.return_value = planner_state(measured_tcp, positions=reached)
    feedback.get_next.side_effect = [arm_state(), arm_state(reached), TimeoutError()]

    arm.move_pose(target)

    planner.move_pose.assert_called_once_with(
        (0.1, 0.2, 0.3), orientation=(0.0, 0.0, 0.0, 1.0), speed_scale=0.15, timeout=60.0
    )
    image, _, _ = marker_image
    image.ts = 100.0
    mount = Transform(frame_id="pedestal", child_frame_id="mount", ts=image.ts)

    def capture():
        feedback.subscribe.call_args.args[0](arm_state(reached))
        return image, mount

    assert arm.capture_at_view(capture) == (image, mount)


@pytest.mark.parametrize(
    "snapshot, message",
    [
        (
            planner_state(PoseStamped(position=(0.010001, 0, 0), frame_id="world", ts=100.0)),
            r"position error 10\.001 mm \(limit 10 mm\), rotation error 0\.000° \(limit 2°\)",
        ),
        (
            planner_state(
                PoseStamped(
                    orientation=Rotation.from_euler("z", 2.001, degrees=True).as_quat(),
                    frame_id="world",
                    ts=100.0,
                )
            ),
            r"position error 0\.000 mm \(limit 10 mm\), rotation error 2\.001° \(limit 2°\)",
        ),
        (planner_state(PoseStamped(frame_id="world", ts=90.0)), "does not match fresh"),
        (
            planner_state(PoseStamped(frame_id="world", ts=100.0), positions=(0.0,) * 7),
            "does not match fresh",
        ),
    ],
)
def test_cartesian_view_requires_fresh_matching_achieved_pose(calibration_arm, snapshot, message):
    arm, planner, feedback = calibration_arm
    planner.state.return_value = snapshot
    feedback.get_next.side_effect = [arm_state(), arm_state(), TimeoutError()]

    with pytest.raises(RuntimeError, match=message):
        arm.move_pose(PoseStamped(frame_id="world"))
    planner.cancel.assert_called_once()


@pytest.mark.parametrize(
    "failure",
    [
        KeyboardInterrupt(),
        MotionError("execute", ExecutionResult(ExecutionStatus.TIMED_OUT)),
    ],
)
def test_interrupted_execution_requests_cancellation(calibration_arm, failure):
    arm, planner, _ = calibration_arm
    planner.move_pose.side_effect = failure

    with pytest.raises(type(failure)):
        arm.move_pose(PoseStamped(frame_id="world"))
    planner.cancel.assert_called_once()


@pytest.mark.parametrize(
    "rejection",
    [
        PlanResult(PlanStatus.INVALID_TARGET),
        ExecutionResult(ExecutionStatus.NO_PLAN),
        ExecutionResult(ExecutionStatus.REJECTED, "Pending plan was replaced"),
    ],
)
def test_rejected_move_does_not_cancel_another_clients_motion(calibration_arm, rejection):
    arm, planner, _ = calibration_arm
    planner.move_pose.side_effect = MotionError("move_pose", rejection)

    with pytest.raises(MotionError):
        arm.move_pose(PoseStamped(frame_id="world"))
    planner.cancel.assert_not_called()


@pytest.fixture
def deployed_arm(mocker):
    app = mocker.patch("dimos.calibration.hand_eye_motion.Dimos.connect").return_value
    discover = mocker.patch.object(Arm, "from_app")
    discover.return_value.info = PlanningGroupInfo("left", _JOINTS, "pedestal", "tcp", True)
    app.describe.return_value = ModuleInfo("WristRig", qualified_path="example.WristRig")
    app.list_streams.return_value = [
        StreamInfo("joint_state", "JointState", "WristRig", "out"),
        StreamInfo("joint_state", "JointState", "Logger", "in"),
        StreamInfo("coordinator_joint_state", "JointState", "Coordinator", "out"),
    ]
    return app, discover


def test_discovers_raw_arm_output_on_running_stack(deployed_arm):
    app, discover = deployed_arm
    with connect_calibration_arm() as arm:
        assert arm.info.joint_names == _JOINTS
        assert arm.return_joints == (0.0,) * len(_JOINTS)
    discover.assert_called_once_with(app)
    app.get_module.assert_called_once_with("WristRig")
    app.get_module.return_value.get_stream.assert_called_once_with("joint_state")
    app.stop.assert_called_once()


@pytest.mark.parametrize(
    ("qualified_path", "expected"),
    [
        ("dimos.robot.manipulators.a1z.connection.A1ZConnection", (0, -0.05, 0, 0.05, 0, 0)),
        ("example.A1ZConnection", (0,) * 6),
    ],
)
def test_a1z_return_uses_feedback_source_type_and_planner_joint_order(
    deployed_arm, qualified_path, expected
):
    app, discover = deployed_arm
    names = tuple(f"arm/arm_joint{i}" for i in (1, 3, 4, 2, 5, 6))
    discover.return_value.info = PlanningGroupInfo("arm", names, "base", "tcp", True)
    app.describe.return_value = ModuleInfo("WristRig", qualified_path=qualified_path)

    with connect_calibration_arm() as arm:
        assert arm.return_joints == expected


def test_ambiguous_measured_joint_sources_fail_before_motion(deployed_arm):
    app, discover = deployed_arm
    app.list_streams.return_value.append(StreamInfo("joint_state", "JointState", "OtherRig", "out"))

    with pytest.raises(ValueError, match="WristRig.*OtherRig"), connect_calibration_arm():
        pytest.fail("Ambiguous feedback unexpectedly selected")
    discover.return_value.move_joints.assert_not_called()
    app.stop.assert_called_once()
