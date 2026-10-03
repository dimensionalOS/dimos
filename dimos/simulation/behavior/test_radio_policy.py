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

"""CPU checks for sensor grounding, checked execution and confirmed cancellation."""

import threading
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.manipulation.manipulation_spec import (
    CommandResult,
    CommandStatus,
    ExecutionResult,
    ExecutionStatus,
    PlanningGroupInfo,
)
from dimos.manipulation.sdk import Arm
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.simulation.behavior.radio_policy import (
    RadioPolicy,
    RadioPolicyModule,
    RadioPolicySupervisor,
    camera_observation,
    ground_pixel,
)


@pytest.fixture
def module_transport(mocker):
    # Keep normal configuration/lifecycle while replacing the external transport.
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    mocker.patch("dimos.core.module.get_loop", return_value=(None, None))


@pytest.fixture
def raw_camera(monkeypatch):
    monkeypatch.setattr("dimos.simulation.behavior.radio_policy.time.time", lambda: 123.0)
    frame = "left_wrist_optical"
    return {
        "left_wrist_image": Image(np.zeros((5, 5, 3), dtype=np.uint8), ImageFormat.RGB, frame, 123),
        "left_wrist_depth": Image(
            np.full((5, 5), 2, dtype=np.float32), ImageFormat.DEPTH, frame, 123
        ),
        "left_wrist_camera_info": CameraInfo(
            width=5, height=5, K=[2, 0, 2, 0, 2, 2, 0, 0, 1], frame_id=frame, ts=123
        ),
        "tf": TFMessage(
            Transform(
                translation=Vector3(1, 2, 3),
                rotation=Quaternion(0, 0, 2**-0.5, 2**-0.5),
                frame_id="base_link",
                child_frame_id=frame,
                ts=123,
            ),
            Transform(frame_id="world", child_frame_id="base_link", ts=123),
        ),
        "objects": {"radio": {"position": [9, 8, 7]}},
        "goal_status": {"success": True},
        "odometry": "perfect simulator localization",
        "evaluator_side": "not a policy camera",
    }


def test_observation_excludes_truth_and_world_pose_and_copies_rgb(raw_camera):
    result = camera_observation(raw_camera, "left_wrist")
    assert set(result) == {
        "id",
        "camera",
        "capture",
        "fingerprint",
        "rgb",
        "depth",
        "calibration",
        "camera_to_base",
        "provenance",
    }
    assert result["camera_to_base"].frame_id == "base_link"
    result["rgb"].data[0, 0, 0] = 255
    assert raw_camera["left_wrist_image"].data[0, 0, 0] == 0


def test_grounding_uses_depth_intrinsics_and_rotated_camera_tf(raw_camera):
    observation = camera_observation(raw_camera, "left_wrist")
    result = ground_pixel(observation, 3, 2)
    assert result["position"] == pytest.approx([1, 3, 5])
    assert result["frame"] == "base_link"
    assert result["observation_id"] == observation["id"]
    assert result["pixel"] == [3, 2]


@pytest.mark.parametrize("failure", ["stale", "skew", "frame"])
def test_stale_or_mismatched_sensor_data_rejected(raw_camera, failure):
    if failure == "stale":
        raw_camera["left_wrist_image"].ts = 121
    elif failure == "skew":
        raw_camera["left_wrist_depth"].ts = 122.5
    else:
        raw_camera["left_wrist_image"].frame_id = "wrong_camera"
    with pytest.raises(ValueError):
        camera_observation(raw_camera, "left_wrist")


def test_invalid_depth_never_falls_back_to_radio_truth(raw_camera):
    raw_camera["left_wrist_depth"].data.fill(np.nan)
    with pytest.raises(ValueError, match="valid measured depth"):
        ground_pixel(camera_observation(raw_camera, "left_wrist"), 2, 2)


@pytest.fixture
def supervisor(mocker):
    rpc = mocker.Mock()
    rpc.execute.return_value = ExecutionResult(ExecutionStatus.ACCEPTED)
    rpc.wait_for_execution.return_value = ExecutionResult(ExecutionStatus.COMPLETED)
    rpc.cancel.return_value = ExecutionResult(ExecutionStatus.NO_EXECUTION)
    arm = mocker.Mock(rpc=rpc)
    arm.pose.return_value = PoseStamped(
        frame_id="world", position=[1, 2, 3], orientation=[0, 0, 0, 1]
    )
    motion = mocker.Mock()
    motion.move.side_effect = lambda *args, executor: executor("validated-plan-7", args[2])
    service = RadioPolicySupervisor(
        arm, motion, lambda: {}, lambda: Transform(translation=Vector3(1, 2, 3))
    )
    yield SimpleNamespace(service=service, arm=arm, motion=motion)
    service.close()
    if service._worker:
        service._worker.join(timeout=1)
        assert not service._worker.is_alive()


def test_pose_action_converts_base_target_and_dispatches_same_checked_id(supervisor):
    action = supervisor.service.move_pose([0.1, 0.2, 0.3], [0, 0, 0, 1])
    supervisor.service._worker.join(timeout=1)
    assert supervisor.service.status(action.id).state == "completed"
    supervisor.motion.move.assert_called_once()
    assert supervisor.motion.move.call_args.args[0] == pytest.approx([1.1, 2.2, 3.3])
    supervisor.arm.rpc.execute.assert_called_once_with(
        blocking=False, timeout=20, plan_id="validated-plan-7"
    )


def test_cancel_during_checking_prevents_later_dispatch(supervisor):
    entered, release = threading.Event(), threading.Event()

    def delayed(*args, executor):
        entered.set()
        assert release.wait(timeout=1)
        executor("validated-plan-7", args[2])

    supervisor.motion.move.side_effect = delayed
    action = supervisor.service.move_pose([0.1, 0, 0], [0, 0, 0, 1])
    assert entered.wait(timeout=1)
    try:
        result = supervisor.service.cancel(action.id)
        assert result.stop_confirmed and result.state == "cancelled"
        with pytest.raises(RuntimeError, match="busy"):
            supervisor.service.move_pose([0.2, 0, 0], [0, 0, 0, 1])
    finally:
        release.set()
    supervisor.service._worker.join(timeout=1)
    supervisor.arm.rpc.execute.assert_not_called()
    assert supervisor.service.status(action.id).state == "cancelled"


def test_unconfirmed_stop_blocks_next_action_and_hides_internal_error(supervisor):
    supervisor.motion.move.side_effect = RuntimeError("oracle radio position [1,2,3]")
    supervisor.arm.rpc.cancel.return_value = ExecutionResult(ExecutionStatus.UNCERTAIN)
    action = supervisor.service.move_pose([0.1, 0, 0], [0, 0, 0, 1])
    supervisor.service._worker.join(timeout=1)
    result = supervisor.service.status(action.id)
    assert result.state == "uncertain" and not result.stop_confirmed
    assert result.error == "stop_unconfirmed"
    with pytest.raises(RuntimeError, match="unconfirmed"):
        supervisor.service.move_pose([0.2, 0, 0], [0, 0, 0, 1])


def test_press_rejects_large_or_nonfinite_displacement(supervisor):
    for delta in ([0.03, 0, 0], [float("nan"), 0, 0], [0, 0, 0]):
        with pytest.raises(ValueError):
            supervisor.service.press(delta)
    supervisor.motion.move.assert_not_called()


def test_gripper_facade_uses_selected_sdk_group_and_returns_acceptance_only(mocker):
    proxy = mocker.Mock()
    accepted = CommandResult(CommandStatus.SUCCEEDED)
    proxy.set_gripper_position.return_value = accepted
    arm = Arm(proxy, PlanningGroupInfo("right_arm", (), "world", "right_tip", True))
    service = RadioPolicySupervisor(arm, mocker.Mock(), lambda: {}, Transform.identity)
    try:
        result = RadioPolicy(service).set_gripper_position(0.25)
        assert result is accepted
        proxy.set_gripper_position.assert_called_once_with(0.25, planning_group="right_arm")
        assert service._action is None  # Acceptance is not a completed motion/grasp.
    finally:
        service.close()


@pytest.mark.parametrize("value", [-0.1, 1.1, float("nan"), float("inf")])
def test_policy_gripper_rejects_invalid_values_without_dispatch(supervisor, value):
    with pytest.raises(ValueError, match="normalized gripper travel"):
        supervisor.service.set_gripper_position(value)
    supervisor.arm.set_gripper_position.assert_not_called()


def test_gripper_cannot_race_motion_or_bypass_uncertain_stop(supervisor):
    entered, release = threading.Event(), threading.Event()

    def delayed(*args, executor):
        entered.set()
        assert release.wait(timeout=1)
        executor("validated-plan-7", args[2])

    supervisor.motion.move.side_effect = delayed
    supervisor.arm.rpc.cancel.return_value = ExecutionResult(ExecutionStatus.UNCERTAIN)
    action = supervisor.service.move_pose([0.1, 0, 0], [0, 0, 0, 1])
    assert entered.wait(timeout=1)
    try:
        with pytest.raises(RuntimeError, match="busy"):
            supervisor.service.set_gripper_position(0.5)
        assert not supervisor.service.cancel(action.id).stop_confirmed
    finally:
        release.set()
        supervisor.service._worker.join(timeout=1)
    with pytest.raises(RuntimeError, match="unconfirmed"):
        supervisor.service.set_gripper_position(0.5)
    supervisor.arm.set_gripper_position.assert_not_called()


def test_lost_gripper_reply_latches_further_commands_and_hides_internal_error(supervisor):
    supervisor.arm.set_gripper_position.side_effect = TimeoutError("private actuator diagnostic")
    with pytest.raises(RuntimeError, match="^gripper_command_unconfirmed$"):
        supervisor.service.set_gripper_position(0.5)
    with pytest.raises(RuntimeError, match="unconfirmed"):
        supervisor.service.set_gripper_position(0.25)
    with pytest.raises(RuntimeError, match="unconfirmed"):
        supervisor.service.move_pose([0.1, 0, 0], [0, 0, 0, 1])
    supervisor.arm.set_gripper_position.assert_called_once_with(0.5)


@pytest.mark.parametrize("groups", [("torso",), ()])
def test_owner_binds_right_arm_and_exact_auxiliary_groups(
    mocker, tmp_path, groups, module_transport
):
    app = mocker.Mock()
    sim = mocker.Mock()
    sim.describe.return_value = {
        "policy_visuals": {"toggle_markers_hidden": True, "hidden_count": 1}
    }
    app.get_module.side_effect = lambda name: sim if name == "BehaviorConnection" else mocker.Mock()
    mocker.patch("dimos.porcelain.dimos.Dimos.connect", return_value=app)
    selected = mocker.patch("dimos.simulation.behavior.radio_policy.Arm.from_app")
    factory = mocker.patch("dimos.simulation.behavior.radio_motion.make_development_motion")
    module = RadioPolicyModule(arm="right_arm", auxiliary_groups=groups)
    try:
        module.initialize_development_scene(
            {"physical_to_sdk_fk_translation": [0, 0, 0]}, str(tmp_path)
        )
        selected.assert_called_once_with(app, group="right_arm", instance_name="ManipulationModule")
        assert factory.call_args.args[2] is selected.return_value
        assert factory.call_args.args[5] == groups
    finally:
        module.stop()
    app.stop.assert_called_once()


def test_watchdog_cancels_execution_and_late_completion_does_not_overwrite(supervisor, mocker):
    timer = mocker.patch("dimos.simulation.behavior.radio_policy.threading.Timer")
    entered, release = threading.Event(), threading.Event()

    def wait(timeout):
        entered.set()
        assert release.wait(timeout=1)
        return ExecutionResult(ExecutionStatus.COMPLETED)

    supervisor.arm.rpc.wait_for_execution.side_effect = wait
    supervisor.arm.rpc.cancel.return_value = ExecutionResult(ExecutionStatus.ABORTED)
    action = supervisor.service.move_pose([0.1, 0, 0], [0, 0, 0, 1], timeout=0.5)
    assert entered.wait(timeout=1)
    try:
        _, callback = timer.call_args.args
        callback(*timer.call_args.kwargs["args"])
        result = supervisor.service.status(action.id)
        assert result.cancel_requested and result.stop_confirmed and result.state == "cancelled"
    finally:
        release.set()
    supervisor.service._worker.join(timeout=1)
    assert supervisor.service.status(action.id).state == "cancelled"


def test_cancel_failure_is_uncertain_not_a_successful_stop(supervisor):
    entered, release = threading.Event(), threading.Event()

    def delayed(*args, executor):
        entered.set()
        assert release.wait(timeout=1)
        executor("validated-plan-7", args[2])

    supervisor.motion.move.side_effect = delayed
    supervisor.arm.rpc.cancel.side_effect = RuntimeError("transport lost")
    action = supervisor.service.move_pose([0.1, 0, 0], [0, 0, 0, 1])
    assert entered.wait(timeout=1)
    try:
        result = supervisor.service.cancel(action.id)
        assert result.state == "uncertain" and not result.stop_confirmed
    finally:
        release.set()
    supervisor.service._worker.join(timeout=1)
    supervisor.arm.rpc.execute.assert_not_called()


def test_grounding_rejects_replaced_observation_id_and_expired_frames(raw_camera, mocker):
    service = RadioPolicySupervisor(
        mocker.Mock(), mocker.Mock(), lambda: raw_camera, Transform.identity
    )
    first = service.observe()
    assert service.ground(first["id"], 3, 2)["position"] == pytest.approx([1, 3, 5])
    second = service.observe()
    with pytest.raises(ValueError, match="replaced"):
        service.ground(first["id"], 3, 2)
    mocker.patch("dimos.simulation.behavior.radio_policy.time.time", return_value=134)
    with pytest.raises(ValueError, match="expired"):
        service.ground(second["id"], 3, 2)
    service.close()


def test_owner_initialization_rejects_unhidden_toggle_visuals(mocker, tmp_path, module_transport):
    app = mocker.Mock()
    app.get_module.return_value.describe.return_value = {
        "policy_visuals": {"toggle_markers_hidden": False}
    }
    mocker.patch("dimos.porcelain.dimos.Dimos.connect", return_value=app)
    module = RadioPolicyModule()
    try:
        with pytest.raises(RuntimeError, match="hidden diagnostic toggle markers"):
            module.initialize_development_scene({}, str(tmp_path))
        app.stop.assert_called_once()
        with pytest.raises(RuntimeError, match="not initialized"):
            module.observe()
    finally:
        module.stop()


def test_checkpoint_owner_uses_verified_factory_instead_of_legacy_guard(
    mocker, tmp_path, module_transport
):
    app = mocker.Mock()
    sim = mocker.Mock()
    sim.describe.return_value = {
        "policy_visuals": {"toggle_markers_hidden": True, "hidden_count": 1}
    }
    app.get_module.side_effect = lambda name: sim if name == "BehaviorConnection" else mocker.Mock()
    mocker.patch("dimos.porcelain.dimos.Dimos.connect", return_value=app)
    mocker.patch("dimos.simulation.behavior.radio_policy.Arm.from_app")
    checkpoint = mocker.patch("dimos.simulation.behavior.radio_policy.make_radio_grasp_checkpoint")
    legacy = mocker.patch("dimos.simulation.behavior.radio_motion.make_development_motion")
    scene = {"physical_to_sdk_fk_translation": [0, 0, 0]}
    module = RadioPolicyModule(
        arm="right_arm", auxiliary_groups=("torso",), motion_contract="checkpoint"
    )
    try:
        module.initialize_development_scene(scene, str(tmp_path))
        checkpoint.assert_called_once_with(app, sim, scene, module._development_evidence)
        legacy.assert_not_called()
        assert module._supervisor._motion.checkpoint is checkpoint.return_value
    finally:
        module.stop()
