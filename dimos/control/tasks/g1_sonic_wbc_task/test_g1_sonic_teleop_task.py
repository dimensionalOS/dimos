# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Iterator
from pathlib import Path
from typing import Any, cast

import numpy as np
import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_teleop_task import G1SonicTeleopTask
from dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_wbc_task import (
    G1SonicWBCTaskConfig,
    SonicControlState,
)
from dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline import WRIST_ONNX_INDICES
from dimos.control.tasks.g1_sonic_wbc_task.webxr_retargeting import SMPL_WEBXR_JOINTS
from dimos.hardware.whole_body.spec import IMUState
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.teleop.webxr.body_tracking import BodyJointPose, BodyTrackingSnapshot
from dimos.teleop.webxr.controller_types import Buttons

_JOINT_NAMES = [f"joint_{index}" for index in range(29)]


def _body_snapshot(
    *,
    capture_time_s: float = 1.0,
    frame_id: str = "local-floor",
    omitted: frozenset[str] = frozenset(),
    available: bool = True,
) -> BodyTrackingSnapshot:
    joints = None
    if available:
        joints = {
            name: BodyJointPose(
                position=(0.0, float(index) * 0.01, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0),
            )
            for index, name in enumerate(SMPL_WEBXR_JOINTS)
            if name not in omitted
        }
    return BodyTrackingSnapshot(
        type="body_tracking_snapshot",
        capture_time_s=capture_time_s,
        frame_id=frame_id,
        joints=joints,
    )


def _buttons(*, a: bool = False, b: bool = False, x: bool = False, y: bool = False) -> Buttons:
    buttons = Buttons()
    buttons.right_primary = a
    buttons.right_secondary = b
    buttons.left_primary = x
    buttons.left_secondary = y
    return buttons


def _state(t_now: float, dt: float = 0.02) -> CoordinatorState:
    joints = JointStateSnapshot(
        joint_positions=dict.fromkeys(_JOINT_NAMES, 0.0),
        joint_velocities=dict.fromkeys(_JOINT_NAMES, 0.0),
    )
    return CoordinatorState(joints=joints, imu={"g1": IMUState()}, t_now=t_now, dt=dt)


def _prime_pose_stream(task: G1SonicTeleopTask) -> None:
    task.on_body_tracking(_body_snapshot(), t_now=1.0)


def _fill_pose_buffer(task: G1SonicTeleopTask) -> None:
    for index in range(1, 10):
        capture_time = 1.0 + 0.02 * index
        task.on_body_tracking(
            _body_snapshot(capture_time_s=capture_time),
            t_now=capture_time,
        )


def _start_pose_transition(task: G1SonicTeleopTask) -> None:
    _prime_pose_stream(task)
    _fill_pose_buffer(task)
    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.19)


def _enter_pose(task: G1SonicTeleopTask) -> None:
    _start_pose_transition(task)
    cast("Any", task._pipeline).reference_transition_active = False
    task.compute(_state(1.20))


@pytest.fixture
def task_and_pipeline(mocker: Any) -> Iterator[tuple[G1SonicTeleopTask, Any]]:
    pipeline_class = mocker.patch(
        "dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_wbc_task.SonicPipeline"
    )
    pipeline = pipeline_class.return_value
    pipeline.set_pose_window.return_value = {"frames": 10, "encode_mode": 2}
    pipeline.begin_stream_transition.return_value = True
    pipeline.prepare_planner_transition.return_value = True
    pipeline.planner_transition_ready = False
    pipeline.reference_transition_active = True
    pipeline.reference_transition_progress = 0.0

    def begin_planner_transition(_duration: float) -> bool:
        pipeline.reference_transition_active = True
        return True

    pipeline.begin_planner_transition.side_effect = begin_planner_transition
    pipeline.step.return_value = np.zeros(29, dtype=np.float32)
    pipeline.snapshot.return_value = {}
    adapter = mocker.MagicMock()
    adapter.read_imu.return_value = IMUState()
    config = G1SonicWBCTaskConfig(
        encoder_onnx=Path("encoder.onnx"),
        decoder_onnx=Path("decoder.onnx"),
        planner_onnx=Path("planner.onnx"),
        joint_names=_JOINT_NAMES,
        auto_arm=True,
        default_ramp_seconds=0.0,
        zmq_enabled=False,
    )
    task = G1SonicTeleopTask("sonic_teleop", config, adapter)
    task.start()
    task.compute(_state(0.5))
    task.compute(_state(0.52))

    yield task, pipeline

    task.stop()


def test_live_policy_enters_planner_without_controller_buttons(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline

    teleop = task.state_snapshot()["webxr_teleop"]
    assert teleop["mode"] == "planner"
    assert teleop["buffered_frames"] == 0
    assert task.state_snapshot()["reference_source"] == "planner"
    pipeline.set_pose_window.assert_not_called()


def test_dry_run_keeps_webxr_planner_available(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, _ = task_and_pipeline

    task.set_dry_run(True)
    assert task.state_snapshot()["webxr_teleop"]["mode"] == "planner"


def test_dry_run_pose_preview_runs_sonic_without_actuator_output(
    task_and_pipeline: tuple[Any, Any], mocker: Any
) -> None:
    task, pipeline = task_and_pipeline
    publish = mocker.Mock()
    task.set_pose_reference_publisher(publish)
    task.set_dry_run(True)
    publish.reset_mock()

    _enter_pose(task)
    output = task.compute(_state(1.20))

    snapshot = task.state_snapshot()
    assert output is None
    assert snapshot["dry_run"] is True
    assert snapshot["webxr_teleop"]["mode"] == "pose"
    assert snapshot["reference_source"] == "webxr_pose"
    pipeline.set_pose_window.assert_called()
    assert publish.call_args.args[0].active is True


def test_ax_starts_smooth_transition_from_planner(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline

    _start_pose_transition(task)

    snapshot = task.state_snapshot()
    assert snapshot["webxr_teleop"]["mode"] == "pose_transition"
    assert snapshot["webxr_teleop"]["pose_transition_seconds"] == 0.5
    assert snapshot["webxr_teleop"]["pose_transition_progress"] == 0.0
    assert snapshot["reference_source"] == "planner_to_webxr_pose"
    pipeline.begin_stream_transition.assert_called_once_with(0.5)


def test_slow_policy_timing_does_not_gate_pose(task_and_pipeline: tuple[Any, Any]) -> None:
    task, _pipeline = task_and_pipeline
    for index in range(10):
        task._record_policy_timing(0.201, 1.0 + index * 0.02)

    _start_pose_transition(task)

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "pose_transition"


def test_completed_transition_enters_pose(task_and_pipeline: tuple[Any, Any]) -> None:
    task, pipeline = task_and_pipeline
    _start_pose_transition(task)
    pipeline.reference_transition_active = False

    task.compute(_state(1.20))

    snapshot = task.state_snapshot()
    assert snapshot["webxr_teleop"]["mode"] == "pose"
    assert snapshot["webxr_teleop"]["pose_transition_progress"] == 1.0
    assert snapshot["webxr_teleop"]["last_transition_reason"] == "pose_transition_complete"
    assert snapshot["reference_source"] == "webxr_pose"


def test_ax_holds_pose_while_preparing_fresh_planner(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _start_pose_transition(task)
    task.on_teleop_buttons(_buttons(), t_now=1.20)

    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.21)

    snapshot = task.state_snapshot()
    assert snapshot["webxr_teleop"]["mode"] == "planner_prepare"
    assert snapshot["reference_source"] == "webxr_pose_held_for_planner"
    pipeline.prepare_planner_transition.assert_called_once_with()
    pipeline.begin_planner_transition.assert_not_called()


def test_pose_transition_rejects_missing_planner_reference(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    pipeline.begin_stream_transition.return_value = False

    _start_pose_transition(task)

    snapshot = task.state_snapshot()
    assert snapshot["webxr_teleop"]["mode"] == "planner"
    assert snapshot["webxr_teleop"]["last_transition_reason"] == ("planner_reference_not_ready")
    assert snapshot["reference_source"] == "planner"


def test_pose_updates_continue_during_transition(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _start_pose_transition(task)
    initial_calls = pipeline.set_pose_window.call_count

    task.on_body_tracking(_body_snapshot(capture_time_s=1.20), t_now=1.20)
    task.compute(_state(1.20))

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "pose_transition"
    assert pipeline.set_pose_window.call_count == initial_calls + 1


def test_tracking_loss_during_transition_returns_to_planner(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _start_pose_transition(task)

    task.on_body_tracking(_body_snapshot(available=False), t_now=1.20)

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "planner_prepare"
    assert task.state_snapshot()["reference_source"] == "webxr_pose_held_for_planner"
    pipeline.prepare_planner_transition.assert_called_once_with()


def test_enabling_from_dry_run_pose_returns_to_planner_before_output(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    task.set_dry_run(True)
    _enter_pose(task)
    pipeline.stop_clip.reset_mock()

    task.set_dry_run(False)

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "planner"
    assert task.state_snapshot()["reference_source"] == "planner"
    pipeline.stop_clip.assert_called_once_with()
    pipeline.reset.assert_called()


def test_ax_is_ignored_while_policy_is_unarmed(task_and_pipeline: tuple[Any, Any]) -> None:
    task, pipeline = task_and_pipeline
    assert task.disarm()
    task.on_body_tracking(_body_snapshot(), t_now=1.0)

    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.0)

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "off"
    pipeline.set_pose_window.assert_not_called()


def test_pose_requires_complete_ten_frame_buffer(task_and_pipeline: tuple[Any, Any]) -> None:
    task, pipeline = task_and_pipeline
    _prime_pose_stream(task)

    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.01)

    teleop = task.state_snapshot()["webxr_teleop"]
    assert teleop["mode"] == "planner"
    assert teleop["last_transition_reason"] == "pose_buffer_not_ready"
    pipeline.set_pose_window.assert_not_called()


def test_low_latency_pipeline_requires_four_frames_and_is_reported(mocker: Any) -> None:
    pipeline_class = mocker.patch(
        "dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_wbc_task.SonicPipeline"
    )
    pipeline = pipeline_class.return_value
    pipeline.snapshot.return_value = {}
    pipeline.step.return_value = np.zeros(29, dtype=np.float32)
    adapter = mocker.MagicMock()
    adapter.read_imu.return_value = IMUState()
    config = G1SonicWBCTaskConfig(
        encoder_onnx=Path("encoder.onnx"),
        decoder_onnx=Path("decoder.onnx"),
        planner_onnx=Path("planner.onnx"),
        joint_names=_JOINT_NAMES,
        sonic_pipeline="sonic-low-latency",
        auto_arm=True,
        default_ramp_seconds=0.0,
        zmq_enabled=False,
    )
    task = G1SonicTeleopTask("sonic_teleop", config, adapter)
    try:
        task.start()
        task.compute(_state(0.5))
        task.compute(_state(0.52))
        _prime_pose_stream(task)
        for capture_time in (1.02, 1.04, 1.06):
            task.on_body_tracking(_body_snapshot(capture_time_s=capture_time), t_now=capture_time)
        task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.07)

        teleop = task.state_snapshot()["webxr_teleop"]
        fields = pipeline.set_pose_window.call_args.args[0]
        assert teleop["mode"] == "pose_transition"
        assert teleop["sonic_pipeline"] == "sonic-low-latency"
        assert teleop["pose_window_frames"] == 4
        assert fields["frame_index"].tolist() == [0, 1, 2, 3]
    finally:
        task.stop()


def test_pose_data_is_applied_before_stream_source_is_selected(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _prime_pose_stream(task)
    _fill_pose_buffer(task)
    pipeline.reset_mock()
    pipeline.set_pose_window.return_value = {"frames": 10, "encode_mode": 2}

    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.19)

    fields = pipeline.set_pose_window.call_args.args[0]
    assert fields["frame_index"].tolist() == list(range(10))
    assert fields["smpl_joints"].shape == (10, 24, 3)
    call_names = [call[0] for call in pipeline.method_calls]
    assert call_names.index("set_pose_window") < call_names.index("begin_stream_transition")
    assert task.state_snapshot()["webxr_teleop"]["mode"] == "pose_transition"


def test_accepted_pose_publishes_exact_sonic_reference(
    task_and_pipeline: tuple[Any, Any], mocker: Any
) -> None:
    task, pipeline = task_and_pipeline
    publish = mocker.Mock()
    task.set_pose_reference_publisher(publish)
    publish.reset_mock()

    _enter_pose(task)

    fields = pipeline.set_pose_window.call_args.args[0]
    reference = publish.call_args.args[0]
    assert reference.active is True
    np.testing.assert_array_equal(reference.frame_indices, fields["frame_index"])
    np.testing.assert_array_equal(reference.smpl_joints, fields["smpl_joints"])
    np.testing.assert_array_equal(reference.body_quat_w, fields["body_quat_w"])
    np.testing.assert_array_equal(
        reference.wrist_joint_pos,
        fields["joint_pos"][:, WRIST_ONNX_INDICES],
    )


def test_leaving_pose_clears_sonic_reference(
    task_and_pipeline: tuple[Any, Any], mocker: Any
) -> None:
    task, _ = task_and_pipeline
    publish = mocker.Mock()
    task.set_pose_reference_publisher(publish)
    _enter_pose(task)
    task.on_teleop_buttons(_buttons(), t_now=1.20)

    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.21)

    assert publish.call_args.args[0].active is False


def test_ax_transitions_pose_back_to_balancing_planner(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _enter_pose(task)
    task.on_teleop_buttons(_buttons(), t_now=1.20)

    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.21)

    assert task.control_state is SonicControlState.CONTROL
    assert task.state_snapshot()["webxr_teleop"]["mode"] == "planner_prepare"
    assert task.state_snapshot()["reference_source"] == "webxr_pose_held_for_planner"
    pipeline.prepare_planner_transition.assert_called_once_with()


def test_completed_planner_transition_enters_planner_and_preserves_reason(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _enter_pose(task)
    task.on_teleop_buttons(_buttons(), t_now=1.20)
    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.21)
    pipeline.planner_transition_ready = True
    task.compute(_state(1.22))
    pipeline.reference_transition_active = False

    task.compute(_state(1.24))

    snapshot = task.state_snapshot()
    assert snapshot["webxr_teleop"]["mode"] == "planner"
    assert snapshot["webxr_teleop"]["last_transition_reason"] == "operator_planner_toggle"
    assert snapshot["reference_source"] == "planner"


def test_ax_is_ignored_during_planner_transition(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _enter_pose(task)
    task.on_teleop_buttons(_buttons(), t_now=1.20)
    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.21)
    pipeline.planner_transition_ready = True
    task.compute(_state(1.22))
    task.on_teleop_buttons(_buttons(), t_now=1.22)
    pipeline.begin_stream_transition.reset_mock()

    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.23)

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "planner_transition"
    pipeline.begin_stream_transition.assert_not_called()


def test_abxy_does_not_change_pose_mode(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _enter_pose(task)
    task.on_teleop_buttons(_buttons(), t_now=1.20)
    pipeline.stop_clip.reset_mock()

    task.on_teleop_buttons(_buttons(a=True, b=True, x=True, y=True), t_now=1.21)

    assert task.control_state is SonicControlState.CONTROL
    assert task.state_snapshot()["webxr_teleop"]["mode"] == "pose"
    assert task.state_snapshot()["reference_source"] == "webxr_pose"
    pipeline.stop_clip.assert_not_called()


def test_entering_dry_run_from_live_pose_keeps_preview_active(
    task_and_pipeline: tuple[Any, Any], mocker: Any
) -> None:
    task, pipeline = task_and_pipeline
    publish = mocker.Mock()
    task.set_pose_reference_publisher(publish)
    _enter_pose(task)
    publish.reset_mock()
    pipeline.stop_clip.reset_mock()

    task.set_dry_run(True)

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "pose"
    assert task.state_snapshot()["reference_source"] == "webxr_pose"
    publish.assert_not_called()
    pipeline.stop_clip.assert_not_called()


def test_webxr_stays_off_while_policy_is_unarmed(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    assert task.disarm()
    pipeline.set_pose_window.reset_mock()
    task.on_body_tracking(_body_snapshot(), t_now=1.0)

    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.0)

    assert task.control_state is SonicControlState.UNARMED
    assert task.state_snapshot()["webxr_teleop"]["mode"] == "off"
    pipeline.set_pose_window.assert_not_called()


def test_tracking_loss_in_pose_returns_to_planner(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _enter_pose(task)

    task.on_body_tracking(_body_snapshot(available=False), t_now=1.20)
    task.compute(_state(1.22))

    teleop = task.state_snapshot()["webxr_teleop"]
    assert teleop["mode"] == "planner_prepare"
    assert teleop["stream_ready"] is False
    assert teleop["last_transition_reason"] == "body_tracking_unavailable"
    pipeline.prepare_planner_transition.assert_called_once_with()


def test_tracking_reference_change_returns_to_planner(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, _ = task_and_pipeline
    _enter_pose(task)

    task.on_body_tracking(
        _body_snapshot(capture_time_s=1.20, frame_id="bounded-floor"),
        t_now=1.20,
    )

    teleop = task.state_snapshot()["webxr_teleop"]
    assert teleop["mode"] == "planner_prepare"
    assert teleop["last_transition_reason"] == "tracking_reference_changed"


def test_stale_tracking_in_pose_returns_to_planner(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _enter_pose(task)

    task.compute(_state(2.17))

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "pose"
    pipeline.begin_planner_transition.assert_not_called()

    task.compute(_state(2.19))

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "planner_prepare"
    assert task.state_snapshot()["webxr_teleop"]["last_transition_reason"] == (
        "body_tracking_stale"
    )
    pipeline.prepare_planner_transition.assert_called_once_with()


def test_capture_gap_holds_pose_until_window_refills(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _enter_pose(task)
    applied_before_gap = pipeline.set_pose_window.call_count

    task.on_body_tracking(_body_snapshot(capture_time_s=1.40), t_now=1.40)

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "pose"
    assert task.state_snapshot()["webxr_teleop"]["last_transition_reason"] == (
        "body_tracking_refilling"
    )
    pipeline.prepare_planner_transition.assert_not_called()

    for index in range(1, 10):
        capture_time = 1.40 + 0.02 * index
        task.on_body_tracking(_body_snapshot(capture_time_s=capture_time), t_now=capture_time)
    task.compute(_state(1.58))

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "pose"
    assert pipeline.set_pose_window.call_count == applied_before_gap + 1
    pipeline.prepare_planner_transition.assert_not_called()


def test_capture_gap_returns_to_planner_if_window_does_not_refill(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _enter_pose(task)

    task.on_body_tracking(_body_snapshot(capture_time_s=1.40), t_now=1.40)
    task.on_body_tracking(_body_snapshot(capture_time_s=1.70), t_now=1.70)
    task.compute(_state(2.39))

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "pose"
    pipeline.prepare_planner_transition.assert_not_called()

    task.compute(_state(2.41))

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "planner_prepare"
    assert task.state_snapshot()["webxr_teleop"]["last_transition_reason"] == (
        "body_tracking_refill_timeout"
    )
    pipeline.prepare_planner_transition.assert_called_once_with()


def test_pose_twist_ignores_translation_and_applies_yaw(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _enter_pose(task)
    task.on_twist_command(
        Twist(linear=Vector3(1.0, 2.0, 0.0), angular=Vector3(0.0, 0.0, 0.5)),
        t_now=1.19,
    )

    task.compute(_state(1.20, dt=0.02))

    pipeline.set_velocity.assert_called_with(0.0, 0.0, 0.0)
    pipeline.apply_heading_increment.assert_called_once_with(0.01)
