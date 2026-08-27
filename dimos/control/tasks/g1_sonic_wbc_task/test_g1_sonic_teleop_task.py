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
from typing import Any

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


def _start_session(task: G1SonicTeleopTask) -> None:
    task.on_body_tracking(_body_snapshot(), t_now=1.0)
    task.on_teleop_buttons(_buttons(a=True, b=True, x=True, y=True), t_now=1.0)
    task.on_teleop_buttons(_buttons(), t_now=1.001)


def _fill_pose_buffer(task: G1SonicTeleopTask) -> None:
    for index in range(1, 10):
        capture_time = 1.0 + 0.02 * index
        task.on_body_tracking(
            _body_snapshot(capture_time_s=capture_time),
            t_now=capture_time,
        )


def _enter_pose(task: G1SonicTeleopTask) -> None:
    _start_session(task)
    _fill_pose_buffer(task)
    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.19)


@pytest.fixture
def task_and_pipeline(mocker: Any) -> Iterator[tuple[G1SonicTeleopTask, Any]]:
    pipeline_class = mocker.patch(
        "dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_wbc_task.SonicPipeline"
    )
    pipeline = pipeline_class.return_value
    pipeline.apply_pose_message.return_value = {"frames": 10, "encode_mode": 2}
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


def test_start_combo_enters_planner_and_has_priority_over_ax(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    task.on_body_tracking(_body_snapshot(), t_now=1.0)

    task.on_teleop_buttons(_buttons(a=True, b=True, x=True, y=True), t_now=1.0)

    teleop = task.state_snapshot()["webxr_teleop"]
    assert teleop["mode"] == "planner"
    assert teleop["buffered_frames"] == 0
    assert task.state_snapshot()["reference_source"] == "planner"
    pipeline.apply_pose_message.assert_not_called()


def test_ax_is_ignored_while_teleop_is_off(task_and_pipeline: tuple[Any, Any]) -> None:
    task, pipeline = task_and_pipeline
    task.on_body_tracking(_body_snapshot(), t_now=1.0)

    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.0)

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "off"
    pipeline.apply_pose_message.assert_not_called()


def test_pose_requires_complete_ten_frame_buffer(task_and_pipeline: tuple[Any, Any]) -> None:
    task, pipeline = task_and_pipeline
    _start_session(task)

    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.01)

    teleop = task.state_snapshot()["webxr_teleop"]
    assert teleop["mode"] == "planner"
    assert teleop["last_transition_reason"] == "pose_buffer_not_ready"
    pipeline.apply_pose_message.assert_not_called()


def test_low_latency_pipeline_requires_two_frames_and_is_reported(mocker: Any) -> None:
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
        _start_session(task)
        task.on_body_tracking(_body_snapshot(capture_time_s=1.02), t_now=1.02)
        task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.03)

        teleop = task.state_snapshot()["webxr_teleop"]
        fields = pipeline.apply_pose_message.call_args.args[0]
        assert teleop["mode"] == "pose"
        assert teleop["sonic_pipeline"] == "sonic-low-latency"
        assert teleop["pose_window_frames"] == 2
        assert fields["frame_index"].tolist() == [0, 1]
    finally:
        task.stop()


def test_pose_data_is_applied_before_stream_source_is_selected(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _start_session(task)
    _fill_pose_buffer(task)
    pipeline.reset_mock()
    pipeline.apply_pose_message.return_value = {"frames": 10, "encode_mode": 2}

    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.19)

    fields = pipeline.apply_pose_message.call_args.args[0]
    assert fields["frame_index"].tolist() == list(range(10))
    assert fields["smpl_joints"].shape == (10, 24, 3)
    call_names = [call[0] for call in pipeline.method_calls]
    assert call_names.index("apply_pose_message") < call_names.index("set_source_stream")
    assert task.state_snapshot()["webxr_teleop"]["mode"] == "pose"


def test_accepted_pose_publishes_exact_sonic_reference(
    task_and_pipeline: tuple[Any, Any], mocker: Any
) -> None:
    task, pipeline = task_and_pipeline
    publish = mocker.Mock()
    task.set_pose_reference_publisher(publish)
    publish.reset_mock()

    _enter_pose(task)

    fields = pipeline.apply_pose_message.call_args.args[0]
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


def test_ax_toggles_pose_back_to_balancing_planner(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _enter_pose(task)
    task.on_teleop_buttons(_buttons(), t_now=1.20)

    task.on_teleop_buttons(_buttons(a=True, x=True), t_now=1.21)

    assert task.control_state is SonicControlState.CONTROL
    assert task.state_snapshot()["webxr_teleop"]["mode"] == "planner"
    assert task.state_snapshot()["reference_source"] == "planner"
    pipeline.stop_clip.assert_called()


def test_start_combo_stops_teleop_without_disarming_policy(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _enter_pose(task)
    task.on_teleop_buttons(_buttons(), t_now=1.20)

    task.on_teleop_buttons(_buttons(a=True, b=True, x=True, y=True), t_now=1.21)

    assert task.control_state is SonicControlState.CONTROL
    assert task.state_snapshot()["webxr_teleop"]["mode"] == "off"
    assert task.state_snapshot()["reference_source"] == "planner"
    pipeline.stop_clip.assert_called()


def test_webxr_cannot_start_while_policy_is_unarmed(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    assert task.disarm()
    pipeline.apply_pose_message.reset_mock()
    task.on_body_tracking(_body_snapshot(), t_now=1.0)

    task.on_teleop_buttons(_buttons(a=True, b=True, x=True, y=True), t_now=1.0)

    assert task.control_state is SonicControlState.UNARMED
    assert task.state_snapshot()["webxr_teleop"]["mode"] == "off"
    assert task.state_snapshot()["webxr_teleop"]["last_transition_reason"] == "policy_inactive"
    pipeline.apply_pose_message.assert_not_called()


def test_tracking_loss_in_pose_returns_to_planner(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _enter_pose(task)

    task.on_body_tracking(_body_snapshot(available=False), t_now=1.20)

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "planner"
    assert task.state_snapshot()["webxr_teleop"]["stream_ready"] is False
    pipeline.stop_clip.assert_called()


def test_tracking_reference_change_invalidates_session(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, _ = task_and_pipeline
    _enter_pose(task)

    task.on_body_tracking(
        _body_snapshot(capture_time_s=1.20, frame_id="bounded-floor"),
        t_now=1.20,
    )

    teleop = task.state_snapshot()["webxr_teleop"]
    assert teleop["mode"] == "off"
    assert teleop["last_transition_reason"] == "tracking_reference_changed"


def test_stale_tracking_in_pose_returns_to_planner(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    _enter_pose(task)

    task.compute(_state(1.34))

    assert task.state_snapshot()["webxr_teleop"]["mode"] == "planner"
    assert task.state_snapshot()["webxr_teleop"]["last_transition_reason"] == (
        "body_tracking_stale"
    )
    pipeline.stop_clip.assert_called()


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
