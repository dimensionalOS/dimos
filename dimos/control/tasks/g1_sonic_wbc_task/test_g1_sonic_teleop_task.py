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
from dimos.control.tasks.g1_sonic_wbc_task.webxr_retargeting import SMPL_WEBXR_JOINTS
from dimos.hardware.whole_body.spec import IMUState
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.teleop.webxr.body_tracking import BodyJointPose, BodyTrackingSnapshot
from dimos.teleop.webxr.controller_types import Buttons

_JOINT_NAMES = [f"joint_{index}" for index in range(29)]


def _body_snapshot(
    *,
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
        capture_time_s=1.0,
        frame_id=frame_id,
        joints=joints,
    )


def _deadman(held: bool) -> Buttons:
    buttons = Buttons()
    buttons.left_primary = held
    buttons.right_primary = held
    return buttons


def _state(t_now: float, dt: float = 0.02) -> CoordinatorState:
    joints = JointStateSnapshot(
        joint_positions=dict.fromkeys(_JOINT_NAMES, 0.0),
        joint_velocities=dict.fromkeys(_JOINT_NAMES, 0.0),
    )
    return CoordinatorState(joints=joints, imu={"g1": IMUState()}, t_now=t_now, dt=dt)


@pytest.fixture
def task_and_pipeline(mocker: Any) -> Iterator[tuple[G1SonicTeleopTask, Any]]:
    pipeline_class = mocker.patch(
        "dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_wbc_task.SonicPipeline"
    )
    pipeline = pipeline_class.return_value
    pipeline.apply_pose_message.return_value = {"frames": 1, "encode_mode": 2}
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


def test_deadman_engages_full_body_stream(task_and_pipeline: tuple[Any, Any]) -> None:
    task, pipeline = task_and_pipeline
    task.on_body_tracking(_body_snapshot(), t_now=1.0)
    task.on_teleop_buttons(_deadman(True), t_now=1.0)

    task.compute(_state(1.01))

    pose_fields = pipeline.apply_pose_message.call_args.args[0]
    assert pose_fields["smpl_joints"].shape == (1, 24, 3)
    pipeline.set_source_stream.assert_called_with(True)
    assert task.state_snapshot()["webxr_teleop"]["engaged"] is True


def test_webxr_cannot_engage_while_policy_is_unarmed(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    assert task.disarm()
    pipeline.apply_pose_message.reset_mock()

    task.on_body_tracking(_body_snapshot(), t_now=1.0)
    task.on_teleop_buttons(_deadman(True), t_now=1.0)
    task.compute(_state(1.01))

    assert task.control_state is SonicControlState.UNARMED
    assert task.state_snapshot()["webxr_teleop"]["engaged"] is False
    pipeline.apply_pose_message.assert_not_called()


def test_disarm_clears_webxr_engagement(task_and_pipeline: tuple[Any, Any]) -> None:
    task, pipeline = task_and_pipeline
    task.on_body_tracking(_body_snapshot(), t_now=1.0)
    task.on_teleop_buttons(_deadman(True), t_now=1.0)
    task.compute(_state(1.01))

    assert task.disarm()

    teleop = task.state_snapshot()["webxr_teleop"]
    assert teleop["engaged"] is False
    assert teleop["deadman_held"] is False
    assert task.state_snapshot()["reference_source"] == "planner"
    pipeline.stop_clip.assert_called()


def test_deadman_release_returns_to_planner_without_stopping_policy(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    task.on_body_tracking(_body_snapshot(), t_now=1.0)
    task.on_teleop_buttons(_deadman(True), t_now=1.0)
    task.compute(_state(1.01))
    policy_steps_before_release = pipeline.step.call_count

    task.on_teleop_buttons(_deadman(False), t_now=1.02)
    task.compute(_state(1.03))

    assert task.control_state is SonicControlState.CONTROL
    assert pipeline.step.call_count == policy_steps_before_release + 1
    pipeline.stop_clip.assert_called_once_with()
    assert task.state_snapshot()["reference_source"] == "planner"


def test_engaged_twist_ignores_translation_and_applies_yaw(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    task.on_body_tracking(_body_snapshot(), t_now=1.0)
    task.on_teleop_buttons(_deadman(True), t_now=1.0)
    task.on_twist_command(
        Twist(linear=Vector3(1.0, 2.0, 0.0), angular=Vector3(0.0, 0.0, 0.5)),
        t_now=1.0,
    )

    task.compute(_state(1.01, dt=0.02))

    pipeline.set_velocity.assert_called_with(0.0, 0.0, 0.0)
    pipeline.apply_heading_increment.assert_called_once_with(0.01)


def test_partial_frame_holds_last_complete_pose_for_150ms(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    task.on_body_tracking(_body_snapshot(), t_now=1.0)
    task.on_teleop_buttons(_deadman(True), t_now=1.0)
    task.compute(_state(1.01))
    task.on_body_tracking(_body_snapshot(omitted=frozenset({"head"})), t_now=1.05)

    task.compute(_state(1.14))

    assert pipeline.apply_pose_message.call_count == 1
    assert task.state_snapshot()["webxr_teleop"]["engaged"] is True


def test_stale_body_disengages_and_requires_deadman_repress(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    task.on_body_tracking(_body_snapshot(), t_now=1.0)
    task.on_teleop_buttons(_deadman(True), t_now=1.0)
    task.compute(_state(1.01))

    task.compute(_state(1.16))
    task.on_body_tracking(_body_snapshot(), t_now=1.17)

    teleop = task.state_snapshot()["webxr_teleop"]
    assert teleop["engaged"] is False
    assert teleop["blocked_until_release"] is True
    pipeline.stop_clip.assert_called_once_with()


def test_tracking_unavailable_disengages_immediately(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    task.on_body_tracking(_body_snapshot(), t_now=1.0)
    task.on_teleop_buttons(_deadman(True), t_now=1.0)
    task.compute(_state(1.01))

    task.on_body_tracking(_body_snapshot(available=False), t_now=1.02)

    assert task.state_snapshot()["webxr_teleop"]["engaged"] is False
    pipeline.stop_clip.assert_called_once_with()


def test_invalid_complete_pose_disengages_instead_of_crashing_tick(
    task_and_pipeline: tuple[Any, Any],
) -> None:
    task, pipeline = task_and_pipeline
    snapshot = _body_snapshot()
    assert snapshot.joints is not None
    joints = dict(snapshot.joints)
    joints["head"] = BodyJointPose(
        position=joints["head"].position,
        orientation=(0.0, 0.0, 0.0, 0.0),
    )
    invalid = snapshot.model_copy(update={"joints": joints})
    task.on_body_tracking(invalid, t_now=1.0)
    task.on_teleop_buttons(_deadman(True), t_now=1.0)

    task.compute(_state(1.01))

    teleop = task.state_snapshot()["webxr_teleop"]
    assert teleop["engaged"] is False
    assert teleop["last_disengage_reason"] == "invalid_body_pose"
    pipeline.stop_clip.assert_called_once_with()
