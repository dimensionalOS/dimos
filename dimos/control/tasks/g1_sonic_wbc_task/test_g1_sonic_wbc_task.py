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

from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_wbc_task import (
    G1SonicWBCTask,
    G1SonicWBCTaskConfig,
    SonicControlState,
    _create_task,
)
from dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline import DEFAULT_ANGLES_DDS
from dimos.hardware.whole_body.spec import IMUState

_JOINT_NAMES = [f"joint_{index}" for index in range(29)]


def _state(t_now: float, positions: float = 0.0) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions=dict.fromkeys(_JOINT_NAMES, positions),
            joint_velocities=dict.fromkeys(_JOINT_NAMES, 0.0),
        ),
        imu={"g1": IMUState()},
        t_now=t_now,
        dt=0.02,
    )


@pytest.fixture
def make_task(mocker: Any):
    pipeline_class = mocker.patch(
        "dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_wbc_task.SonicPipeline"
    )
    pipeline = pipeline_class.return_value
    pipeline.step.return_value = np.zeros(29, dtype=np.float32)
    pipeline.snapshot.return_value = {"stream_active": False}

    def factory(
        *,
        auto_arm: bool,
        default_ramp_seconds: float,
        auto_dry_run: bool = False,
    ) -> G1SonicWBCTask:
        config = G1SonicWBCTaskConfig(
            encoder_onnx=Path("encoder.onnx"),
            decoder_onnx=Path("decoder.onnx"),
            planner_onnx=Path("planner.onnx"),
            joint_names=_JOINT_NAMES,
            auto_arm=auto_arm,
            auto_dry_run=auto_dry_run,
            default_ramp_seconds=default_ramp_seconds,
            zmq_enabled=False,
        )
        return G1SonicWBCTask("sonic", config, mocker.MagicMock())

    return factory, pipeline


def test_auto_arm_finishes_ramp_before_first_policy_step(make_task: Any) -> None:
    factory, pipeline = make_task
    task = factory(auto_arm=True, default_ramp_seconds=0.0)
    task.start()

    initialization_output = task.compute(_state(1.0))

    assert task.control_state is SonicControlState.CONTROL
    assert initialization_output is not None
    assert initialization_output.positions == pytest.approx(DEFAULT_ANGLES_DDS.tolist())
    pipeline.step.assert_not_called()

    task.compute(_state(1.02))

    pipeline.step.assert_called_once()


def test_start_without_auto_arm_holds_measured_pose(make_task: Any) -> None:
    factory, pipeline = make_task
    task = factory(auto_arm=False, default_ramp_seconds=3.0)
    task.start()

    output = task.compute(_state(10.0, positions=0.25))

    assert task.control_state is SonicControlState.UNARMED
    assert output is not None and output.positions == pytest.approx([0.25] * 29)
    snapshot = task.state_snapshot()
    assert snapshot["active"] is True
    assert snapshot["armed"] is False
    assert snapshot["arming"] is False
    assert snapshot["arm_pending"] is False
    assert snapshot["dry_run"] is False
    assert snapshot["arming_duration"] == 3.0
    pipeline.step.assert_not_called()


def test_arm_snapshots_current_pose_then_ramps_to_default(make_task: Any) -> None:
    factory, pipeline = make_task
    task = factory(auto_arm=False, default_ramp_seconds=3.0)
    task.start()
    task.compute(_state(9.0, positions=0.25))
    assert task.arm()
    assert task.state_snapshot()["arm_pending"] is True

    first = task.compute(_state(10.0, positions=0.25))
    halfway = task.compute(_state(11.5, positions=0.25))
    complete = task.compute(_state(13.0, positions=0.25))

    assert first is not None and first.positions == pytest.approx([0.25] * 29)
    assert halfway is not None
    expected_halfway = 0.25 + 0.5 * (DEFAULT_ANGLES_DDS - 0.25)
    assert halfway.positions == pytest.approx(expected_halfway.tolist())
    assert complete is not None and complete.positions == pytest.approx(DEFAULT_ANGLES_DDS.tolist())
    assert task.control_state is SonicControlState.CONTROL
    pipeline.step.assert_not_called()


def test_manual_arm_starts_policy_only_after_ramp(make_task: Any) -> None:
    factory, pipeline = make_task
    task = factory(auto_arm=False, default_ramp_seconds=0.0)
    task.start()
    task.compute(_state(1.0))

    assert task.control_state is SonicControlState.UNARMED
    assert task.arm()

    task.compute(_state(1.02))
    assert task.control_state is SonicControlState.CONTROL
    pipeline.step.assert_not_called()

    task.compute(_state(1.04))
    pipeline.step.assert_called_once()


def test_disarm_returns_to_measured_pose_hold_and_planner(make_task: Any) -> None:
    factory, pipeline = make_task
    task = factory(auto_arm=True, default_ramp_seconds=0.0)
    task.start()
    task.compute(_state(1.0))
    task.compute(_state(1.02))
    task._select_stream_reference(True)

    assert task.disarm()
    hold = task.compute(_state(2.0, positions=0.3))

    snapshot = task.state_snapshot()
    assert snapshot["control_state"] == "unarmed"
    assert snapshot["armed"] is False
    assert snapshot["reference_source"] == "planner"
    assert hold is not None and hold.positions == pytest.approx([0.3] * 29)
    pipeline.reset.assert_called()


def test_reset_reactivate_replays_arm_ramp(make_task: Any) -> None:
    factory, _pipeline = make_task
    task = factory(auto_arm=True, default_ramp_seconds=0.0)
    task.start()
    task.compute(_state(1.0))

    assert task.reset_runtime_state(reactivate=True)

    snapshot = task.state_snapshot()
    assert snapshot["control_state"] == "unarmed"
    assert snapshot["arm_pending"] is True


def test_dry_run_outputs_arm_ramp_but_suppresses_policy_output(make_task: Any) -> None:
    factory, pipeline = make_task
    task = factory(auto_arm=True, default_ramp_seconds=0.0, auto_dry_run=True)
    task.start()

    ramp_output = task.compute(_state(1.0))
    policy_output = task.compute(_state(1.02))

    assert ramp_output is not None
    assert policy_output is None
    pipeline.step.assert_called_once()


def test_policy_timing_is_observational(make_task: Any) -> None:
    factory, _pipeline = make_task
    task = factory(auto_arm=True, default_ramp_seconds=0.0)

    task._record_policy_timing(0.201, 1.0)
    task._record_policy_timing(0.005, 1.02)

    assert task._policy_timing_snapshot() == {
        "step_ms": {"samples": 2, "mean": 103.0, "p95": 191.2, "p99": 199.04, "max": 201.0},
        "start_interval_ms": {
            "samples": 1,
            "mean": 20.0,
            "p95": 20.0,
            "p99": 20.0,
            "max": 20.0,
        },
    }


def test_task_factory_fails_fast_when_selected_model_bundle_is_missing(tmp_path: Path) -> None:
    cfg = SimpleNamespace(
        name="sonic",
        joint_names=_JOINT_NAMES,
        priority=50,
        params={
            "encoder_onnx": tmp_path / "low_latency/model_encoder.onnx",
            "decoder_onnx": tmp_path / "low_latency/model_decoder.onnx",
            "planner_onnx": tmp_path / "planner_sonic.onnx",
            "hardware_id": "g1",
            "sonic_pipeline": "sonic-low-latency",
        },
    )

    with pytest.raises(FileNotFoundError, match="setup-sonic-models"):
        _create_task(cfg, {}, G1SonicWBCTask)
