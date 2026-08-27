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
from typing import Any

import numpy as np
import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_wbc_task import (
    G1SonicWBCTask,
    G1SonicWBCTaskConfig,
    SonicControlState,
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

    def factory(*, auto_start: bool, initialization_seconds: float) -> G1SonicWBCTask:
        config = G1SonicWBCTaskConfig(
            encoder_onnx=Path("encoder.onnx"),
            decoder_onnx=Path("decoder.onnx"),
            planner_onnx=Path("planner.onnx"),
            joint_names=_JOINT_NAMES,
            auto_start_policy=auto_start,
            initialization_seconds=initialization_seconds,
            zmq_enabled=False,
        )
        return G1SonicWBCTask("sonic", config, mocker.MagicMock())

    return factory, pipeline


def test_auto_start_finishes_initialization_before_first_policy_step(make_task: Any) -> None:
    factory, pipeline = make_task
    task = factory(auto_start=True, initialization_seconds=0.0)
    task.start()

    initialization_output = task.compute(_state(1.0))

    assert task.control_state is SonicControlState.CONTROL
    assert initialization_output is not None
    assert initialization_output.positions == pytest.approx(DEFAULT_ANGLES_DDS.tolist())
    pipeline.step.assert_not_called()

    task.compute(_state(1.02))

    pipeline.step.assert_called_once()


def test_initialization_ramps_before_latched_start_enters_control(make_task: Any) -> None:
    factory, pipeline = make_task
    task = factory(auto_start=True, initialization_seconds=3.0)
    task.start()

    first = task.compute(_state(10.0))
    halfway = task.compute(_state(11.5))
    complete = task.compute(_state(13.0))

    assert first is not None and first.positions == pytest.approx([0.0] * 29)
    assert halfway is not None
    assert halfway.positions == pytest.approx((DEFAULT_ANGLES_DDS * 0.5).tolist())
    assert complete is not None and complete.positions == pytest.approx(DEFAULT_ANGLES_DDS.tolist())
    assert task.control_state is SonicControlState.CONTROL
    pipeline.step.assert_not_called()


def test_manual_arm_starts_policy_only_after_ready(make_task: Any) -> None:
    factory, pipeline = make_task
    task = factory(auto_start=False, initialization_seconds=0.0)
    task.start()
    task.compute(_state(1.0))

    assert task.control_state is SonicControlState.READY
    assert task.arm()

    task.compute(_state(1.02))
    assert task.control_state is SonicControlState.CONTROL
    pipeline.step.assert_not_called()

    task.compute(_state(1.04))
    pipeline.step.assert_called_once()


def test_disarm_returns_to_ready_and_planner_without_reinitializing(make_task: Any) -> None:
    factory, pipeline = make_task
    task = factory(auto_start=True, initialization_seconds=0.0)
    task.start()
    task.compute(_state(1.0))
    task.compute(_state(1.02))
    task._select_stream_reference(True)

    assert task.disarm()

    snapshot = task.state_snapshot()
    assert snapshot["control_state"] == "ready"
    assert snapshot["reference_source"] == "planner"
    assert not snapshot["policy_start_requested"]
    pipeline.reset.assert_called()


def test_reset_reactivate_replays_initialization(make_task: Any) -> None:
    factory, _pipeline = make_task
    task = factory(auto_start=True, initialization_seconds=0.0)
    task.start()
    task.compute(_state(1.0))

    assert task.reset_runtime_state(reactivate=True)

    snapshot = task.state_snapshot()
    assert snapshot["control_state"] == "initializing"
    assert snapshot["policy_start_requested"] is True
