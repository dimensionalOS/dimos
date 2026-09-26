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

import numpy as np
import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.g1_sonic_wbc_task.coordinator import SonicCoordinator
from dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_wbc_task import (
    G1SonicWBCTask,
    G1SonicWBCTaskConfig,
)
from dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline import DEFAULT_ANGLES_DDS
from dimos.control.tasks.g1_sonic_wbc_task.sonic_safety import damping_commands
from dimos.core.global_config import global_config
from dimos.hardware.whole_body.spec import IMUState
from dimos.msgs.std_msgs.String import String

_JOINT_NAMES = [f"joint_{index}" for index in range(29)]


def _state(t_now, positions=0.0):
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
def task(mocker):
    pipeline = mocker.patch(
        "dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_wbc_task.SonicPipeline"
    ).return_value
    pipeline.step.return_value = np.ones(29, dtype=np.float32)
    config = G1SonicWBCTaskConfig(
        encoder_onnx="encoder.onnx",
        decoder_onnx="decoder.onnx",
        planner_onnx="planner.onnx",
        joint_names=_JOINT_NAMES,
    )
    task = G1SonicWBCTask("sonic", config, mocker.Mock())
    yield task
    task.stop()


def _arm(task):
    task.start()
    task.arm()
    task.compute(_state(0.0))
    return task.compute(_state(3.0))


def test_hold_and_measured_pose_ramp_precede_policy_output(task):
    task.start()
    hold = task.compute(_state(9.0, positions=0.25))
    assert hold.positions == pytest.approx([0.25] * 29)

    task.arm()
    first = task.compute(_state(10.0, positions=0.4))
    halfway = task.compute(_state(11.5, positions=0.4))
    complete = task.compute(_state(13.0, positions=0.4))

    assert first.positions == pytest.approx([0.4] * 29)
    assert halfway.positions == pytest.approx((0.4 + 0.5 * (DEFAULT_ANGLES_DDS - 0.4)).tolist())
    assert complete.positions == pytest.approx(DEFAULT_ANGLES_DDS.tolist())
    task._pipeline.step.assert_not_called()
    assert task.compute(_state(13.02)).positions == [1.0] * 29


def test_dry_run_republishes_hold_while_policy_runs(task):
    ramp = _arm(task)
    task.set_dry_run(True)

    first = task.compute(_state(3.02))
    second = task.compute(_state(3.04))

    assert first.positions == second.positions == ramp.positions
    assert first.positions != [1.0] * 29
    assert task._pipeline.step.call_count == 2


@pytest.mark.parametrize(
    "operator_stop, reason", [(False, "decoder failed"), (True, "operator stop")]
)
def test_fault_during_inference_discards_targets_and_stays_latched(task, operator_stop, reason):
    _arm(task)

    def inference(**kwargs):
        if operator_stop:
            task.set_estop(True)
            return np.ones(29, dtype=np.float32)
        raise RuntimeError("decoder failed")

    task._pipeline.step.side_effect = inference

    assert task.compute(_state(3.02)) is None
    assert task.compute(_state(3.04)) is None
    assert task.fault_reason == reason
    task._adapter.write_motor_commands.assert_called_with(damping_commands(29))
    task._pipeline.step.assert_called_once()
    assert task.reset_runtime_state(reactivate=True) is False
    with pytest.raises(RuntimeError, match="restart"):
        task.arm()
    with pytest.raises(RuntimeError, match="restart"):
        task.set_estop(False)


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "source, reason", [("operator", "operator stop"), ("hardware", "stale feedback")]
)
async def test_coordinator_routes_faults_without_triggering_motor_takeover(
    task, mocker, monkeypatch, source, reason
):
    coordinator = SonicCoordinator(publish_joint_state=False)
    monkeypatch.setattr(global_config, "simulation", "")
    publish = mocker.patch.object(coordinator.sonic_fault, "publish")
    try:
        coordinator.add_task(task)
        coordinator._setup_from_config()

        if source == "operator":
            coordinator.set_estop(True)
        else:
            await coordinator.handle_g1_fault(String(reason))

        assert task.fault_reason == reason
        assert publish.call_args.args[0].data == reason
        task._adapter.write_motor_commands.assert_not_called()
    finally:
        coordinator.stop()
