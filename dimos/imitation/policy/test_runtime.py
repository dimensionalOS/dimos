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

"""Shared rollout behavior without network inference or robot hardware."""

from collections.abc import Iterator
from threading import Event
import time
from typing import Any

import numpy as np
import pytest

from dimos.control.tasks.eef_trajectory_task.trajectory import EefTrajectory, EefTrajectoryPoint
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.imitation.policy import runtime as policy_runtime
from dimos.imitation.policy.dreamlabs.backend import DreamlabsBackend
from dimos.imitation.policy.runtime import _PolicyRuntime
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
def rollout(mocker: Any) -> Iterator[tuple[_PolicyRuntime, Any, Any]]:
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    for name in ("__init__", "serve_module_rpc", "start", "stop"):
        mocker.patch.object(LCMRPC, name, return_value=None)
    module = _PolicyRuntime(
        _isolated_python_runtime=True,
        rpc_transport=LCMRPC,
        backend="dreamlabs",
        policy_path="dreamlabs/dream-1-beta",
        task="test task",
        joint_names=["joint"],
    )
    backend = mocker.MagicMock()
    backend.observation_period = None
    backend.observations_required = 0
    backend.n_action_steps = 1
    backend.chunk_size = 1
    backend.fps = 20.0
    backend.action_lower = backend.action_upper = None
    backend.predict.return_value = np.array([[0.1]], dtype=np.float32)
    control = mocker.MagicMock()
    control.list_tasks.return_value = ["joint_trajectory", "eef_trajectory"]
    control.execute.return_value = TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)
    control.eef_execute.return_value = TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)
    control.cancel.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.CANCELLED
    )
    control.eef_cancel.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.CANCELLED
    )
    control.status.return_value = TrajectoryStatus(state=TrajectoryState.EXECUTING)

    def invoke(task_name: str, method: str, kwargs: dict[str, Any]) -> Any:
        assert task_name in {"joint_trajectory", "eef_trajectory"}
        if method == "execute":
            execute = control.eef_execute if task_name == "eef_trajectory" else control.execute
            return execute(
                kwargs["trajectory"],
                **{key: value for key, value in kwargs.items() if key != "trajectory"},
            )
        if method == "cancel":
            assert kwargs == {}
            cancel = control.eef_cancel if task_name == "eef_trajectory" else control.cancel
            return cancel()
        assert method in {"get_status", "keepalive"}
        return control.status(**kwargs)

    control.task_invoke.side_effect = invoke

    mocker.patch.object(module, "_load_policy", return_value=backend)
    mocker.patch.object(module, "_control", control, create=True)
    provide_observation(module)
    try:
        yield module, backend, control
    finally:
        module.stop()


def provide_observation(module: _PolicyRuntime, *, ts: float | None = None) -> None:
    stamp = time.time() if ts is None else ts
    module._on_image(
        "color_image",
        Image(data=np.zeros((2, 3, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=stamp),
    )
    module._on_joint_state(JointState(ts=stamp, name=["joint"], position=[0.0]))


def test_preflight_never_requests_remote_inference(rollout: tuple[Any, Any, Any]) -> None:
    module, backend, control = rollout
    assert module.preflight_rollout()["policy_ready"] is True
    backend.predict.assert_not_called()
    backend.observe.assert_not_called()
    control.execute.assert_not_called()


def test_remote_rollout_defaults_to_preview(rollout: tuple[Any, Any, Any]) -> None:
    module, backend, control = rollout
    module.preflight_rollout()
    module.start_rollout()
    wait_until(lambda: module.rollout_status()["chunks_received"] >= 1, timeout=1)
    module.stop_rollout()
    assert module.rollout_status()["chunks_accepted"] == 0
    assert module.get_last_preview().points[1].positions == pytest.approx([0.1])
    control.execute.assert_not_called()
    control.cancel.assert_not_called()


def test_stop_discards_a_delayed_inference_and_blocks_restart_until_drained(
    rollout: tuple[Any, Any, Any], mocker: Any
) -> None:
    module, backend, control = rollout
    entered, release = Event(), Event()

    def predict(*args: Any, **kwargs: Any) -> np.ndarray:
        entered.set()
        assert release.wait(timeout=2)
        return np.array([[0.1]], dtype=np.float32)

    backend.predict.side_effect = predict
    mocker.patch.object(policy_runtime, "DEFAULT_THREAD_JOIN_TIMEOUT", 0.01)
    module.preflight_rollout()
    module.start_rollout(dispatch_actions=True)
    assert entered.wait(timeout=1)
    try:
        assert module.stop_rollout()["busy"] is True
        assert module.start_rollout()["active"] is False
    finally:
        release.set()
    wait_until(lambda: not module.rollout_status()["busy"], timeout=1)
    assert module.rollout_status()["chunks_received"] == 0
    control.execute.assert_not_called()


def test_cancel_uncertainty_prevents_restart(rollout: tuple[Any, Any, Any]) -> None:
    module, backend, control = rollout
    control.cancel.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.UNCERTAIN, "driver did not acknowledge"
    )
    module.preflight_rollout()
    module.start_rollout(dispatch_actions=True)
    wait_until(lambda: control.execute.call_count == 1, timeout=1)
    status = module.stop_rollout()
    assert "driver did not acknowledge" in status["last_error"]
    assert module.start_rollout()["active"] is False


def test_eef_chunk_uses_eef_task_and_cancellation(rollout: tuple[Any, Any, Any]) -> None:
    module, backend, control = rollout
    module.config.dreamlabs.mode = "eef"
    backend.predict.return_value = EefTrajectory(
        points=[
            EefTrajectoryPoint(0.0, {"tool": PoseStamped(frame_id="world")}),
            EefTrajectoryPoint(
                0.05, {"tool": PoseStamped(frame_id="world", position=[0.01, 0, 0])}
            ),
        ]
    )
    module.preflight_rollout()
    module.start_rollout(dispatch_actions=True)
    wait_until(lambda: control.eef_execute.call_count == 1, timeout=1)
    assert module.rollout_status()["chunks_completed"] == 0
    module.stop_rollout()
    control.execute.assert_not_called()
    control.eef_cancel.assert_called_once_with()
    assert module.rollout_status()["chunks_accepted"] == 1
    assert control.eef_execute.call_args.kwargs == {"deadman_timeout": 1.0}
    control.task_invoke.assert_any_call(
        "eef_trajectory",
        "execute",
        {"trajectory": backend.predict.return_value, "deadman_timeout": 1.0},
    )
    control.task_invoke.assert_any_call("eef_trajectory", "cancel", {})
    control.task_invoke.assert_any_call("eef_trajectory", "keepalive", {})


def test_completed_count_requires_coordinator_completion(rollout: tuple[Any, Any, Any]) -> None:
    module, backend, control = rollout
    module.preflight_rollout()
    module.start_rollout(dispatch_actions=True)
    wait_until(lambda: control.task_invoke.call_count >= 1, timeout=1)
    assert module.rollout_status()["chunks_completed"] == 0
    control.status.return_value = TrajectoryStatus(state=TrajectoryState.COMPLETED)
    wait_until(lambda: module.rollout_status()["chunks_completed"] >= 1, timeout=1)
    module.stop_rollout()
    assert module.rollout_status()["chunks_completed"] >= 1


def test_streaming_backend_collects_distinct_history_before_prediction(
    rollout: tuple[Any, Any, Any],
) -> None:
    module, backend, control = rollout
    backend.observation_period = 0.002
    backend.observations_required = 2
    observed = Event()
    backend.observe.side_effect = lambda *args, **kwargs: observed.set()
    module.preflight_rollout()
    module.start_rollout()
    assert observed.wait(timeout=1)
    backend.predict.assert_not_called()
    provide_observation(module)
    wait_until(lambda: module.rollout_status()["chunks_received"] >= 1, timeout=1)
    module.stop_rollout()
    assert backend.observe.call_count >= 2


def test_task_change_stops_and_remains_idle(rollout: tuple[Any, Any, Any]) -> None:
    module, backend, control = rollout
    module.preflight_rollout()
    module.start_rollout()
    wait_until(lambda: module.rollout_status()["chunks_received"] >= 1, timeout=1)
    status = module.set_task("new task")
    assert status["task"] == "new task"
    assert status["active"] is False
    assert status["busy"] is False


def test_stale_or_skewed_inputs_cannot_preflight(rollout: tuple[Any, Any, Any]) -> None:
    module, backend, control = rollout
    provide_observation(module, ts=time.time() - 2)
    assert module.preflight_rollout()["policy_ready"] is False
    backend.predict.assert_not_called()


def test_manual_takeover_stops_an_accepted_chunk(rollout: tuple[Any, Any, Any]) -> None:
    module, backend, control = rollout
    module.preflight_rollout()
    module.start_rollout(dispatch_actions=True)
    wait_until(lambda: control.execute.call_count == 1, timeout=1)
    buttons = Buttons()
    buttons.left_grip = True
    module._on_teleop_buttons(buttons)
    wait_until(lambda: not module.rollout_status()["busy"], timeout=1)
    assert module.rollout_status()["active"] is False
    control.cancel.assert_called_once_with()


def test_dreamlabs_factory_loads_without_sdk_or_credentials(rollout, mocker):
    module, backend, control = rollout
    module.config.joint_names = list(DUAL_OPENYAM_JOINTS)
    module.config.robot_type = "dual_openyam"
    mocker.patch("dimos.imitation.policy.dreamlabs.backend.DreamlabsKinematics")
    create_client = mocker.patch("dimos.imitation.policy.dreamlabs.backend.create_client")
    loaded = _PolicyRuntime._load_policy(module)
    try:
        assert isinstance(loaded, DreamlabsBackend)
        create_client.assert_not_called()
    finally:
        loaded.close()


def test_failed_preflight_invalidates_an_earlier_success(rollout):
    module, backend, control = rollout
    assert module.preflight_rollout()["policy_ready"] is True
    control.list_tasks.return_value = []
    assert module.preflight_rollout()["policy_ready"] is False
    assert module.start_rollout()["active"] is False
    backend.close.assert_called_once_with()
    backend.predict.assert_not_called()


def test_reset_failure_requires_new_preflight(rollout):
    module, backend, control = rollout
    backend.reset.side_effect = [None, RuntimeError("session reset refused")]
    module.preflight_rollout()
    module.start_rollout()
    wait_until(lambda: module.rollout_status()["chunks_received"] >= 1, timeout=1)
    status = module.stop_rollout()
    assert status["policy_ready"] is False
    assert "session reset refused" in status["last_error"]
    backend.close.assert_called_once_with()
    assert module.start_rollout()["active"] is False
