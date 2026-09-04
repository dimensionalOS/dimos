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

from collections.abc import Iterator, Mapping
from pathlib import Path
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
import pytest
import pytest_mock

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.imitation.dataprep.core import SyncConfig
from dimos.imitation.policy.backend import PolicyBackendInfo
from dimos.imitation.policy.module import PolicyRolloutConfig, declare_policy_module
from dimos.imitation.policy.runtime import declare_policy_runtime
from dimos.imitation.profile import (
    ImageSource,
    JointPositionAction,
    JointPositionSource,
    PolicyIOProfile,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.utils.testing.waiting import wait_until

JOINTS = ("left", "right")
PROFILE = PolicyIOProfile(
    name="runtime-test",
    robot_type="test",
    observations={
        "top": ImageSource(stream="top_image", shape=(4, 5, 3)),
        "left": ImageSource(stream="left_image", shape=(4, 5, 3)),
        "state": JointPositionSource(stream="joint_state", joints=JOINTS),
    },
    action=JointPositionAction(
        key="actions",
        demonstration=JointPositionSource(stream="joint_command", joints=JOINTS),
    ),
    sync=SyncConfig(anchor="top", rate_hz=30.0, tolerance_ms=20.0),
)


class _TestConfig(PolicyRolloutConfig):
    pass


PolicyDeclaration = declare_policy_module(
    "RuntimeTestPolicy",
    __name__,
    PROFILE,
    _TestConfig,
    "unused:TestRuntime",
)


class FakeBackend:
    actions = np.arange(60, dtype=np.float32).reshape(30, 2)

    def __init__(self, _config: _TestConfig) -> None:
        self.actions = type(self).actions.copy()
        self.info = PolicyBackendInfo(
            name="fake",
            chunk_length=30,
            preferred_execution_steps=20,
        )
        self.load_count = 0
        self.reset_count = 0
        self.reset_error: str | None = None

    def load(self, _profile: PolicyIOProfile) -> PolicyBackendInfo:
        self.load_count += 1
        return self.info

    def reset(self) -> None:
        self.reset_count += 1
        if self.reset_error is not None:
            raise RuntimeError(self.reset_error)

    def predict(
        self,
        _observations: Mapping[str, NDArray[Any]],
        _task: str,
    ) -> NDArray[np.float32]:
        return self.actions.copy()


RuntimeModule = declare_policy_runtime(
    "RuntimeTestModule",
    __name__,
    PolicyDeclaration,
    FakeBackend,
)


@pytest.fixture
def runtime(
    tmp_path: Path,
    mocker: pytest_mock.MockerFixture,
) -> Iterator[tuple[RuntimeModule, Any]]:
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    mocker.patch.object(LCMRPC, "__init__", return_value=None)
    mocker.patch.object(LCMRPC, "serve_module_rpc", return_value=None)
    mocker.patch.object(LCMRPC, "start", return_value=None)
    mocker.patch.object(LCMRPC, "stop", return_value=None)
    module = RuntimeModule(
        _isolated_python_runtime=True,
        artifact=str(tmp_path / "artifact"),
        task="test task",
    )
    control = mocker.MagicMock()
    control.list_tasks.return_value = ["policy_rollout"]
    control.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.ACCEPTED
    )
    control.cancel_trajectory.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.ALREADY_STOPPED
    )
    mocker.patch.object(module, "_control", control, create=True)
    yield module, control
    module.stop()


def _image(ts: float) -> Image:
    return Image(data=np.zeros((4, 5, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=ts)


def _provide(module: RuntimeModule, *, top_ts: float, other_ts: float) -> None:
    module._on_observation("top_image", _image(top_ts))
    module._on_observation("left_image", _image(other_ts))
    module._on_observation(
        "joint_state",
        JointState(ts=other_ts, name=list(reversed(JOINTS)), position=[2.0, 1.0]),
    )


def test_preflight_requires_every_camera_within_profile_tolerance(
    runtime: tuple[RuntimeModule, Any],
) -> None:
    module, _control = runtime
    now = time.time()
    _provide(module, top_ts=now, other_ts=now - 0.021)

    status = module.preflight_rollout()

    assert status["policy_ready"] is False
    assert status["observations_ready"] is False
    assert "21.0ms" in (status["last_error"] or "")


def test_rollout_caps_execution_horizon_and_preserves_profile_joint_order(
    runtime: tuple[RuntimeModule, Any],
) -> None:
    module, control = runtime
    now = time.time()
    _provide(module, top_ts=now, other_ts=now - 0.005)
    assert module.preflight_rollout()["policy_ready"] is True

    module.start_rollout()
    wait_until(lambda: control.execute_trajectory.call_count >= 1, timeout=1.0)
    module.stop_rollout()

    trajectory = control.execute_trajectory.call_args.args[0]
    assert trajectory.joint_names == list(JOINTS)
    assert len(trajectory.points) == 16
    assert trajectory.points[-1].time_from_start == pytest.approx(0.5)
    assert trajectory.points[0].positions == [1.0, 2.0]
    np.testing.assert_array_equal(trajectory.points[-1].positions, FakeBackend.actions[14])


def test_missing_declared_top_camera_fails_before_backend_load(
    runtime: tuple[RuntimeModule, Any],
) -> None:
    module, _control = runtime
    now = time.time()
    module._on_observation("left_image", _image(now))
    module._on_observation(
        "joint_state",
        JointState(ts=now, name=list(JOINTS), position=[1.0, 2.0]),
    )

    status = module.preflight_rollout()

    assert status["policy_ready"] is False
    assert status["last_error"] == "no 'top' observation has been received"
    assert module._backend.load_count == 0


def test_start_requires_successful_preflight(
    runtime: tuple[RuntimeModule, Any],
) -> None:
    module, control = runtime
    now = time.time()
    _provide(module, top_ts=now, other_ts=now)

    status = module.start_rollout()

    assert status["active"] is False
    assert status["last_error"] == "policy preflight has not passed"
    control.execute_trajectory.assert_not_called()


def test_preflight_requires_configured_coordinator_task_without_loading_backend(
    runtime: tuple[RuntimeModule, Any],
) -> None:
    module, control = runtime
    now = time.time()
    _provide(module, top_ts=now, other_ts=now)
    control.list_tasks.return_value = ["another_task"]

    status = module.preflight_rollout()

    assert status["policy_ready"] is False
    assert "missing configured rollout task" in (status["last_error"] or "")
    assert module._backend.load_count == 0
    control.execute_trajectory.assert_not_called()


def test_policy_actions_are_clipped_to_backend_bounds(
    runtime: tuple[RuntimeModule, Any],
) -> None:
    module, control = runtime
    module._backend.actions[:, 1] = 2.0
    module._backend.info = PolicyBackendInfo(
        name="fake",
        chunk_length=30,
        preferred_execution_steps=1,
        action_lower=np.asarray([-100.0, 0.0], dtype=np.float32),
        action_upper=np.asarray([100.0, 1.0], dtype=np.float32),
    )
    now = time.time()
    _provide(module, top_ts=now, other_ts=now)

    assert module.preflight_rollout()["policy_ready"] is True
    module.start_rollout()
    wait_until(lambda: control.execute_trajectory.call_count >= 1, timeout=1.0)
    module.stop_rollout()

    trajectory = control.execute_trajectory.call_args.args[0]
    assert trajectory.points[1].positions[1] == 1.0


@pytest.mark.parametrize(
    ("actions", "message"),
    [
        (np.zeros((30, 1), dtype=np.float32), "expected (steps, 2)"),
        (np.full((30, 2), np.nan, dtype=np.float32), "non-finite joint targets"),
    ],
)
def test_invalid_action_chunk_cancels_and_latches_rollout_off(
    runtime: tuple[RuntimeModule, Any],
    actions: NDArray[np.float32],
    message: str,
) -> None:
    module, control = runtime
    module._backend.actions = actions
    now = time.time()
    _provide(module, top_ts=now, other_ts=now)

    assert module.preflight_rollout()["policy_ready"] is True
    module.start_rollout()

    wait_until(lambda: module.rollout_status()["active"] is False, timeout=1.0)
    assert message in (module.rollout_status()["last_error"] or "")
    control.execute_trajectory.assert_not_called()
    control.cancel_trajectory.assert_called_with(task_name="policy_rollout")


def test_trajectory_rejection_cancels_and_latches_rollout_off(
    runtime: tuple[RuntimeModule, Any],
) -> None:
    module, control = runtime
    now = time.time()
    _provide(module, top_ts=now, other_ts=now)
    control.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.POSITION_LIMIT_VIOLATION,
        "outside hardware limits",
    )

    assert module.preflight_rollout()["policy_ready"] is True
    module.start_rollout()

    wait_until(lambda: module.rollout_status()["active"] is False, timeout=1.0)
    assert module.rollout_status()["last_error"] == "outside hardware limits"
    control.cancel_trajectory.assert_called_with(task_name="policy_rollout")


def test_backend_reset_failure_does_not_leave_rollout_active(
    runtime: tuple[RuntimeModule, Any],
) -> None:
    module, control = runtime
    now = time.time()
    _provide(module, top_ts=now, other_ts=now)
    assert module.preflight_rollout()["policy_ready"] is True
    module._backend.reset_error = "backend state is stuck"

    module.start_rollout()

    wait_until(lambda: module.rollout_status()["active"] is False, timeout=1.0)
    assert "backend state is stuck" in (module.rollout_status()["last_error"] or "")
    control.cancel_trajectory.assert_called_with(task_name="policy_rollout")
