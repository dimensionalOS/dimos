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

"""Behavior tests for the isolated LeRobot runtime."""

from collections.abc import Callable, Iterator
from threading import Event, Thread
import time
from typing import Any, Protocol

from dimos_lerobot import runtime as policy_runtime
from dimos_lerobot.runtime import LeRobotPolicyRuntime
from lerobot.configs.policies import PreTrainedConfig
import numpy as np
from numpy.typing import NDArray
import pytest
import pytest_mock
import torch
from torch import Tensor

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.teleop.quest.quest_types import Buttons
from dimos.utils.testing.waiting import wait_until

JOINTS = [f"test_arm/joint{i}" for i in range(1, 5)]


class FakeFeature:
    def __init__(self, shape: tuple[int, ...]) -> None:
        self.shape = shape


class FakeUpstreamConfig:
    def __init__(self, joint_count: int, n_action_steps: int) -> None:
        self.type = "fake_policy"
        self.device: str | None = "cpu"
        self.use_amp = False
        self.chunk_size = 3
        self.n_action_steps: int | None = n_action_steps
        self.temporal_ensemble_coeff: float | None = None
        self.input_features = {
            "observation.images.wrist": FakeFeature((3, 4, 5)),
            "observation.state": FakeFeature((joint_count,)),
        }
        self.output_features = {"action": FakeFeature((joint_count,))}


class FakePipeline:
    def __init__(self, steps: list[object] | None = None) -> None:
        self.calls: list[object] = []
        self.reset_count = 0
        self.steps = steps or []

    def __call__(self, value: object) -> object:
        self.calls.append(value)
        return value

    def reset(self) -> None:
        self.reset_count += 1


class FakeActionStats:
    def __init__(self, lower: list[float], upper: list[float]) -> None:
        self._state = {
            "action.min": torch.tensor(lower),
            "action.max": torch.tensor(upper),
        }

    def state_dict(self) -> dict[str, Tensor]:
        return self._state


class FakePolicy:
    def __init__(
        self,
        action_chunk: NDArray[np.float32],
        n_action_steps: int = 2,
        *,
        action_lower: list[float] | None = None,
        action_upper: list[float] | None = None,
    ) -> None:
        self.action_chunk = torch.from_numpy(action_chunk).unsqueeze(0)
        self.called = Event()
        self.reset_count = 0
        self.batch: dict[str, object] | None = None
        self.upstream_config = FakeUpstreamConfig(len(JOINTS), n_action_steps)
        self.preprocessor = FakePipeline()
        self.postprocessor = FakePipeline(
            [
                FakeActionStats(
                    action_lower or [-100.0] * len(JOINTS),
                    action_upper or [100.0] * len(JOINTS),
                )
            ]
        )
        self.config_load_count = 0

    def reset(self) -> None:
        self.reset_count += 1

    def predict_action_chunk(self, batch: dict[str, object]) -> Tensor:
        self.batch = dict(batch)
        self.called.set()
        return self.action_chunk


class RuntimeFactory(Protocol):
    def __call__(
        self,
        policy: FakePolicy,
        *,
        device: str | None = None,
    ) -> tuple[LeRobotPolicyRuntime, Any]: ...


@pytest.fixture
def make_runtime(mocker: pytest_mock.MockerFixture) -> Iterator[RuntimeFactory]:
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    mocker.patch.object(LCMRPC, "__init__", return_value=None)
    mocker.patch.object(LCMRPC, "serve_module_rpc", return_value=None)
    mocker.patch.object(LCMRPC, "start", return_value=None)
    mocker.patch.object(LCMRPC, "stop", return_value=None)
    built: list[LeRobotPolicyRuntime] = []

    def _make(
        policy: FakePolicy,
        *,
        device: str | None = None,
    ) -> tuple[LeRobotPolicyRuntime, Any]:
        def load_config(_path: str) -> FakeUpstreamConfig:
            policy.config_load_count += 1
            return policy.upstream_config

        policy_class = mocker.MagicMock()
        policy_class.from_pretrained.return_value = policy
        mocker.patch.object(PreTrainedConfig, "from_pretrained", side_effect=load_config)
        mocker.patch.object(policy_runtime, "get_policy_class", return_value=policy_class)
        mocker.patch.object(
            policy_runtime,
            "make_pre_post_processors",
            return_value=(policy.preprocessor, policy.postprocessor),
        )

        def prepare_observation(
            observation: dict[str, NDArray[np.uint8] | NDArray[np.float32]],
            _device: torch.device,
            *,
            task: str,
            robot_type: str,
        ) -> dict[str, object]:
            return {
                **{
                    name: torch.from_numpy(value).unsqueeze(0)
                    for name, value in observation.items()
                },
                "task": task,
                "robot_type": robot_type,
            }

        mocker.patch.object(
            policy_runtime,
            "prepare_observation_for_inference",
            side_effect=prepare_observation,
        )
        mocker.patch.object(policy_runtime, "register_third_party_plugins")

        module = LeRobotPolicyRuntime(
            _isolated_python_runtime=True,
            policy_path="checkpoint/default",
            task="pick up the test object",
            device=device,
            joint_names=JOINTS,
            fps=50.0,
            robot_type="test_arm",
        )
        control = mocker.MagicMock()
        control.execute_trajectory.return_value = TrajectoryExecutionResult(
            TrajectoryExecutionStatus.ACCEPTED
        )
        control.cancel_trajectory.return_value = TrajectoryCancellationResult(
            TrajectoryCancellationStatus.ALREADY_STOPPED
        )
        mocker.patch.object(module, "_control", control, create=True)
        built.append(module)
        return module, control

    yield _make
    for module in built:
        module.stop()


def _action_chunk(steps: int = 3) -> NDArray[np.float32]:
    return np.arange(steps * len(JOINTS), dtype=np.float32).reshape(
        steps, len(JOINTS)
    ) / np.float32(10)


def _provide_observation(
    module: LeRobotPolicyRuntime,
    *,
    positions: list[float] | None = None,
    ts: float | None = None,
) -> tuple[NDArray[np.uint8], list[float], float]:
    bgr = np.zeros((4, 5, 3), dtype=np.uint8)
    bgr[..., 0] = 10
    bgr[..., 1] = 20
    bgr[..., 2] = 30
    values = positions or [float(i) / 10 for i in range(len(JOINTS))]
    timestamp = time.time() if ts is None else ts
    module._on_color_image(Image(data=bgr, format=ImageFormat.BGR, ts=timestamp))
    module._on_joint_state(JointState(ts=timestamp, name=JOINTS, position=values))
    return bgr, values, timestamp


def test_policy_predicts_and_executes_one_native_joint_chunk(make_runtime: RuntimeFactory) -> None:
    actions = _action_chunk()
    policy = FakePolicy(actions, n_action_steps=2)
    module, control = make_runtime(policy)
    bgr, positions, _ts = _provide_observation(module)

    assert module.start_rollout()["active"] is True
    wait_until(lambda: control.execute_trajectory.call_count >= 1, timeout=1.0)
    module.stop_rollout()

    call = control.execute_trajectory.call_args_list[0]
    trajectory = call.args[0]
    assert call.kwargs == {"task_name": "policy_rollout"}
    assert trajectory.joint_names == JOINTS
    assert [point.time_from_start for point in trajectory.points] == [0.0, 0.02, 0.04]
    np.testing.assert_allclose(trajectory.points[0].positions, positions)
    np.testing.assert_allclose(trajectory.points[1].positions, actions[0])
    np.testing.assert_allclose(trajectory.points[2].positions, actions[1])
    assert policy.batch is not None
    assert policy.batch["task"] == "pick up the test object"
    image = policy.batch["observation.images.wrist"]
    assert isinstance(image, Tensor)
    np.testing.assert_array_equal(image.squeeze(0).numpy(), bgr[..., ::-1])
    assert policy.postprocessor.calls
    assert module.rollout_status()["chunks_accepted"] >= 1


def test_policy_actions_are_clipped_to_checkpoint_range(make_runtime: RuntimeFactory) -> None:
    actions = np.zeros((3, len(JOINTS)), dtype=np.float32)
    actions[:, -1] = 1.016
    policy = FakePolicy(
        actions,
        n_action_steps=2,
        action_lower=[-10.0, -10.0, -10.0, 0.0],
        action_upper=[10.0, 10.0, 10.0, 1.0],
    )
    module, control = make_runtime(policy)
    _provide_observation(module)

    module.start_rollout()
    wait_until(lambda: control.execute_trajectory.call_count >= 1, timeout=1.0)
    module.stop_rollout()

    trajectory = control.execute_trajectory.call_args_list[0].args[0]
    assert [point.positions[-1] for point in trajectory.points[1:]] == [1.0, 1.0]


def test_next_chunk_uses_latest_joint_observation(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(_action_chunk(), n_action_steps=1)
    module, control = make_runtime(policy)
    first_submitted = Event()
    release_first = Event()

    def execute_trajectory(*_args: object, **_kwargs: object) -> TrajectoryExecutionResult:
        if not first_submitted.is_set():
            first_submitted.set()
            assert release_first.wait(timeout=1.0)
        return TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)

    control.execute_trajectory.side_effect = execute_trajectory
    _provide_observation(module, positions=[0.0] * len(JOINTS))
    module.start_rollout()
    assert first_submitted.wait(timeout=1.0)

    latest = [0.4, 0.3, 0.2, 0.1]
    _provide_observation(module, positions=latest)
    release_first.set()
    wait_until(lambda: control.execute_trajectory.call_count >= 2, timeout=1.0)
    module.stop_rollout()

    second = control.execute_trajectory.call_args_list[1].args[0]
    np.testing.assert_allclose(second.points[0].positions, latest)


def test_start_mismatch_waits_for_new_joint_state_before_retry(
    make_runtime: RuntimeFactory,
) -> None:
    policy = FakePolicy(_action_chunk(), n_action_steps=1)
    module, control = make_runtime(policy)
    _provide_observation(module)

    def execute_trajectory(*_args: object, **_kwargs: object) -> TrajectoryExecutionResult:
        status = (
            TrajectoryExecutionStatus.START_STATE_MISMATCH
            if control.execute_trajectory.call_count == 1
            else TrajectoryExecutionStatus.ACCEPTED
        )
        return TrajectoryExecutionResult(status)

    control.execute_trajectory.side_effect = execute_trajectory
    module.start_rollout()
    wait_until(lambda: control.execute_trajectory.call_count == 1, timeout=1.0)

    _provide_observation(module, positions=[0.2] * len(JOINTS), ts=time.time() + 0.01)
    wait_until(lambda: control.execute_trajectory.call_count >= 2, timeout=1.0)
    module.stop_rollout()

    assert module.rollout_status()["last_error"] is None


def test_a_press_stops_worker_before_cancelling_its_trajectory(
    make_runtime: RuntimeFactory,
) -> None:
    policy = FakePolicy(_action_chunk(), n_action_steps=2)
    module, control = make_runtime(policy)
    _provide_observation(module)
    execute_started = Event()
    release_execute = Event()
    stop_finished = Event()

    def execute_trajectory(*_args: object, **_kwargs: object) -> TrajectoryExecutionResult:
        execute_started.set()
        assert release_execute.wait(timeout=1.0)
        return TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)

    control.execute_trajectory.side_effect = execute_trajectory
    pressed = Buttons()
    pressed.right_primary = True

    def stop_from_button() -> None:
        module._on_button_pressed(pressed)
        stop_finished.set()

    module._on_button_pressed(pressed)
    assert execute_started.wait(timeout=1.0)
    stop_thread = Thread(target=stop_from_button)
    stop_thread.start()
    try:
        assert module._stop_event.wait(timeout=1.0)
        assert module.rollout_status()["active"] is True
        control.cancel_trajectory.assert_not_called()
    finally:
        release_execute.set()
        stop_thread.join(timeout=1.0)

    assert stop_finished.is_set()
    assert control.execute_trajectory.call_count == 1
    control.cancel_trajectory.assert_called_with(task_name="policy_rollout")


def test_uncertain_cancellation_is_reported(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(_action_chunk(), n_action_steps=1)
    module, control = make_runtime(policy)
    _provide_observation(module)
    control.cancel_trajectory.return_value = TrajectoryCancellationResult(
        TrajectoryCancellationStatus.UNCERTAIN,
        "coordinator did not confirm cancellation",
    )

    module.start_rollout()
    wait_until(lambda: control.execute_trajectory.call_count >= 1, timeout=1.0)
    status = module.stop_rollout()

    assert status["active"] is False
    assert status["last_error"] == "coordinator did not confirm cancellation"


@pytest.mark.parametrize(
    ("actions", "message"),
    [
        (np.zeros((3, len(JOINTS) - 1), dtype=np.float32), "action width"),
        (np.full((3, len(JOINTS)), np.nan, dtype=np.float32), "non-finite joint targets"),
    ],
)
def test_invalid_action_chunk_cancels_and_latches_rollout_off(
    make_runtime: RuntimeFactory,
    actions: NDArray[np.float32],
    message: str,
) -> None:
    policy = FakePolicy(actions)
    module, control = make_runtime(policy)
    _provide_observation(module)

    module.start_rollout()

    wait_until(lambda: module.rollout_status()["active"] is False, timeout=1.0)
    assert message in (module.rollout_status()["last_error"] or "")
    control.cancel_trajectory.assert_called_with(task_name="policy_rollout")


def test_trajectory_rejection_cancels_and_latches_rollout_off(
    make_runtime: RuntimeFactory,
) -> None:
    policy = FakePolicy(_action_chunk())
    module, control = make_runtime(policy)
    _provide_observation(module)
    control.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.POSITION_LIMIT_VIOLATION,
        "outside hardware limits",
    )

    module.start_rollout()

    wait_until(lambda: module.rollout_status()["active"] is False, timeout=1.0)
    assert module.rollout_status()["last_error"] == "outside hardware limits"
    control.cancel_trajectory.assert_called_with(task_name="policy_rollout")


@pytest.mark.parametrize(
    ("configure", "message"),
    [
        (lambda config: setattr(config, "n_action_steps", None), "positive int"),
        (lambda config: setattr(config, "n_action_steps", 0), "positive int"),
        (
            lambda config: setattr(config, "temporal_ensemble_coeff", 0.01),
            "temporal ensembling",
        ),
    ],
)
def test_incompatible_chunk_contract_is_rejected(
    make_runtime: RuntimeFactory,
    configure: Callable[[FakeUpstreamConfig], None],
    message: str,
) -> None:
    policy = FakePolicy(_action_chunk())
    configure(policy.upstream_config)
    module, control = make_runtime(policy)
    _provide_observation(module)

    module.start_rollout()

    wait_until(lambda: module.rollout_status()["active"] is False, timeout=1.0)
    assert message in (module.rollout_status()["last_error"] or "")
    control.execute_trajectory.assert_not_called()


def test_policy_refuses_to_load_without_live_observations(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(_action_chunk())
    module, control = make_runtime(policy)

    result = module.start_rollout()

    assert result["active"] is False
    assert "no camera image" in (result["last_error"] or "")
    assert policy.config_load_count == 0
    control.execute_trajectory.assert_not_called()


def test_policy_refuses_stale_observations(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(_action_chunk())
    module, control = make_runtime(policy)
    _provide_observation(module, ts=time.time() - module.config.max_observation_age_s - 1.0)

    result = module.start_rollout()

    assert result["active"] is False
    assert "camera image is stale" in (result["last_error"] or "")
    assert policy.config_load_count == 0
    control.execute_trajectory.assert_not_called()


def test_checkpoint_loads_on_demand_and_is_cached(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(_action_chunk())
    module, control = make_runtime(policy)
    _provide_observation(module)

    module.start_rollout()
    wait_until(lambda: control.execute_trajectory.call_count >= 1, timeout=1.0)
    module.stop_rollout()
    control.execute_trajectory.reset_mock()
    module.start_rollout()
    wait_until(lambda: control.execute_trajectory.call_count >= 1, timeout=1.0)
    module.stop_rollout()

    assert policy.config_load_count == 1
