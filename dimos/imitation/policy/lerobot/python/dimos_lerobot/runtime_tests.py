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

from collections.abc import Iterator
from threading import Event
import time
from typing import Protocol

from dimos_lerobot import runtime as policy_runtime
from dimos_lerobot.runtime import LeRobotPolicyRuntime
from lerobot.configs.policies import PreTrainedConfig
import numpy as np
from numpy.typing import NDArray
import pytest
import pytest_mock
import torch
from torch import Tensor

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.utils.testing.waiting import wait_until

JOINTS = [f"test_arm/joint{i}" for i in range(1, 5)]


class FakeFeature:
    def __init__(self, shape: tuple[int, ...]) -> None:
        self.shape = shape


class FakeUpstreamConfig:
    def __init__(self, joint_count: int) -> None:
        self.type = "fake_policy"
        self.device: str | None = "cpu"
        self.use_amp = False
        self.chunk_size = 100
        self.n_action_steps = 100
        self.input_features = {
            "observation.images.wrist": FakeFeature((3, 4, 5)),
            "observation.state": FakeFeature((joint_count,)),
        }
        self.output_features = {"action": FakeFeature((joint_count,))}


class FakePipeline:
    def __init__(self) -> None:
        self.calls: list[object] = []
        self.reset_count = 0

    def __call__(self, value: object) -> object:
        self.calls.append(value)
        return value

    def reset(self) -> None:
        self.reset_count += 1


class FakePolicy:
    def __init__(self, action: NDArray[np.float32]) -> None:
        self.action = torch.from_numpy(action).unsqueeze(0)
        self.called = Event()
        self.reset_count = 0
        self.batch: dict[str, object] | None = None
        self.upstream_config = FakeUpstreamConfig(len(JOINTS))
        self.preprocessor = FakePipeline()
        self.postprocessor = FakePipeline()
        self.config_load_count = 0

    def reset(self) -> None:
        self.reset_count += 1

    def select_action(self, batch: dict[str, object]) -> Tensor:
        self.batch = dict(batch)
        self.called.set()
        return self.action


class CapturingOutput:
    def __init__(self) -> None:
        self.messages: list[JointState] = []
        self.published = Event()

    def publish(self, message: JointState) -> None:
        self.messages.append(message)
        self.published.set()


class CapturingGripperOutput:
    def __init__(self) -> None:
        self.messages: list[Float32] = []

    def publish(self, message: Float32) -> None:
        self.messages.append(message)


class RuntimeFactory(Protocol):
    def __call__(
        self,
        policy: FakePolicy,
        *,
        device: str | None = None,
        gripper_joint_name: str | None = None,
    ) -> tuple[LeRobotPolicyRuntime, CapturingOutput]: ...


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
        gripper_joint_name: str | None = None,
    ) -> tuple[LeRobotPolicyRuntime, CapturingOutput]:
        def load_config(_path: str) -> FakeUpstreamConfig:
            policy.config_load_count += 1
            return policy.upstream_config

        policy_class = mocker.MagicMock()
        policy_class.from_pretrained.return_value = policy
        mocker.patch.object(
            PreTrainedConfig,
            "from_pretrained",
            side_effect=load_config,
        )
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
            gripper_joint_name=gripper_joint_name,
            fps=50.0,
            robot_type="test_arm",
        )
        output = CapturingOutput()
        mocker.patch.object(module, "joint_command", output)
        mocker.patch.object(module, "gripper_command", CapturingGripperOutput())
        built.append(module)
        return module, output

    yield _make
    for module in built:
        module.stop()


def _provide_observation(
    module: LeRobotPolicyRuntime,
) -> tuple[NDArray[np.uint8], list[float]]:
    bgr = np.zeros((4, 5, 3), dtype=np.uint8)
    bgr[..., 0] = 10
    bgr[..., 1] = 20
    bgr[..., 2] = 30
    positions = [float(i) / 10 for i in range(len(JOINTS))]
    now = time.time()
    module._on_color_image(Image(data=bgr, format=ImageFormat.BGR, ts=now))
    module._on_joint_state(JointState(ts=now, name=JOINTS, position=positions))
    return bgr, positions


def test_policy_uses_direct_lerobot_inference_pipeline(make_runtime: RuntimeFactory) -> None:
    action = (np.arange(len(JOINTS), dtype=np.float32) / 20).astype(np.float32)
    policy = FakePolicy(action)
    module, output = make_runtime(policy)
    bgr, positions = _provide_observation(module)

    result = module.start_rollout(duration=1.0)

    assert result["active"] is True
    assert output.published.wait(1.0), "policy did not publish a command"
    assert policy.reset_count >= 1
    assert policy.preprocessor.reset_count >= 1
    assert policy.postprocessor.reset_count >= 1
    assert policy.batch is not None
    assert policy.batch["task"] == "pick up the test object"
    assert policy.batch["robot_type"] == "test_arm"
    image = policy.batch["observation.images.wrist"]
    state = policy.batch["observation.state"]
    assert isinstance(image, Tensor)
    assert isinstance(state, Tensor)
    np.testing.assert_array_equal(image.squeeze(0).numpy(), bgr[..., ::-1])
    np.testing.assert_allclose(state.squeeze(0).numpy(), positions)
    assert output.messages[0].name == JOINTS
    np.testing.assert_allclose(output.messages[0].position, action)


def test_policy_splits_gripper_without_changing_checkpoint_action_order(
    make_runtime: RuntimeFactory,
) -> None:
    action = np.asarray([0.1, 0.2, 0.3, 1.02], dtype=np.float32)
    policy = FakePolicy(action)
    module, arm_output = make_runtime(policy, gripper_joint_name=JOINTS[-1])
    _provide_observation(module)

    module.start_rollout(duration=1.0)

    assert arm_output.published.wait(1.0)
    gripper_output = module.gripper_command
    assert isinstance(gripper_output, CapturingGripperOutput)
    assert arm_output.messages[0].name == JOINTS[:-1]
    np.testing.assert_allclose(arm_output.messages[0].position, action[:-1])
    assert gripper_output.messages[0].data == pytest.approx(action[-1])


def test_policy_refuses_to_load_without_live_observations(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output = make_runtime(policy)

    result = module.start_rollout(duration=1.0)

    assert result["active"] is False
    assert "no camera image" in (result["last_error"] or "")
    assert policy.config_load_count == 0
    assert output.messages == []
    assert module.rollout_status()["active"] is False


def test_policy_refuses_stale_observations(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output = make_runtime(policy)
    stale = time.time() - module.config.max_observation_age_s - 1.0
    module._on_color_image(
        Image(data=np.zeros((4, 5, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=stale)
    )
    module._on_joint_state(JointState(ts=stale, name=JOINTS, position=[0.0] * len(JOINTS)))

    result = module.start_rollout(duration=1.0)

    assert "camera image is stale" in (result["last_error"] or "")
    assert policy.config_load_count == 0
    assert output.messages == []


def test_policy_refuses_incomplete_joint_state(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output = make_runtime(policy)
    now = time.time()
    module._on_color_image(
        Image(data=np.zeros((4, 5, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=now)
    )
    module._on_joint_state(JointState(ts=now, name=JOINTS[:-1], position=[0.0] * 3))

    result = module.start_rollout(duration=1.0)

    assert JOINTS[-1] in (result["last_error"] or "")
    assert policy.config_load_count == 0
    assert output.messages == []


@pytest.mark.parametrize(
    ("action", "message"),
    [
        (np.zeros(len(JOINTS) - 1, dtype=np.float32), f"expected ({len(JOINTS)},)"),
        (
            np.asarray([0.0, 0.0, 0.0, np.nan], dtype=np.float32),
            "policy returned non-finite joint targets",
        ),
    ],
)
def test_invalid_policy_action_stops_without_publishing(
    make_runtime: RuntimeFactory,
    action: NDArray[np.float32],
    message: str,
) -> None:
    policy = FakePolicy(action)
    module, output = make_runtime(policy)
    _provide_observation(module)

    module.start_rollout(duration=1.0)

    assert policy.called.wait(1.0), "policy was not invoked"
    wait_until(lambda: module.rollout_status()["active"] is False, timeout=1.0)
    assert output.messages == []
    assert message in (module.rollout_status()["last_error"] or "")


def test_checkpoint_loads_on_demand_and_is_cached(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, _output = make_runtime(policy)
    _provide_observation(module)

    assert module.start_rollout(duration=1.0)["active"] is True
    assert policy.called.wait(1.0)
    assert module.stop_rollout()["active"] is False
    policy.called.clear()
    assert module.start_rollout(duration=1.0)["active"] is True
    assert policy.called.wait(1.0)
    module.stop_rollout()

    assert policy.config_load_count == 1
    assert policy.reset_count >= 4


def test_policy_accepts_config_without_action_chunk_metadata(
    make_runtime: RuntimeFactory,
) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    del policy.upstream_config.chunk_size
    del policy.upstream_config.n_action_steps
    module, output = make_runtime(policy)
    _provide_observation(module)

    result = module.start_rollout(duration=1.0)

    assert result["active"] is True
    assert output.published.wait(1.0), "policy did not publish a command"
    assert module.rollout_status()["last_error"] is None


@pytest.mark.parametrize("attribute", ["chunk_size", "n_action_steps"])
def test_policy_rejects_non_integer_action_chunk_metadata(
    make_runtime: RuntimeFactory,
    attribute: str,
) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    setattr(policy.upstream_config, attribute, "invalid")
    module, output = make_runtime(policy)
    _provide_observation(module)

    module.start_rollout(duration=1.0)

    wait_until(lambda: module.rollout_status()["active"] is False, timeout=1.0)
    assert module.rollout_status()["last_error"] == f"{attribute} must be an int, got str"
    assert output.messages == []


def test_policy_rejects_incompatible_checkpoint_features(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    del policy.upstream_config.input_features["observation.images.wrist"]
    module, output = make_runtime(policy)
    _provide_observation(module)

    module.start_rollout(duration=1.0)

    wait_until(lambda: module.rollout_status()["active"] is False, timeout=1.0)
    assert "missing input features" in (module.rollout_status()["last_error"] or "")
    assert output.messages == []


def test_policy_rejects_unavailable_cuda_device(
    make_runtime: RuntimeFactory,
    mocker: pytest_mock.MockerFixture,
) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output = make_runtime(policy, device="cuda")
    _provide_observation(module)
    mocker.patch.object(torch.cuda, "is_available", return_value=False)

    result = module.start_rollout(duration=1.0)

    assert result["active"] is True
    wait_until(lambda: module.rollout_status()["active"] is False, timeout=1.0)
    assert "CUDA is not available" in (module.rollout_status()["last_error"] or "")
    assert output.messages == []


def test_concurrent_policy_start_is_rejected(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, _output = make_runtime(policy)
    _provide_observation(module)
    assert module.start_rollout(duration=1.0)["active"] is True
    assert policy.called.wait(1.0)

    result = module.start_rollout(duration=1.0)

    assert result["active"] is True
    assert "already active" in (result["last_error"] or "")
    assert module.stop_rollout()["active"] is False


def test_policy_reports_duration_completion(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output = make_runtime(policy)
    _provide_observation(module)

    result = module.start_rollout(duration=0.05)

    assert result["active"] is True
    assert output.published.wait(1.0), "policy did not publish before its deadline"
    wait_until(lambda: module.rollout_status()["active"] is False, timeout=1.0)


def test_rollout_without_duration_runs_until_stopped(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output = make_runtime(policy)
    _provide_observation(module)

    assert module.start_rollout()["active"] is True
    assert output.published.wait(1.0)

    assert module.rollout_status()["active"] is True
    assert module.stop_rollout()["active"] is False


def test_invalid_duration_is_rejected_without_loading(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output = make_runtime(policy)
    _provide_observation(module)

    result = module.start_rollout(duration=0.0)

    assert result["active"] is False
    assert result["last_error"] == "duration must be greater than zero"
    assert policy.config_load_count == 0
    assert output.messages == []
