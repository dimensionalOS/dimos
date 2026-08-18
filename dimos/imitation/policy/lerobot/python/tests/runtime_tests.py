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
import numpy as np
from numpy.typing import NDArray
import pytest
import pytest_mock
import torch
from torch import Tensor

from dimos.imitation.policy.lerobot.module import LeRobotPolicyConfig
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
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
        self.input_features = {
            "observation.images.image": FakeFeature((3, 4, 5)),
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


class RuntimeFactory(Protocol):
    def __call__(
        self,
        policies: dict[str, FakePolicy],
        *,
        devices: dict[str, str] | None = None,
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
        policies: dict[str, FakePolicy],
        *,
        devices: dict[str, str] | None = None,
    ) -> tuple[LeRobotPolicyRuntime, CapturingOutput]:
        policy_configs = {
            name: LeRobotPolicyConfig(
                policy_path=f"checkpoint/{name}",
                task=f"task for {name}",
                device=(devices or {}).get(name),
            )
            for name in policies
        }

        def load_config(path: str) -> FakeUpstreamConfig:
            policy = policies[path.rsplit("/", maxsplit=1)[-1]]
            policy.config_load_count += 1
            return policy.upstream_config

        policy_class = mocker.MagicMock()
        policy_class.from_pretrained.side_effect = lambda path, *, config: policies[
            path.rsplit("/", maxsplit=1)[-1]
        ]
        mocker.patch.object(
            policy_runtime.PreTrainedConfig,
            "from_pretrained",
            side_effect=load_config,
        )
        mocker.patch.object(policy_runtime, "get_policy_class", return_value=policy_class)
        mocker.patch.object(
            policy_runtime,
            "make_pre_post_processors",
            side_effect=lambda *, pretrained_path, **_kwargs: (
                policies[pretrained_path.rsplit("/", maxsplit=1)[-1]].preprocessor,
                policies[pretrained_path.rsplit("/", maxsplit=1)[-1]].postprocessor,
            ),
        )

        def prepare_observation(
            observation: dict[str, np.ndarray],
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
            _python_native_runtime=True,
            policies=policy_configs,
            joint_names=JOINTS,
            fps=50.0,
            robot_type="test_arm",
        )
        output = CapturingOutput()
        mocker.patch.object(module, "joint_command", output)
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
    action = np.arange(len(JOINTS), dtype=np.float32) / 20
    policy = FakePolicy(action)
    module, output = make_runtime({"pick_up_cube": policy})
    bgr, positions = _provide_observation(module)

    result = module.execute_learned_policy("pick_up_cube", duration=1.0)

    assert "started" in result.lower()
    assert output.published.wait(1.0), "policy did not publish a command"
    assert policy.reset_count == 1
    assert policy.preprocessor.reset_count == 1
    assert policy.postprocessor.reset_count == 1
    assert policy.batch is not None
    assert policy.batch["task"] == "task for pick_up_cube"
    assert policy.batch["robot_type"] == "test_arm"
    image = policy.batch["observation.images.image"]
    state = policy.batch["observation.state"]
    assert isinstance(image, Tensor)
    assert isinstance(state, Tensor)
    np.testing.assert_array_equal(image.squeeze(0).numpy(), bgr[..., ::-1])
    np.testing.assert_allclose(state.squeeze(0).numpy(), positions)
    assert output.messages[0].name == JOINTS
    np.testing.assert_allclose(output.messages[0].position, action)


def test_policy_refuses_to_load_without_live_observations(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output = make_runtime({"default": policy})

    result = module.execute_learned_policy("default", duration=1.0)

    assert "no camera image" in result
    assert policy.config_load_count == 0
    assert output.messages == []
    assert module.policy_status()["running"] is False


def test_policy_refuses_stale_observations(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output = make_runtime({"default": policy})
    stale = time.time() - module.config.max_observation_age_s - 1.0
    module._on_color_image(
        Image(data=np.zeros((4, 5, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=stale)
    )
    module._on_joint_state(JointState(ts=stale, name=JOINTS, position=[0.0] * len(JOINTS)))

    result = module.execute_learned_policy("default", duration=1.0)

    assert "camera image is stale" in result
    assert policy.config_load_count == 0
    assert output.messages == []


def test_policy_refuses_incomplete_joint_state(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output = make_runtime({"default": policy})
    now = time.time()
    module._on_color_image(
        Image(data=np.zeros((4, 5, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=now)
    )
    module._on_joint_state(JointState(ts=now, name=JOINTS[:-1], position=[0.0] * 3))

    result = module.execute_learned_policy("default", duration=1.0)

    assert JOINTS[-1] in result
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
    module, output = make_runtime({"invalid": policy})
    _provide_observation(module)

    module.execute_learned_policy("invalid", duration=1.0)

    assert policy.called.wait(1.0), "policy was not invoked"
    wait_until(lambda: module.policy_status()["running"] is False, timeout=1.0)
    assert output.messages == []
    assert message in (module.policy_status()["last_error"] or "")


def test_named_policies_load_on_demand_and_are_cached(make_runtime: RuntimeFactory) -> None:
    cup = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    plate = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, _output = make_runtime({"cup": cup, "plate": plate})
    _provide_observation(module)

    assert "started" in module.execute_learned_policy("cup", duration=1.0).lower()
    assert cup.called.wait(1.0)
    module.stop_learned_policy()
    assert "started" in module.execute_learned_policy("plate", duration=1.0).lower()
    assert plate.called.wait(1.0)
    module.stop_learned_policy()
    assert "started" in module.execute_learned_policy("cup", duration=1.0).lower()
    module.stop_learned_policy()

    assert cup.config_load_count == 1
    assert plate.config_load_count == 1
    assert cup.reset_count == 2
    assert module.policy_status()["available_policies"] == ["cup", "plate"]


def test_policy_rejects_incompatible_checkpoint_features(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    del policy.upstream_config.input_features["observation.images.image"]
    module, output = make_runtime({"invalid": policy})
    _provide_observation(module)

    result = module.execute_learned_policy("invalid", duration=1.0)

    assert "missing input features" in result
    assert output.messages == []


def test_policy_rejects_unavailable_cuda_device(
    make_runtime: RuntimeFactory,
    mocker: pytest_mock.MockerFixture,
) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output = make_runtime({"default": policy}, devices={"default": "cuda"})
    _provide_observation(module)
    mocker.patch.object(policy_runtime.torch.cuda, "is_available", return_value=False)

    result = module.execute_learned_policy("default", duration=1.0)

    assert "CUDA is not available" in result
    assert output.messages == []


def test_concurrent_policy_start_is_rejected(make_runtime: RuntimeFactory) -> None:
    cup = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    plate = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, _output = make_runtime({"cup": cup, "plate": plate})
    _provide_observation(module)
    assert "started" in module.execute_learned_policy("cup", duration=1.0).lower()
    assert cup.called.wait(1.0)

    result = module.execute_learned_policy("plate", duration=1.0)

    assert "already running" in result.lower()
    assert plate.config_load_count == 0
    assert module.stop_learned_policy() == "Learned policy stopped."


def test_policy_reports_duration_completion(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output = make_runtime({"default": policy})
    _provide_observation(module)

    result = module.execute_learned_policy("default", duration=0.05)

    assert "started" in result.lower()
    assert output.published.wait(1.0), "policy did not publish before its deadline"
    wait_until(lambda: module.policy_status()["running"] is False, timeout=1.0)


def test_unknown_policy_is_rejected_without_loading(make_runtime: RuntimeFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output = make_runtime({"cup": policy})
    _provide_observation(module)

    result = module.execute_learned_policy("missing")

    assert "unknown learned policy" in result.lower()
    assert policy.config_load_count == 0
    assert output.messages == []
