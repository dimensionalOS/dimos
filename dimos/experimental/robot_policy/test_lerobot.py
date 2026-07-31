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

"""Unit tests for the live LeRobot policy module."""

from __future__ import annotations

import builtins
from collections.abc import Iterator
from importlib import util
from pathlib import Path
import sys
from threading import Event
import time
from types import ModuleType
from typing import Protocol

import numpy as np
from numpy.typing import NDArray
from pydantic import ValidationError
import pytest
import pytest_mock
import torch
from torch import Tensor

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.rpc.pubsubrpc import LCMRPC


class _ImportPreTrainedConfig:
    @classmethod
    def from_pretrained(cls, _path: str) -> _ImportPreTrainedConfig:
        raise AssertionError("test must patch checkpoint loading")


class _ImportPreTrainedPolicy:
    pass


class _ImportPolicyProcessorPipeline:
    pass


def _install_import_fakes() -> dict[str, ModuleType | None]:
    """Install the minimum LeRobot module tree needed to import the optional runtime."""
    modules = {
        name: ModuleType(name)
        for name in (
            "lerobot",
            "lerobot.configs",
            "lerobot.configs.policies",
            "lerobot.policies",
            "lerobot.policies.factory",
            "lerobot.policies.pretrained",
            "lerobot.policies.utils",
            "lerobot.processor",
            "lerobot.utils",
            "lerobot.utils.import_utils",
        )
    }
    modules["lerobot.configs.policies"].PreTrainedConfig = _ImportPreTrainedConfig
    modules["lerobot.policies.factory"].get_policy_class = lambda _name: None
    modules["lerobot.policies.factory"].make_pre_post_processors = lambda **_kwargs: None
    modules["lerobot.policies.pretrained"].PreTrainedPolicy = _ImportPreTrainedPolicy
    modules["lerobot.policies.utils"].prepare_observation_for_inference = lambda *args: args
    modules["lerobot.processor"].PolicyProcessorPipeline = _ImportPolicyProcessorPipeline
    modules["lerobot.utils.import_utils"].register_third_party_plugins = lambda: None

    previous = {name: sys.modules.get(name) for name in modules}
    sys.modules.update(modules)
    return previous


def _restore_import_modules(previous: dict[str, ModuleType | None]) -> None:
    for name, module in previous.items():
        if module is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = module


# This optional module intentionally raises when LeRobot is absent. Match the
# RoboPlan tests by faking the dependency only around the top-level import.
_previous_modules = _install_import_fakes()
try:
    from dimos.experimental.robot_policy import lerobot as policy_runtime
    from dimos.experimental.robot_policy.lerobot import (
        LeRobotPolicyConfig,
        LeRobotPolicyModule,
        LeRobotPolicyModuleConfig,
    )
finally:
    _restore_import_modules(_previous_modules)


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


class CupPolicyModule(LeRobotPolicyModule):
    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def pick_up_cup(self) -> str:
        """Pick up the wooden cup."""
        return self.start_configured_policy("pick_up_cup", tool_name="pick_up_cup")


class CapturingOutput:
    def __init__(self) -> None:
        self.messages: list[JointState] = []
        self.published = Event()

    def publish(self, message: JointState) -> None:
        self.messages.append(message)
        self.published.set()


class ModuleFactory(Protocol):
    def __call__(
        self,
        policies: dict[str, FakePolicy],
        *,
        module_type: type[LeRobotPolicyModule] = LeRobotPolicyModule,
        devices: dict[str, str] | None = None,
    ) -> tuple[LeRobotPolicyModule, CapturingOutput, Event]: ...


@pytest.fixture
def make_module(
    mocker: pytest_mock.MockerFixture,
) -> Iterator[ModuleFactory]:
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    mocker.patch.object(LCMRPC, "__init__", return_value=None)
    mocker.patch.object(LCMRPC, "serve_module_rpc", return_value=None)
    mocker.patch.object(LCMRPC, "start", return_value=None)
    mocker.patch.object(LCMRPC, "stop", return_value=None)

    built: list[LeRobotPolicyModule] = []

    def _make(
        policies: dict[str, FakePolicy],
        *,
        module_type: type[LeRobotPolicyModule] = LeRobotPolicyModule,
        devices: dict[str, str] | None = None,
    ) -> tuple[LeRobotPolicyModule, CapturingOutput, Event]:
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

        module = module_type(
            policies=policy_configs,
            joint_names=JOINTS,
            fps=50.0,
            robot_type="test_arm",
        )
        output = CapturingOutput()
        finished = Event()
        module.joint_command = output  # type: ignore[assignment]
        mocker.patch.object(module, "start_tool")
        mocker.patch.object(module, "tool_update")
        mocker.patch.object(module, "stop_tool", side_effect=lambda _name: finished.set())
        module.build()
        built.append(module)
        return module, output, finished

    yield _make
    for module in built:
        module.stop()


def _provide_observation(module: LeRobotPolicyModule) -> tuple[NDArray[np.uint8], list[float]]:
    bgr = np.zeros((4, 5, 3), dtype=np.uint8)
    bgr[..., 0] = 10
    bgr[..., 1] = 20
    bgr[..., 2] = 30
    positions = [float(i) / 10 for i in range(len(JOINTS))]
    now = time.time()
    module._on_color_image(Image(data=bgr, format=ImageFormat.BGR, ts=now))
    module._on_joint_state(JointState(ts=now, name=JOINTS, position=positions))
    return bgr, positions


def test_policy_uses_direct_lerobot_inference_pipeline(make_module: ModuleFactory) -> None:
    action = np.arange(len(JOINTS), dtype=np.float32) / 20
    policy = FakePolicy(action)
    module, output, _finished = make_module({"pick_up_cube": policy})
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
    assert policy.preprocessor.calls
    assert len(policy.postprocessor.calls) == 1
    assert policy.postprocessor.calls[0] is policy.action
    assert output.messages[0].name == JOINTS
    np.testing.assert_allclose(output.messages[0].position, action)
    module.stop_learned_policy()


def test_invalid_policy_action_stops_without_publishing(make_module: ModuleFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS) - 1, dtype=np.float32))
    module, output, finished = make_module({"invalid": policy})
    _provide_observation(module)

    module.execute_learned_policy("invalid", duration=1.0)

    assert policy.called.wait(1.0), "policy was not invoked"
    assert finished.wait(1.0), "policy thread did not stop after invalid output"
    assert output.messages == []
    module.stop_learned_policy()
    status = module.policy_status()
    assert status["running"] is False
    assert f"expected ({len(JOINTS)},)" in (status["last_error"] or "")


def test_policy_refuses_to_start_without_live_observations(make_module: ModuleFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output, _finished = make_module({"default": policy})

    result = module.execute_learned_policy("default", duration=1.0)

    assert "no camera image" in result
    assert policy.config_load_count == 0
    assert output.messages == []
    assert module.policy_status()["running"] is False


def test_policy_refuses_stale_observations_without_loading(make_module: ModuleFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output, _finished = make_module({"default": policy})
    stale = time.time() - module.config.max_observation_age_s - 1.0
    module._on_color_image(
        Image(data=np.zeros((4, 5, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=stale)
    )
    module._on_joint_state(JointState(ts=stale, name=JOINTS, position=[0.0] * len(JOINTS)))

    result = module.execute_learned_policy("default", duration=1.0)

    assert "camera image is stale" in result
    assert policy.config_load_count == 0
    assert output.messages == []


def test_policy_refuses_incomplete_joint_state_without_loading(make_module: ModuleFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output, _finished = make_module({"default": policy})
    now = time.time()
    module._on_color_image(
        Image(data=np.zeros((4, 5, 3), dtype=np.uint8), format=ImageFormat.RGB, ts=now)
    )
    module._on_joint_state(JointState(ts=now, name=JOINTS[:-1], position=[0.0] * 3))

    result = module.execute_learned_policy("default", duration=1.0)

    assert JOINTS[-1] in result
    assert policy.config_load_count == 0
    assert output.messages == []


def test_policy_stops_when_action_contains_non_finite_value(make_module: ModuleFactory) -> None:
    action = np.zeros(len(JOINTS), dtype=np.float32)
    action[-1] = np.nan
    policy = FakePolicy(action)
    module, output, finished = make_module({"invalid": policy})
    _provide_observation(module)

    module.execute_learned_policy("invalid", duration=1.0)

    assert finished.wait(1.0), "policy thread did not stop after invalid output"
    assert output.messages == []
    assert module.policy_status()["last_error"] == "policy returned non-finite joint targets"


def test_named_policies_load_on_demand_and_are_cached(make_module: ModuleFactory) -> None:
    cup = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    plate = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, _output, _finished = make_module({"cup": cup, "plate": plate})
    _provide_observation(module)

    assert cup.config_load_count == 0
    assert plate.config_load_count == 0
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
    assert module.policy_status()["active_policy"] == "cup"


def test_policy_rejects_incompatible_checkpoint_features(make_module: ModuleFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    del policy.upstream_config.input_features["observation.images.image"]
    module, output, _finished = make_module({"invalid": policy})
    _provide_observation(module)

    result = module.execute_learned_policy("invalid", duration=1.0)

    assert "missing input features" in result
    assert output.messages == []


def test_policy_rejects_unavailable_cuda_device(
    make_module: ModuleFactory,
    mocker: pytest_mock.MockerFixture,
) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output, _finished = make_module({"default": policy}, devices={"default": "cuda"})
    _provide_observation(module)
    mocker.patch.object(policy_runtime.torch.cuda, "is_available", return_value=False)

    result = module.execute_learned_policy("default", duration=1.0)

    assert "CUDA is not available" in result
    assert output.messages == []


def test_concurrent_policy_start_is_rejected(make_module: ModuleFactory) -> None:
    cup = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    plate = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, _output, _finished = make_module({"cup": cup, "plate": plate})
    _provide_observation(module)
    assert "started" in module.execute_learned_policy("cup", duration=1.0).lower()
    assert cup.called.wait(1.0)

    result = module.execute_learned_policy("plate", duration=1.0)

    assert "already running" in result.lower()
    assert plate.config_load_count == 0
    assert module.stop_learned_policy() == "Learned policy stopped."


def test_policy_reports_duration_completion(make_module: ModuleFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output, finished = make_module({"default": policy})
    _provide_observation(module)

    result = module.execute_learned_policy("default", duration=0.05)

    assert "started" in result.lower()
    assert output.published.wait(1.0), "policy did not publish before its deadline"
    assert finished.wait(1.0), "policy did not finish after its duration"
    assert module.policy_status()["running"] is False


def test_named_skill_uses_its_own_tool_lifecycle(make_module: ModuleFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, _output, _finished = make_module({"pick_up_cup": policy}, module_type=CupPolicyModule)
    assert isinstance(module, CupPolicyModule)
    _provide_observation(module)

    skill_names = {skill_info.func_name for skill_info in module.get_skills()}
    result = module.pick_up_cup()

    assert "pick_up_cup" in skill_names
    assert "started" in result.lower()
    assert policy.called.wait(1.0)
    module.stop_learned_policy()
    module.start_tool.assert_called_with("pick_up_cup")  # type: ignore[attr-defined]
    module.stop_tool.assert_any_call("pick_up_cup")  # type: ignore[attr-defined]


def test_unknown_policy_is_rejected_without_loading(make_module: ModuleFactory) -> None:
    policy = FakePolicy(np.zeros(len(JOINTS), dtype=np.float32))
    module, output, _finished = make_module({"cup": policy})
    _provide_observation(module)

    result = module.execute_learned_policy("missing")

    assert "unknown learned policy" in result.lower()
    assert policy.config_load_count == 0
    assert output.messages == []


@pytest.mark.parametrize(
    "config_kwargs, message",
    [
        (
            {
                "policies": {"default": LeRobotPolicyConfig(policy_path="checkpoint/default")},
                "joint_names": ["joint1", "joint1"],
            },
            "joint_names must not contain duplicates",
        ),
        (
            {
                "policies": {" ": LeRobotPolicyConfig(policy_path="checkpoint/default")},
                "joint_names": ["joint1"],
            },
            "policy names must not be empty",
        ),
    ],
)
def test_module_config_rejects_ambiguous_names(
    config_kwargs: dict[str, object], message: str
) -> None:
    with pytest.raises(ValidationError, match=message):
        LeRobotPolicyModuleConfig(**config_kwargs)


def test_module_import_reports_optional_dependency_install_commands(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime_path = Path(policy_runtime.__file__)
    spec = util.spec_from_file_location("_missing_lerobot_runtime", runtime_path)
    assert spec is not None
    assert spec.loader is not None
    original_import = builtins.__import__

    def reject_lerobot(
        name: str,
        globals_: dict[str, object] | None = None,
        locals_: dict[str, object] | None = None,
        fromlist: tuple[str, ...] = (),
        level: int = 0,
    ) -> ModuleType:
        if name == "lerobot" or name.startswith("lerobot."):
            raise ImportError("missing optional dependency")
        return original_import(name, globals_, locals_, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", reject_lerobot)

    with pytest.raises(ImportError, match=r"dimos\[lerobot\].*uv sync --extra lerobot"):
        spec.loader.exec_module(util.module_from_spec(spec))
