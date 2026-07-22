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

from collections.abc import Iterator
from threading import Event
import time
from typing import Any, Protocol

import numpy as np
import pytest
import pytest_mock

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.learning.lerobot_policy import LeRobotPolicyConfig, LeRobotPolicyModule
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.rpc.pubsubrpc import LCMRPC

JOINTS = [f"arm/joint{i}" for i in range(1, 7)] + ["arm/gripper"]


class CupPolicyModule(LeRobotPolicyModule):
    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    def pick_up_cup(self) -> str:
        """Pick up the wooden cup."""
        return self.start_configured_policy("pick_up_cup", tool_name="pick_up_cup")


class FakeBackend:
    def __init__(self, action: np.ndarray[Any, Any]) -> None:
        self.action = action
        self.called = Event()
        self.reset_count = 0
        self.image: np.ndarray[Any, Any] | None = None
        self.state: np.ndarray[Any, Any] | None = None
        self.task = ""
        self.robot_type = ""

    def reset(self) -> None:
        self.reset_count += 1

    def predict(
        self,
        image: np.ndarray[Any, Any],
        state: np.ndarray[Any, Any],
        *,
        task: str,
        robot_type: str,
    ) -> np.ndarray[Any, Any]:
        self.image = image
        self.state = state
        self.task = task
        self.robot_type = robot_type
        self.called.set()
        return self.action


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
        backends: dict[str, FakeBackend],
        *,
        module_type: type[LeRobotPolicyModule] = LeRobotPolicyModule,
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
        backends: dict[str, FakeBackend],
        *,
        module_type: type[LeRobotPolicyModule] = LeRobotPolicyModule,
    ) -> tuple[LeRobotPolicyModule, CapturingOutput, Event]:
        policies = {
            name: LeRobotPolicyConfig(
                policy_path=f"checkpoint/{name}",
                task=f"task for {name}",
            )
            for name in backends
        }

        def _load(_config: Any, policy: LeRobotPolicyConfig) -> FakeBackend:
            return backends[policy.policy_path.rsplit("/", maxsplit=1)[-1]]

        mocker.patch("dimos.learning.lerobot_policy._load_policy_backend", side_effect=_load)
        module = module_type(
            policies=policies,
            joint_names=JOINTS,
            fps=50.0,
            robot_type="galaxea_a1z",
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


def _provide_observation(module: LeRobotPolicyModule) -> tuple[np.ndarray[Any, Any], list[float]]:
    # BGR input verifies that the module supplies the RGB convention used by DataPrep.
    bgr = np.zeros((4, 5, 3), dtype=np.uint8)
    bgr[..., 0] = 10
    bgr[..., 1] = 20
    bgr[..., 2] = 30
    positions = [float(i) / 10 for i in range(len(JOINTS))]
    now = time.time()
    module._on_color_image(Image(data=bgr, format=ImageFormat.BGR, ts=now))
    module._on_joint_state(JointState(ts=now, name=JOINTS, position=positions))
    return bgr, positions


def test_policy_observation_and_action_use_canonical_order(
    make_module: ModuleFactory,
) -> None:
    action = np.arange(len(JOINTS), dtype=np.float32) / 20
    backend = FakeBackend(action)
    module, output, _finished = make_module({"pick_up_cube": backend})
    bgr, positions = _provide_observation(module)

    result = module.execute_learned_policy("pick_up_cube", duration=1.0)

    assert "started" in result.lower()
    assert output.published.wait(1.0), "policy did not publish a command"
    assert backend.reset_count == 1
    assert backend.task == "task for pick_up_cube"
    assert backend.robot_type == "galaxea_a1z"
    assert backend.image is not None
    np.testing.assert_array_equal(backend.image, bgr[..., ::-1])
    assert backend.state is not None
    np.testing.assert_allclose(backend.state, np.asarray(positions))
    assert output.messages[0].name == JOINTS
    np.testing.assert_allclose(output.messages[0].position, action)
    module.stop_learned_policy()


def test_invalid_policy_action_stops_without_publishing(
    make_module: ModuleFactory,
) -> None:
    backend = FakeBackend(np.zeros(len(JOINTS) - 1, dtype=np.float32))
    module, output, finished = make_module({"invalid": backend})
    _provide_observation(module)

    module.execute_learned_policy("invalid", duration=1.0)

    assert backend.called.wait(1.0), "policy was not invoked"
    assert finished.wait(1.0), "policy thread did not stop after invalid output"
    assert output.messages == []
    status = module.policy_status()
    assert status["running"] is False
    assert "expected (7,)" in status["last_error"]


def test_policy_refuses_to_start_without_live_observations(
    make_module: ModuleFactory,
) -> None:
    backend = FakeBackend(np.zeros(len(JOINTS), dtype=np.float32))
    module, output, _finished = make_module({"default": backend})

    result = module.execute_learned_policy("default", duration=1.0)

    assert "no camera image" in result
    assert backend.reset_count == 0
    assert output.messages == []
    assert module.policy_status()["running"] is False


def test_named_policies_load_lazily_and_are_cached(
    make_module: ModuleFactory,
    mocker: pytest_mock.MockerFixture,
) -> None:
    cup = FakeBackend(np.zeros(len(JOINTS), dtype=np.float32))
    plate = FakeBackend(np.zeros(len(JOINTS), dtype=np.float32))
    module, _output, _finished = make_module({"cup": cup, "plate": plate})
    loader = mocker.patch(
        "dimos.learning.lerobot_policy._load_policy_backend",
        side_effect=lambda _config, policy: {
            "checkpoint/cup": cup,
            "checkpoint/plate": plate,
        }[policy.policy_path],
    )
    _provide_observation(module)

    assert loader.call_count == 0
    assert "started" in module.execute_learned_policy("cup", duration=1.0).lower()
    assert cup.called.wait(1.0)
    module.stop_learned_policy()
    assert "started" in module.execute_learned_policy("plate", duration=1.0).lower()
    assert plate.called.wait(1.0)
    module.stop_learned_policy()
    assert "started" in module.execute_learned_policy("cup", duration=1.0).lower()
    module.stop_learned_policy()

    assert loader.call_count == 2
    assert module.policy_status()["available_policies"] == ["cup", "plate"]
    assert module.policy_status()["active_policy"] == "cup"


def test_named_skill_uses_its_own_tool_lifecycle(
    make_module: ModuleFactory,
) -> None:
    backend = FakeBackend(np.zeros(len(JOINTS), dtype=np.float32))
    module, _output, _finished = make_module({"pick_up_cup": backend}, module_type=CupPolicyModule)
    assert isinstance(module, CupPolicyModule)
    _provide_observation(module)

    skill_names = {skill_info.func_name for skill_info in module.get_skills()}
    result = module.pick_up_cup()

    assert "pick_up_cup" in skill_names
    assert "started" in result.lower()
    assert backend.called.wait(1.0)
    module.stop_learned_policy()
    module.start_tool.assert_called_with("pick_up_cup")  # type: ignore[attr-defined]
    module.stop_tool.assert_any_call("pick_up_cup")  # type: ignore[attr-defined]


def test_unknown_policy_is_rejected_without_loading(
    make_module: ModuleFactory,
) -> None:
    backend = FakeBackend(np.zeros(len(JOINTS), dtype=np.float32))
    module, output, _finished = make_module({"cup": backend})
    _provide_observation(module)

    result = module.execute_learned_policy("missing")

    assert "unknown learned policy" in result.lower()
    assert backend.reset_count == 0
    assert output.messages == []
