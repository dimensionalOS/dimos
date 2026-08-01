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

"""Tests for the fake simulator runtime DimOS module path."""

from __future__ import annotations

from pathlib import Path
import sys
from unittest.mock import patch

from dimos.core.module import Module
from dimos.core.runtime_environment import PythonProjectRuntimeEnvironment
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.simulation.runtime_module import SimulatorOwnerThread, publish_output

REPO_ROOT = Path(__file__).resolve().parents[3]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
FAKE_RUNTIME_SRC = REPO_ROOT / "packages" / "dimos-fake-runtime-sidecar" / "src"
sys.path.insert(0, str(PROTOCOL_SRC))
sys.path.insert(0, str(FAKE_RUNTIME_SRC))

from dimos_fake_runtime_sidecar.blueprint import (
    FAKE_RUNTIME_ENV_NAME,
    FAKE_RUNTIME_PROJECT,
    fake_runtime_blueprint,
)
from dimos_fake_runtime_sidecar.module import FakeRuntimeModule
from dimos_runtime_protocol.models import (
    CommandMode,
    EpisodeResetRequest,
    MotorActionFrame,
    MotorStateFrame,
    ObservationFrame,
    StepRequest,
)


def test_fake_runtime_blueprint_places_only_module_in_python_project_runtime() -> None:
    blueprint = fake_runtime_blueprint(robot_id="testbot", dof=2, step_hz=50)

    assert [atom.module for atom in blueprint.blueprints] == [FakeRuntimeModule]
    assert blueprint.blueprints[0].kwargs == {"robot_id": "testbot", "dof": 2, "step_hz": 50}
    assert blueprint.runtime_placement_map == {FakeRuntimeModule: FAKE_RUNTIME_ENV_NAME}

    environment = blueprint.runtime_environment_registry.resolve(FAKE_RUNTIME_ENV_NAME)
    assert isinstance(environment, PythonProjectRuntimeEnvironment)
    assert environment.name == FAKE_RUNTIME_ENV_NAME
    assert environment.project == FAKE_RUNTIME_PROJECT


def test_fake_runtime_blueprint_accepts_explicit_python_project_runtime() -> None:
    runtime = PythonProjectRuntimeEnvironment(
        name="custom-fake-runtime", project=FAKE_RUNTIME_PROJECT
    )

    blueprint = fake_runtime_blueprint(runtime=runtime)

    assert blueprint.runtime_environment_registry.resolve(runtime.name) is runtime
    assert blueprint.runtime_placement_map == {FakeRuntimeModule: runtime.name}


def test_fake_runtime_module_reset_step_score_are_synchronous_and_lightweight() -> None:
    module = _make_local_fake_runtime_module(robot_id="testbot", dof=2, step_hz=10)
    motor_states: list[MotorStateFrame] = []
    color_images: list[Image] = []
    camera_infos: list[CameraInfo] = []
    runtime_events: list[ObservationFrame] = []
    module.motor_state.subscribe(motor_states.append)
    module.color_image.subscribe(color_images.append)
    module.camera_info.subscribe(camera_infos.append)
    module.runtime_event.subscribe(runtime_events.append)

    try:
        description = module.describe()
        reset = module.reset(EpisodeResetRequest(episode_id="episode-1", task_id="task-1"))
        step = module.step(
            StepRequest(
                episode_id="episode-1",
                tick_id=1,
                action=MotorActionFrame(
                    robot_id="testbot",
                    mode=CommandMode.POSITION,
                    names=["testbot/joint1", "testbot/joint2"],
                    q=[1.0, -1.0],
                ),
            )
        )
        score = module.score()

        assert description.observation_streams == ["color_image", "camera_info", "runtime_event"]
        assert description.metadata["module_streams"]["motor_state"] == "motor_state"
        assert reset.episode_id == "episode-1"
        assert reset.runtime_description.observation_streams == [
            "color_image",
            "camera_info",
            "runtime_event",
        ]
        assert reset.observations == []
        assert step.episode_id == "episode-1"
        assert step.tick_id == 1
        assert step.observations == []
        assert step.motor_state.sequence == 1
        assert step.motor_state.q == [0.35, -0.35]
        assert score.episode_id == "episode-1"
        assert score.success is True
        assert score.metrics == {"sequence": 1}

        assert motor_states == [step.motor_state]
        assert len(color_images) == 2
        assert len(camera_infos) == 2
        assert len(runtime_events) == 2
        assert all(isinstance(message, MotorStateFrame) for message in motor_states)
        assert all(isinstance(message, Image) for message in color_images)
        assert all(isinstance(message, CameraInfo) for message in camera_infos)
        assert all(isinstance(message, ObservationFrame) for message in runtime_events)
        assert [event.inline_text for event in runtime_events] == ["reset", "step"]
        assert [event.metadata for event in runtime_events] == [{"sequence": 0}, {"sequence": 1}]
        assert color_images[0].data.shape == (2, 2, 3)
        assert camera_infos[0].width == 2
        assert camera_infos[0].height == 2
    finally:
        module._owner.stop()


def test_fake_runtime_module_rejects_wrong_robot_and_mode() -> None:
    module = _make_local_fake_runtime_module(robot_id="testbot", dof=1, step_hz=10)
    try:
        try:
            module.step(
                StepRequest(
                    episode_id="episode-1",
                    tick_id=1,
                    action=MotorActionFrame(robot_id="other", names=["testbot/joint1"], q=[0.0]),
                )
            )
        except ValueError as exc:
            assert "unexpected robot_id" in str(exc)
        else:
            raise AssertionError("wrong robot id should fail")

        try:
            module.step(
                StepRequest(
                    episode_id="episode-1",
                    tick_id=1,
                    action=MotorActionFrame(
                        robot_id="testbot",
                        mode=CommandMode.TORQUE,
                        names=["testbot/joint1"],
                        q=[0.0],
                    ),
                )
            )
        except ValueError as exc:
            assert "unsupported command mode" in str(exc)
        else:
            raise AssertionError("wrong command mode should fail")
    finally:
        module._owner.stop()


def test_owner_thread_rejects_calls_after_stop() -> None:
    owner = SimulatorOwnerThread(name="test-owner")

    assert owner.call(lambda: "ok") == "ok"
    owner.stop()

    try:
        owner.call(lambda: "late")
    except RuntimeError as exc:
        assert "stopped" in str(exc)
    else:
        raise AssertionError("owner thread should reject calls after stop")


def test_publish_output_supports_remote_transport_only_output() -> None:
    transport = _TransportCapture()
    output = _RemoteOutputLike(transport)

    publish_output(output, "message")

    assert transport.values == ["message"]


def _make_local_fake_runtime_module(*, robot_id: str, dof: int, step_hz: int) -> FakeRuntimeModule:
    with patch.object(Module, "__init__", return_value=None):
        module = FakeRuntimeModule(robot_id=robot_id, dof=dof, step_hz=step_hz)
    module.motor_state = Out(MotorStateFrame, "motor_state", module)
    module.color_image = Out(Image, "color_image", module)
    module.camera_info = Out(CameraInfo, "camera_info", module)
    module.runtime_event = Out(ObservationFrame, "runtime_event", module)
    return module


class _TransportCapture:
    def __init__(self) -> None:
        self.values: list[str] = []

    def publish(self, value: str) -> None:
        self.values.append(value)


class _RemoteOutputLike:
    def __init__(self, transport: _TransportCapture) -> None:
        self.transport = transport
