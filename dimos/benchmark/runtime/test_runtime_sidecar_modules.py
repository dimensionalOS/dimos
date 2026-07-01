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

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""Focused tests for first-class Robosuite/LIBERO-PRO runtime modules."""

from __future__ import annotations

from io import BytesIO
import os
from pathlib import Path
import subprocess
import sys
import threading
from typing import ClassVar, Protocol, cast

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
ROBOSUITE_SIDECAR_SRC = REPO_ROOT / "packages" / "dimos-robosuite-sidecar" / "src"
LIBERO_PRO_SIDECAR_SRC = REPO_ROOT / "packages" / "dimos-libero-pro-sidecar" / "src"
for _src in (PROTOCOL_SRC, ROBOSUITE_SIDECAR_SRC, LIBERO_PRO_SIDECAR_SRC):
    sys.path.insert(0, str(_src))

from dimos_libero_pro_sidecar.blueprint import libero_pro_runtime_blueprint
from dimos_libero_pro_sidecar.module import LiberoProRuntimeModule
from dimos_robosuite_sidecar.blueprint import robosuite_runtime_blueprint
from dimos_robosuite_sidecar.module import RobosuiteRuntimeModule
from dimos_runtime_protocol import EpisodeResetRequest, MotorActionFrame, StepRequest
from dimos_runtime_protocol.models import (
    EpisodeResetResponse,
    MotorDescription,
    MotorStateFrame,
    ObservationFrame,
    ObservationKind,
    RobotMotorSurface,
    RuntimeDescription,
    ScoreOutput,
    StepResponse,
)

from dimos.core.runtime_environment import PythonProjectRuntimeEnvironment
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image


class _Mocker(Protocol):
    def patch(self, target: str, **kwargs: object) -> object: ...


class _CaptureOut:
    def __init__(self) -> None:
        self.values: list[object] = []

    def publish(self, value: object) -> None:
        self.values.append(value)


class _RuntimeModuleProtocol(Protocol):
    motor_state: _CaptureOut
    color_image: _CaptureOut
    camera_info: _CaptureOut
    runtime_event: _CaptureOut


class _StubRobosuiteState:
    init_count: ClassVar[int] = 0

    def __init__(self, config: object) -> None:
        type(self).init_count += 1
        self.config = config
        self._sequence = 0
        self.thread_ids: list[int] = [threading.get_ident()]

    def describe(self) -> RuntimeDescription:
        self.thread_ids.append(threading.get_ident())
        return _description("robosuite")

    def reset(self, request: EpisodeResetRequest) -> object:
        self.thread_ids.append(threading.get_ident())
        return _reset_response(request.episode_id, "robosuite")

    def step(self, request: StepRequest) -> StepResponse:
        self.thread_ids.append(threading.get_ident())
        self._sequence += 1
        return _step_response(request, self._sequence)

    def score(self) -> ScoreOutput:
        self.thread_ids.append(threading.get_ident())
        return ScoreOutput(episode_id="episode", success=True, score=1.0)

    def _camera_image(self) -> np.ndarray:
        return _corrupt_image()

    def _camera_fov_deg(self) -> float:
        return 45.0

    def payload_bytes(self, payload_id: str) -> bytes:
        assert payload_id == "agentview-000001-000001.npy"
        return _npy_bytes(_image())


class _StubLiberoState(_StubRobosuiteState):
    def __init__(self, config: object) -> None:
        super().__init__(config)
        self._last_obs = {"agentview_image": _image()}

    def reset(self, request: EpisodeResetRequest) -> object:
        self.thread_ids.append(threading.get_ident())
        return _reset_response(request.episode_id, "libero")

    def step(self, request: StepRequest) -> StepResponse:
        self.thread_ids.append(threading.get_ident())
        self._sequence += 1
        return _step_response(request, self._sequence, image_convention=None)


class _FailingLiberoState:
    def __init__(self, config: object) -> None:
        raise RuntimeError("missing LIBERO assets")


def test_blueprint_helpers_place_only_runtime_module() -> None:
    robosuite = robosuite_runtime_blueprint()
    libero_runtime = PythonProjectRuntimeEnvironment(
        name="libero-test", project=LIBERO_PRO_SIDECAR_SRC.parent
    )
    libero = libero_pro_runtime_blueprint(
        bddl_root="/tmp/bddl", init_states_root="/tmp/init", runtime=libero_runtime
    )

    assert [atom.module for atom in robosuite.blueprints] == [RobosuiteRuntimeModule]
    assert isinstance(
        robosuite.runtime_environment_registry.resolve("dimos-robosuite-runtime"),
        PythonProjectRuntimeEnvironment,
    )
    assert dict(robosuite.runtime_placement_map) == {
        RobosuiteRuntimeModule: "dimos-robosuite-runtime"
    }
    assert [atom.module for atom in libero.blueprints] == [LiberoProRuntimeModule]
    assert libero.runtime_environment_registry.resolve("libero-test") is libero_runtime
    assert dict(libero.runtime_placement_map) == {LiberoProRuntimeModule: "libero-test"}


def test_importing_runtime_modules_does_not_import_heavy_dependencies() -> None:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [str(PROTOCOL_SRC), str(ROBOSUITE_SIDECAR_SRC), str(LIBERO_PRO_SIDECAR_SRC), str(REPO_ROOT)]
    )
    code = """
import importlib
import sys
for name in ('dimos_robosuite_sidecar.module', 'dimos_robosuite_sidecar.blueprint', 'dimos_libero_pro_sidecar.module', 'dimos_libero_pro_sidecar.blueprint'):
    importlib.import_module(name)
heavy = {'robosuite', 'libero', 'torch'} & set(sys.modules)
if heavy:
    raise SystemExit(f'heavy modules imported: {sorted(heavy)}')
"""
    subprocess.run([sys.executable, "-c", code], check=True, env=env)


def test_robosuite_runtime_module_rpc_and_stream_outputs(mocker: _Mocker) -> None:
    _StubRobosuiteState.init_count = 0
    mocker.patch(
        "dimos_robosuite_sidecar.module.RobosuiteRuntimeState", side_effect=_StubRobosuiteState
    )
    module = RobosuiteRuntimeModule()
    _attach_outputs(module)
    caller_thread_id = threading.get_ident()

    assert module._state is None
    description = module.describe()
    reset = module.reset(EpisodeResetRequest(episode_id="episode", task_id="task"))
    step = module.step(_step_request())
    score = module.score()

    assert description.observation_streams == ["color_image", "camera_info", "runtime_event"]
    assert description.metadata["backend_camera_streams"] == ["agentview"]
    assert description.metadata["module_streams"]["color_image"] == "color_image"
    assert reset.observations == []
    assert reset.runtime_description.observation_streams == [
        "color_image",
        "camera_info",
        "runtime_event",
    ]
    assert "sync-http" not in reset.runtime_description.capabilities
    assert step.observations == []
    assert score.success is True
    _assert_published_native_types(module, expected_image=np.flipud(_image()))
    state = module._state
    assert state is not None
    assert _StubRobosuiteState.init_count == 1
    assert len(set(state.thread_ids)) == 1
    assert state.thread_ids[0] != caller_thread_id
    module.stop()


def test_libero_pro_runtime_module_rpc_and_stream_outputs(mocker: _Mocker, tmp_path: Path) -> None:
    _StubLiberoState.init_count = 0
    mocker.patch(
        "dimos_libero_pro_sidecar.module.LiberoProRuntimeState", side_effect=_StubLiberoState
    )
    module = LiberoProRuntimeModule(bddl_root=tmp_path, init_states_root=tmp_path)
    _attach_outputs(module)
    caller_thread_id = threading.get_ident()

    assert module._state is None
    description = module.describe()
    reset = module.reset(EpisodeResetRequest(episode_id="episode", task_id="task"))
    step = module.step(_step_request())
    score = module.score()
    snapshot_observations, snapshot_values = module.observation_snapshot()

    assert description.observation_streams == ["color_image", "camera_info", "runtime_event"]
    assert description.metadata["backend_camera_streams"] == ["agentview"]
    assert reset.observations == []
    assert "sync-http" not in reset.runtime_description.capabilities
    assert step.observations == []
    assert score.score == 1.0
    assert {observation.stream for observation in snapshot_observations} >= {
        "agentview",
        "runtime_event",
    }
    image_snapshot = next(
        observation for observation in snapshot_observations if observation.stream == "agentview"
    )
    assert image_snapshot.data_ref == "/payloads/agentview-000001-000001.npy"
    assert image_snapshot.metadata == {"camera_name": "agentview", "fov_y_deg": 45.0}
    np.testing.assert_array_equal(snapshot_values["agentview"], _image())
    _assert_published_native_types(module, expected_image=_image())
    assert module._state is not None
    assert _StubLiberoState.init_count == 1
    assert len(set(module._state.thread_ids)) == 1
    assert module._state.thread_ids[0] != caller_thread_id
    module.stop()


def test_libero_runtime_state_failures_happen_on_reset_not_deploy(
    mocker: _Mocker, tmp_path: Path
) -> None:
    mocker.patch(
        "dimos_libero_pro_sidecar.module.LiberoProRuntimeState", side_effect=_FailingLiberoState
    )
    module = LiberoProRuntimeModule(bddl_root=tmp_path, init_states_root=tmp_path)
    _attach_outputs(module)

    assert module._state is None
    try:
        try:
            module.reset(EpisodeResetRequest(episode_id="episode", task_id="task"))
        except RuntimeError as exc:
            assert "missing LIBERO assets" in str(exc)
        else:
            raise AssertionError("reset should surface runtime setup failure")
    finally:
        module.stop()


def _attach_outputs(module: _RuntimeModuleProtocol) -> None:
    module.motor_state = _CaptureOut()
    module.color_image = _CaptureOut()
    module.camera_info = _CaptureOut()
    module.runtime_event = _CaptureOut()


def _assert_published_native_types(
    module: _RuntimeModuleProtocol, *, expected_image: np.ndarray
) -> None:
    assert isinstance(module.motor_state.values[-1], MotorStateFrame)
    assert isinstance(module.color_image.values[-1], Image)
    assert isinstance(module.camera_info.values[-1], CameraInfo)
    color_image = cast("Image", module.color_image.values[-1])
    np.testing.assert_array_equal(color_image.data, expected_image)
    assert all(isinstance(value, ObservationFrame) for value in module.runtime_event.values)
    events = cast(
        "list[ObservationFrame]",
        [value for value in module.runtime_event.values if isinstance(value, ObservationFrame)],
    )
    assert {value.kind for value in events} >= {
        ObservationKind.STATE,
        ObservationKind.TEXT,
    }


def _description(backend: str) -> RuntimeDescription:
    return RuntimeDescription(
        runtime_id=f"{backend}-runtime",
        backend=backend,
        capabilities=["sync-http", "step"],
        robot_surfaces=[
            RobotMotorSurface(
                robot_id="panda",
                motors=[MotorDescription(name="panda/joint1", index=0)],
            )
        ],
        control_step_hz=20,
    )


def _reset_response(episode_id: str, backend: str) -> EpisodeResetResponse:
    return EpisodeResetResponse(
        episode_id=episode_id,
        runtime_description=_description(backend),
        observations=[
            _state_observation(),
            _image_observation("opengl" if backend == "robosuite" else None),
        ],
    )


def _step_request() -> StepRequest:
    return StepRequest(
        episode_id="episode",
        tick_id=1,
        action=MotorActionFrame(robot_id="panda", names=["panda/joint1"], q=[0.25]),
    )


def _step_response(
    request: StepRequest, sequence: int, *, image_convention: str | None = "opengl"
) -> StepResponse:
    return StepResponse(
        episode_id=request.episode_id,
        tick_id=request.tick_id,
        motor_state=MotorStateFrame(
            robot_id="panda",
            names=["panda/joint1"],
            q=[0.25],
            dq=[0.0],
            tau=[0.0],
            sequence=sequence,
        ),
        observations=[_state_observation(), _image_observation(image_convention)],
        success=True,
    )


def _image_observation(image_convention: str | None) -> ObservationFrame:
    metadata: dict[str, object] = {"camera_name": "agentview", "fov_y_deg": 45.0}
    if image_convention is not None:
        metadata["image_convention"] = image_convention
    return ObservationFrame(
        stream="agentview",
        kind=ObservationKind.IMAGE,
        encoding="npy",
        data_ref="/payloads/agentview-000001-000001.npy",
        metadata=metadata,
    )


def _state_observation() -> ObservationFrame:
    return ObservationFrame(
        stream="robot_state", kind=ObservationKind.STATE, metadata={"source": "stub"}
    )


def _image() -> np.ndarray:
    return np.array(
        [
            [[1, 2, 3], [4, 5, 6], [7, 8, 9]],
            [[10, 11, 12], [13, 14, 15], [16, 17, 18]],
        ],
        dtype=np.uint8,
    )


def _corrupt_image() -> np.ndarray:
    return np.full((2, 3, 3), 255, dtype=np.uint8)


def _npy_bytes(value: np.ndarray) -> bytes:
    buffer = BytesIO()
    np.save(buffer, value)
    return buffer.getvalue()
