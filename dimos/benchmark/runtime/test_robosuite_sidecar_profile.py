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

"""Profile tests for the narrow Robosuite sidecar mapping."""

from __future__ import annotations

from collections.abc import Sequence
import importlib.util
from io import BytesIO
from pathlib import Path
import sys
from types import ModuleType
from typing import Protocol, cast

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
ROBOSUITE_SIDECAR_SRC = REPO_ROOT / "packages" / "dimos-robosuite-sidecar" / "src"
sys.path.insert(0, str(PROTOCOL_SRC))
sys.path.insert(0, str(ROBOSUITE_SIDECAR_SRC))

from dimos_robosuite_sidecar.server import (
    RobosuiteRuntimeConfig,
    RobosuiteRuntimeState,
)
from dimos_runtime_protocol import EpisodeResetRequest, MotorActionFrame, StepRequest

DEMO_SCRIPT = REPO_ROOT / "scripts" / "benchmarks" / "demo_robosuite_panda_lift.py"
_DEMO_SPEC = importlib.util.spec_from_file_location("demo_robosuite_panda_lift", DEMO_SCRIPT)
assert _DEMO_SPEC is not None
assert _DEMO_SPEC.loader is not None
_DEMO_MODULE = importlib.util.module_from_spec(_DEMO_SPEC)
sys.modules[_DEMO_SPEC.name] = _DEMO_MODULE
_DEMO_SPEC.loader.exec_module(_DEMO_MODULE)


class _PublishRerunObservations(Protocol):
    def __call__(self, client: object, response_observations: object, publisher: object) -> int: ...


_publish_rerun_observations = cast(
    "_PublishRerunObservations",
    cast("ModuleType", _DEMO_MODULE).__dict__["_publish_rerun_observations"],
)


class _FakeControllers:
    def load_composite_controller_config(self, *, controller: str) -> dict[str, object]:
        return {"type": controller, "body_parts": {"right": {"type": "OSC_POSE"}}}

    def load_part_controller_config(self, *, default_controller: str) -> dict[str, str]:
        return {"type": default_controller}


class _FakeEnv:
    action_spec = ([-1.0] * 8, [1.0] * 8)

    def reset(self) -> dict[str, object]:
        return _fake_obs([0.0] * 7, [0.0])

    def step(self, action: object) -> tuple[dict[str, object], float, bool, dict[str, object]]:
        action_values = [float(item) for item in cast("Sequence[float]", action)]
        return _fake_obs(action_values[:7], [action_values[7]]), 1.0, False, {"success": True}


class _FakeRobosuiteModule:
    controllers = _FakeControllers()

    class macros:
        IMAGE_CONVENTION = "opengl"

    def make(self, **kwargs: object) -> _FakeEnv:
        return _FakeEnv()


class _Mocker(Protocol):
    def patch(self, target: str, **kwargs: object) -> object: ...


class _PayloadClient:
    def __init__(self, state: RobosuiteRuntimeState) -> None:
        self._state = state

    def payload(self, data_ref: str) -> bytes:
        return self._state.payload_bytes(data_ref.removeprefix("/payloads/"))


class _CapturePublisher:
    def __init__(self) -> None:
        self.images: list[np.ndarray] = []
        self.fovs: list[float] = []
        self.frame_ids: list[str] = []

    def publish_rgb(self, rgb: object, *, fov_y_deg: float, frame_id: str) -> None:
        self.images.append(np.asarray(rgb).copy())
        self.fovs.append(fov_y_deg)
        self.frame_ids.append(frame_id)


def test_robosuite_panda_lift_profile_maps_actions_states_and_observations(
    mocker: _Mocker,
) -> None:
    mocker.patch(
        "dimos_robosuite_sidecar.server.require_robosuite",
        return_value=_FakeRobosuiteModule(),
    )
    state = RobosuiteRuntimeState(_config())

    description = state.describe()
    assert description.robot_surfaces[0].robot_id == "panda"
    assert [motor.name for motor in description.robot_surfaces[0].motors] == [
        "panda/joint1",
        "panda/joint2",
        "panda/joint3",
        "panda/joint4",
        "panda/joint5",
        "panda/joint6",
        "panda/joint7",
        "panda/gripper",
    ]

    state.reset(EpisodeResetRequest(episode_id="episode", task_id="Lift"))
    response = state.step(
        StepRequest(
            episode_id="episode",
            tick_id=1,
            action=MotorActionFrame(
                robot_id="panda",
                names=state.motor_names,
                q=[0.1] * 8,
            ),
        )
    )

    assert response.success is True
    assert response.motor_state.names == state.motor_names
    assert response.motor_state.q == [0.1] * 8
    image_frame = next(frame for frame in response.observations if frame.stream == "agentview")
    assert image_frame.encoding == "npy"
    assert image_frame.metadata["camera_name"] == "agentview"
    assert image_frame.metadata["camera_mount"] == "scene"
    assert image_frame.metadata["image_convention"] == "opengl"
    assert image_frame.data_ref is not None
    assert state.payload_bytes(image_frame.data_ref.removeprefix("/payloads/"))
    assert {frame.stream for frame in response.observations} == {"robot_state", "agentview"}


def test_pure_color_camera_payload_round_trips_and_decodes_exactly(mocker: _Mocker) -> None:
    mocker.patch(
        "dimos_robosuite_sidecar.server.require_robosuite",
        return_value=_FakeRobosuiteModule(),
    )
    state = RobosuiteRuntimeState(_config())
    state.reset(EpisodeResetRequest(episode_id="episode", task_id="Lift"))
    response = state.step(
        StepRequest(
            episode_id="episode",
            tick_id=1,
            action=MotorActionFrame(
                robot_id="panda",
                names=state.motor_names,
                q=[0.0] * 8,
            ),
        )
    )

    image_frame = next(frame for frame in response.observations if frame.stream == "agentview")
    assert image_frame.data_ref is not None
    payload = state.payload_bytes(image_frame.data_ref.removeprefix("/payloads/"))
    raw = np.load(BytesIO(payload), allow_pickle=False)
    assert np.array_equal(raw, _pure_color_image())

    publisher = _CapturePublisher()
    published = _publish_rerun_observations(_PayloadClient(state), response.observations, publisher)

    assert published == 1
    assert len(publisher.images) == 1
    assert np.array_equal(publisher.images[0], np.flipud(_pure_color_image()))
    assert publisher.fovs == [45.0]
    assert publisher.frame_ids == ["agentview"]


def _config() -> RobosuiteRuntimeConfig:
    return RobosuiteRuntimeConfig(
        host="127.0.0.1",
        port=8766,
        env_name="Lift",
        robot_id="panda",
        robot_model="Panda",
        controller="JOINT_POSITION",
        control_freq=100,
        horizon=200,
        camera_name="agentview",
        seed=7,
    )


def _fake_obs(joint_q: list[float], gripper_q: list[float]) -> dict[str, object]:
    return {
        "robot0_joint_pos": joint_q,
        "robot0_joint_vel": [0.0] * len(joint_q),
        "robot0_gripper_qpos": gripper_q,
        "robot0_gripper_qvel": [0.0] * len(gripper_q),
        "agentview_image": _pure_color_image(),
    }


def _pure_color_image() -> np.ndarray:
    image = np.zeros((3, 2, 3), dtype=np.uint8)
    image[0, :, :] = [255, 0, 0]
    image[1, :, :] = [0, 255, 0]
    image[2, :, :] = [0, 0, 255]
    return image
