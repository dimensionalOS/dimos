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

"""Stubbed profile tests for the LIBERO-PRO runtime sidecar."""

from __future__ import annotations

from collections.abc import Sequence
from io import BytesIO
from pathlib import Path
import sys
from typing import Literal, cast

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
LIBERO_PRO_SIDECAR_SRC = REPO_ROOT / "packages" / "dimos-libero-pro-sidecar" / "src"
sys.path.insert(0, str(PROTOCOL_SRC))
sys.path.insert(0, str(LIBERO_PRO_SIDECAR_SRC))

from dimos_libero_pro_sidecar.server import (
    NATIVE_ACTION_SPACE_ID,
    LiberoProRuntimeConfig,
    LiberoProRuntimeState,
    RealLiberoBackend,
    ensure_libero_config,
    validate_assets,
)
from dimos_runtime_protocol import (
    EpisodeResetRequest,
    MotorActionFrame,
    RuntimeActionFrame,
    StepRequest,
)


class _FakeLiberoBackend:
    action_low = [-1.0] * 8
    action_high = [1.0] * 8
    task_name = "pick_up_the_black_bowl"
    language = "pick up the black bowl"

    def reset(self, init_state_index: int) -> dict[str, object]:
        return _fake_obs([0.0] * 7, [0.0]) | {"init_state_index": init_state_index}

    def step(
        self, action: Sequence[float]
    ) -> tuple[dict[str, object], float, bool, dict[str, object]]:
        values = [float(item) for item in action]
        return _fake_obs(values[:7], [values[7]]), 0.75, False, {"success": True}

    def render(self) -> None:
        return


class _RenderCountingBackend(_FakeLiberoBackend):
    def __init__(self) -> None:
        self.render_count = 0

    def render(self) -> None:
        self.render_count += 1


class _BadActionBackend(_FakeLiberoBackend):
    action_low = [-1.0] * 7
    action_high = [1.0] * 7


class _FakeNativeBackend(_FakeLiberoBackend):
    action_low = [-1.0] * 7
    action_high = [1.0] * 7

    def __init__(self) -> None:
        self.last_action: list[float] = []

    def step(
        self, action: Sequence[float]
    ) -> tuple[dict[str, object], float, bool, dict[str, object]]:
        self.last_action = [float(item) for item in action]
        return _fake_obs(self.last_action[:7], [0.0]), 0.5, False, {"success": False}


class _BadNativeBoundsBackend(_FakeNativeBackend):
    action_low = [-0.5] * 7
    action_high = [1.0] * 7


class _FakeController:
    def __init__(self) -> None:
        self.use_delta = False


class _FakeRobot:
    def __init__(self) -> None:
        self.controller = _FakeController()


class _FakeNativeEnv:
    action_spec = ([-1.0] * 7, [1.0] * 7)

    def __init__(self, **kwargs: object) -> None:
        self.kwargs = kwargs
        self.robots = [_FakeRobot()]
        self.actions: list[list[float]] = []

    def reset(self) -> dict[str, object]:
        self.robots[0].controller.use_delta = False
        return {"reset": True}

    def set_init_state(self, state: object) -> dict[str, object]:
        return {"state": state}

    def step(
        self, action: Sequence[float]
    ) -> tuple[dict[str, object], float, bool, dict[str, object]]:
        values = [float(item) for item in action]
        self.actions.append(values)
        return {"noop_count": len(self.actions)}, 0.0, False, {}


def test_libero_pro_profile_maps_actions_states_score_and_payloads(tmp_path: Path) -> None:
    state = LiberoProRuntimeState(_config(tmp_path), backend=_FakeLiberoBackend())

    description = state.describe()
    assert description.backend == "libero-pro"
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
    assert description.metadata["benchmark_name"] == "libero_pro"

    reset = state.reset(EpisodeResetRequest(episode_id="episode", task_id="task"))
    assert {frame.stream for frame in reset.observations} == {"robot_state", "agentview"}

    response = state.step(
        StepRequest(
            episode_id="episode",
            tick_id=1,
            action=MotorActionFrame(robot_id="panda", names=state.motor_names, q=[0.1] * 8),
        )
    )

    assert response.success is True
    assert response.motor_state.q == [0.1] * 8
    image_frame = next(frame for frame in response.observations if frame.stream == "agentview")
    assert image_frame.data_ref is not None
    assert image_frame.metadata["image_convention"] == "opengl"
    assert image_frame.metadata["camera_source"] == "libero_pro_observation"
    payload = state.payload_bytes(image_frame.data_ref.removeprefix("/payloads/"))
    assert np.array_equal(np.load(BytesIO(payload), allow_pickle=False), _pure_color_image())

    score = state.score()
    assert score.success is True
    assert score.metrics["steps"] == 1
    assert score.metrics["task_name"] == "pick_up_the_black_bowl"
    assert score.metrics["language"] == "pick up the black bowl"


def test_libero_pro_rejects_incompatible_action_dimension(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match="action_dim=7"):
        LiberoProRuntimeState(_config(tmp_path), backend=_BadActionBackend())


def test_libero_pro_native_mode_description_advertises_runtime_action_surface(
    tmp_path: Path,
) -> None:
    state = LiberoProRuntimeState(
        _config(tmp_path, action_mode="native"), backend=_FakeNativeBackend()
    )

    description = state.describe()

    assert "runtime-action" in description.capabilities
    assert description.metadata["action_mode"] == "native"
    assert description.metadata["native_action_space_id"] == NATIVE_ACTION_SPACE_ID
    assert description.metadata["action_shape"] == [7]
    assert description.metadata["action_low"] == [-1.0] * 7
    assert description.metadata["action_high"] == [1.0] * 7
    assert description.metadata["controller"] == "JOINT_POSITION"
    assert description.metadata["effective_controller"] == "OSC_POSE"
    assert description.metadata["task_metadata"] == {
        "benchmark_name": "libero_pro",
        "task_order_index": 0,
        "task_index": 0,
        "task_name": "pick_up_the_black_bowl",
        "init_state_index": 2,
    }
    assert description.metadata["language"] == "pick up the black bowl"
    assert description.metadata["horizon"] == 1000
    assert description.metadata["effective_horizon"] == 1010
    assert description.metadata["reset_settle_steps"] == 10
    assert description.metadata["camera_config"] == {
        "names": ["agentview"],
        "height": 128,
        "width": 128,
    }


def test_libero_pro_native_mode_rejects_bad_action_spec(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match="bounds compatible"):
        LiberoProRuntimeState(
            _config(tmp_path, action_mode="native"), backend=_BadNativeBoundsBackend()
        )


def test_libero_pro_native_mode_steps_runtime_action_directly(tmp_path: Path) -> None:
    backend = _FakeNativeBackend()
    state = LiberoProRuntimeState(_config(tmp_path, action_mode="native"), backend=backend)

    response = state.step(
        StepRequest(
            episode_id="episode",
            tick_id=1,
            action=RuntimeActionFrame(
                frame_type="runtime_action",
                space_id=NATIVE_ACTION_SPACE_ID,
                values=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
                tick_id=1,
            ),
        )
    )

    assert backend.last_action == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    assert response.reward == 0.5


def test_real_backend_native_reset_runs_lerobot_noops_and_use_delta(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    envs: list[_FakeNativeEnv] = []

    class FakeBenchmark:
        def get_task(self, task_index: int) -> object:
            return type("Task", (), {"name": "task", "language": "language"})()

    def fake_env_cls(**kwargs: object) -> _FakeNativeEnv:
        env = _FakeNativeEnv(**kwargs)
        envs.append(env)
        return env

    monkeypatch.setattr(
        "dimos_libero_pro_sidecar.server.require_libero",
        lambda *, visualize=False: (
            type(
                "BenchmarkModule",
                (),
                {"get_benchmark": lambda self, name: lambda order: FakeBenchmark()},
            )(),
            fake_env_cls,
        ),
    )
    monkeypatch.setattr("dimos_libero_pro_sidecar.server.ensure_libero_config", lambda *_: None)
    monkeypatch.setattr("dimos_libero_pro_sidecar.server._load_init_states", lambda *_: ["init"])

    backend = RealLiberoBackend(_config(tmp_path, action_mode="native"))
    obs = backend.reset(0)

    env = envs[0]
    assert env.kwargs["controller"] == "OSC_POSE"
    assert env.kwargs["horizon"] == 1010
    assert env.robots[0].controller.use_delta is True
    assert env.actions == [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]] * 10
    assert obs == {"noop_count": 10}


def test_libero_pro_native_mode_rejects_motor_frame(tmp_path: Path) -> None:
    state = LiberoProRuntimeState(
        _config(tmp_path, action_mode="native"), backend=_FakeNativeBackend()
    )

    with pytest.raises(ValueError, match="native action mode requires RuntimeActionFrame"):
        state.step(
            StepRequest(
                episode_id="episode",
                tick_id=1,
                action=MotorActionFrame(robot_id="panda", names=state.motor_names, q=[0.1] * 8),
            )
        )


def test_libero_pro_motor_mode_rejects_runtime_frame(tmp_path: Path) -> None:
    state = LiberoProRuntimeState(_config(tmp_path), backend=_FakeLiberoBackend())

    with pytest.raises(ValueError, match="motor action mode requires MotorActionFrame"):
        state.step(
            StepRequest(
                episode_id="episode",
                tick_id=1,
                action=RuntimeActionFrame(
                    frame_type="runtime_action",
                    space_id=NATIVE_ACTION_SPACE_ID,
                    values=[0.1] * 7,
                    tick_id=1,
                ),
            )
        )


def test_libero_pro_rejects_unsupported_controller(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match="unsupported LIBERO-PRO controller"):
        LiberoProRuntimeState(
            _config(tmp_path, controller="OSC_POSE"), backend=_FakeLiberoBackend()
        )


def test_libero_pro_visual_mode_renders_after_reset_and_step(tmp_path: Path) -> None:
    backend = _RenderCountingBackend()
    state = LiberoProRuntimeState(_config(tmp_path, visualize=True), backend=backend)

    state.reset(EpisodeResetRequest(episode_id="episode", task_id="task"))
    state.step(
        StepRequest(
            episode_id="episode",
            tick_id=1,
            action=MotorActionFrame(robot_id="panda", names=state.motor_names, q=[0.1] * 8),
        )
    )

    assert backend.render_count == 2


def test_libero_pro_asset_validation_does_not_bootstrap_by_default(tmp_path: Path) -> None:
    config = LiberoProRuntimeConfig(
        host="127.0.0.1",
        port=8767,
        benchmark_name="libero_pro",
        bddl_root=tmp_path / "missing-bddl",
        init_states_root=tmp_path / "missing-init",
    )

    with pytest.raises(FileNotFoundError, match="BDDL root"):
        validate_assets(config)


def test_libero_config_is_created_noninteractively(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config_root = tmp_path / "libero-config"
    bddl_root = tmp_path / "libero" / "bddl_files"
    init_states_root = tmp_path / "libero" / "init_files"
    monkeypatch.setenv("LIBERO_CONFIG_PATH", str(config_root))

    ensure_libero_config(bddl_root, init_states_root)

    config_text = (config_root / "config.yaml").read_text()
    assert f"bddl_files: {bddl_root}" in config_text
    assert f"init_states: {init_states_root}" in config_text


def _config(
    tmp_path: Path,
    *,
    action_mode: str = "motor",
    controller: str = "JOINT_POSITION",
    visualize: bool = False,
) -> LiberoProRuntimeConfig:
    bddl_root = tmp_path / "bddl"
    init_states_root = tmp_path / "init_states"
    bddl_root.mkdir()
    init_states_root.mkdir()
    (bddl_root / "task.bddl").write_text("fixture")
    (init_states_root / "task.pruned_init").write_bytes(b"fixture")
    return LiberoProRuntimeConfig(
        host="127.0.0.1",
        port=8767,
        benchmark_name="libero_pro",
        bddl_root=bddl_root,
        init_states_root=init_states_root,
        action_mode=cast("Literal['motor', 'native']", action_mode),
        controller=controller,
        camera_names=("agentview",),
        init_state_index=2,
        visualize=visualize,
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
    image = np.zeros((2, 2, 3), dtype=np.uint8)
    image[0, :, :] = [255, 0, 0]
    image[1, :, :] = [0, 255, 0]
    return image
