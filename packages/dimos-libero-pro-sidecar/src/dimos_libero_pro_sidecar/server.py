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

"""LIBERO-PRO runtime state for Simulator Runtime Modules."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import importlib.util
from io import BytesIO
import os
from pathlib import Path
import time
from typing import Literal, Protocol, cast

from dimos_runtime_protocol import (
    CommandMode,
    EpisodeResetRequest,
    EpisodeResetResponse,
    MotorActionFrame,
    MotorDescription,
    MotorStateFrame,
    ObservationFrame,
    ObservationKind,
    RobotMotorSurface,
    RuntimeActionFrame,
    RuntimeDescription,
    ScoreOutput,
    StepRequest,
    StepResponse,
)

ActionMode = Literal["motor", "native"]
NATIVE_ACTION_SPACE_ID = "libero.ee_delta_6d_gripper.normalized.v1"
NATIVE_LIBERO_CONTROLLER = "OSC_POSE"
NATIVE_RESET_SETTLE_STEPS = 10


def require_libero(*, visualize: bool = False) -> tuple[object, LiberoEnvFactory]:
    """Import LIBERO-PRO dependencies only on the runtime path."""

    try:
        try:
            from libero import benchmark as libero_benchmark

            if visualize:
                from libero.envs.env_wrapper import ControlEnv

                env_cls = ControlEnv
            else:
                from libero.envs import OffScreenRenderEnv

                env_cls = OffScreenRenderEnv
        except ImportError:
            from libero.libero import benchmark as libero_benchmark

            if visualize:
                from libero.libero.envs.env_wrapper import ControlEnv

                env_cls = ControlEnv
            else:
                from libero.libero.envs import OffScreenRenderEnv

                env_cls = OffScreenRenderEnv
    except ImportError as exc:
        raise RuntimeError(
            "LIBERO-PRO dependencies are required for dimos-libero-pro-sidecar. "
            "Install the sidecar in a LIBERO/Robosuite-compatible environment."
        ) from exc
    return libero_benchmark, cast("LiberoEnvFactory", env_cls)


@dataclass(frozen=True)
class LiberoProRuntimeConfig:
    host: str
    port: int
    benchmark_name: str
    bddl_root: Path
    init_states_root: Path
    robot_id: str = "panda"
    task_order_index: int = 0
    task_index: int = 0
    init_state_index: int = 0
    action_mode: ActionMode = "motor"
    controller: str = "JOINT_POSITION"
    camera_names: tuple[str, ...] = ("agentview",)
    camera_height: int = 128
    camera_width: int = 128
    control_freq: int = 20
    horizon: int = 1000
    seed: int | None = None
    allow_asset_bootstrap: bool = False
    visualize: bool = False


class LiberoEnv(Protocol):
    action_spec: tuple[Sequence[float], Sequence[float]]

    def reset(self) -> dict[str, object]: ...

    def set_init_state(self, state: object) -> dict[str, object]: ...

    def step(
        self, action: Sequence[float]
    ) -> tuple[dict[str, object], float, bool, dict[str, object]]: ...


class LiberoEnvFactory(Protocol):
    def __call__(
        self,
        *,
        bddl_file_name: str,
        robots: list[str],
        use_camera_obs: bool,
        has_renderer: bool,
        has_offscreen_renderer: bool,
        camera_heights: int,
        camera_widths: int,
        camera_names: list[str],
        controller: str,
        control_freq: int,
        horizon: int,
        render_camera: str | None = None,
    ) -> LiberoEnv: ...


class LiberoBackend(Protocol):
    action_low: Sequence[float]
    action_high: Sequence[float]
    task_name: str
    language: str

    def reset(self, init_state_index: int) -> dict[str, object]: ...

    def step(
        self, action: Sequence[float]
    ) -> tuple[dict[str, object], float, bool, dict[str, object]]: ...


class RealLiberoBackend:
    def __init__(self, config: LiberoProRuntimeConfig) -> None:
        ensure_libero_config(config.bddl_root, config.init_states_root)
        libero_benchmark, env_cls = require_libero(visualize=config.visualize)
        benchmark_factory = libero_benchmark.get_benchmark(config.benchmark_name)
        benchmark = benchmark_factory(config.task_order_index)
        task = benchmark.get_task(config.task_index)
        self.task_name = str(getattr(task, "name", f"task-{config.task_index}"))
        self.language = str(getattr(task, "language", getattr(task, "problem_folder", "")))
        bddl_file = _task_bddl_file(config, benchmark, task)
        init_states = _load_init_states(config, benchmark, task)
        self._init_states = init_states
        self._action_mode = config.action_mode
        self._env: LiberoEnv = env_cls(
            bddl_file_name=str(bddl_file),
            robots=["Panda"],
            use_camera_obs=True,
            has_renderer=config.visualize,
            has_offscreen_renderer=True,
            camera_heights=config.camera_height,
            camera_widths=config.camera_width,
            camera_names=list(config.camera_names),
            controller=_env_controller(config),
            control_freq=config.control_freq,
            horizon=config.horizon + _reset_settle_steps(config),
            render_camera=config.camera_names[0] if config.camera_names else None,
        )
        if config.action_mode == "native":
            _set_controller_use_delta(self._env)
        self.action_low, self.action_high = _action_bounds(self._env)

    def reset(self, init_state_index: int) -> dict[str, object]:
        obs = self._env.reset()
        set_init_state = self._env.set_init_state
        obs = set_init_state(self._init_states[init_state_index])
        if self._action_mode == "native":
            for _ in range(NATIVE_RESET_SETTLE_STEPS):
                obs, _, _, _ = self._env.step([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0])
            _set_controller_use_delta(self._env)
        return cast("dict[str, object]", obs)

    def step(
        self, action: Sequence[float]
    ) -> tuple[dict[str, object], float, bool, dict[str, object]]:
        obs, reward, done, info = self._env.step(action)
        typed_info = cast("dict[str, object]", info)
        if not any(key in typed_info for key in ("success", "is_success", "task_success")):
            check_success = getattr(self._env, "check_success", None)
            if callable(check_success):
                typed_info["success"] = bool(check_success())
        return cast("dict[str, object]", obs), float(reward), bool(done), typed_info

    def render(self) -> None:
        render = getattr(self._env, "render", None)
        if callable(render):
            render()
            return
        wrapped_env = getattr(self._env, "env", None)
        wrapped_render = getattr(wrapped_env, "render", None)
        if callable(wrapped_render):
            wrapped_render()


class LiberoProRuntimeState:
    """Owns one LIBERO-PRO backend and maps it to runtime protocol models."""

    def __init__(
        self, config: LiberoProRuntimeConfig, backend: LiberoBackend | None = None
    ) -> None:
        self.config = config
        validate_assets(config)
        self._backend = backend or RealLiberoBackend(config)
        self._episode_id = "uninitialized"
        self._sequence = 0
        self._last_obs: dict[str, object] = {}
        self._last_reward = 0.0
        self._last_done = False
        self._last_success: bool | None = None
        self._payloads: dict[str, bytes] = {}
        self._action_low = [float(v) for v in self._backend.action_low]
        self._action_high = [float(v) for v in self._backend.action_high]
        self.motor_names = [f"{config.robot_id}/joint{i + 1}" for i in range(7)] + [
            f"{config.robot_id}/gripper"
        ]
        self._validate_action_surface()

    def _validate_action_surface(self) -> None:
        if self.config.action_mode == "native":
            self._validate_native_action_surface()
            return
        if self.config.action_mode != "motor":
            raise RuntimeError(f"unsupported LIBERO-PRO action mode: {self.config.action_mode}")
        self._validate_motor_action_surface()

    def _validate_motor_action_surface(self) -> None:
        if self.config.controller not in {"JOINT_POSITION", "PANDA_JOINT_POSITION"}:
            raise RuntimeError(f"unsupported LIBERO-PRO controller: {self.config.controller}")
        if len(self._action_low) != len(self.motor_names) or len(self._action_high) != len(
            self.motor_names
        ):
            raise RuntimeError(
                "LIBERO-PRO profile expects Panda 7 joint-position actions plus gripper: "
                f"action_dim={len(self._action_low)} motors={len(self.motor_names)}"
            )

    def _validate_native_action_surface(self) -> None:
        if len(self._action_low) != 7 or len(self._action_high) != 7:
            raise RuntimeError(
                f"native LIBERO action mode expects action_dim=7, got {len(self._action_low)}"
            )
        incompatible = [
            (low, high)
            for low, high in zip(self._action_low, self._action_high, strict=True)
            if low > -1.0 or high < 1.0
        ]
        if incompatible:
            raise RuntimeError(
                "native LIBERO action mode requires action_spec bounds compatible with [-1, 1]"
            )

    def describe(self) -> RuntimeDescription:
        return RuntimeDescription(
            runtime_id="libero-pro",
            backend="libero-pro",
            capabilities=[
                "sync-http",
                "libero-pro",
                "whole-body-motor-position"
                if self.config.action_mode == "motor"
                else "runtime-action",
            ],
            robot_surfaces=[
                RobotMotorSurface(
                    robot_id=self.config.robot_id,
                    motors=[
                        MotorDescription(name=n, index=i) for i, n in enumerate(self.motor_names)
                    ],
                    supported_command_modes=[CommandMode.POSITION],
                )
            ],
            control_step_hz=self.config.control_freq,
            observation_streams=[*self.config.camera_names, "robot_state"],
            metadata={
                "benchmark_name": self.config.benchmark_name,
                "task_order_index": self.config.task_order_index,
                "task_index": self.config.task_index,
                "task_name": self._backend.task_name,
                "language": self._backend.language,
                "bddl_root": str(self.config.bddl_root),
                "init_states_root": str(self.config.init_states_root),
                "init_state_index": self.config.init_state_index,
                "controller": self.config.controller,
                "effective_controller": _env_controller(self.config),
                "effective_horizon": self.config.horizon + _reset_settle_steps(self.config),
                "reset_settle_steps": _reset_settle_steps(self.config),
                "action_mode": self.config.action_mode,
                "horizon": self.config.horizon,
                "visualize": self.config.visualize,
                "camera_names": list(self.config.camera_names),
                "camera_config": {
                    "names": list(self.config.camera_names),
                    "height": self.config.camera_height,
                    "width": self.config.camera_width,
                },
                "action_low": self._action_low,
                "action_high": self._action_high,
                "action_shape": [len(self._action_low)],
                "task_metadata": {
                    "benchmark_name": self.config.benchmark_name,
                    "task_order_index": self.config.task_order_index,
                    "task_index": self.config.task_index,
                    "task_name": self._backend.task_name,
                    "init_state_index": self.config.init_state_index,
                },
                "native_action_space_id": NATIVE_ACTION_SPACE_ID,
            },
        )

    def reset(self, request: EpisodeResetRequest) -> EpisodeResetResponse:
        self._episode_id = request.episode_id
        self._sequence = 0
        self._last_reward = 0.0
        self._last_done = False
        self._last_success = None
        self._last_obs = self._backend.reset(self.config.init_state_index)
        observations = self._observations(0)
        self._render_if_enabled()
        return EpisodeResetResponse(
            episode_id=request.episode_id,
            runtime_description=self.describe(),
            observations=observations,
        )

    def step(self, request: StepRequest) -> StepResponse:
        if self.config.action_mode == "native":
            action = self._native_step_action(request.action)
        else:
            action = self._motor_step_action(request.action)
        obs, reward, done, info = self._backend.step(action)
        self._sequence += 1
        self._last_obs = obs
        self._last_reward = reward
        self._last_done = done
        self._last_success = _success_from_info(info)
        observations = self._observations(request.tick_id)
        self._render_if_enabled()
        return StepResponse(
            episode_id=request.episode_id,
            tick_id=request.tick_id,
            motor_state=self._motor_state(),
            observations=observations,
            reward=reward,
            done=done,
            success=self._last_success,
            info={"backend_sequence": self._sequence},
        )

    def _motor_step_action(
        self, action_frame: MotorActionFrame | RuntimeActionFrame
    ) -> list[float]:
        if isinstance(action_frame, RuntimeActionFrame):
            raise ValueError("motor action mode requires MotorActionFrame, got RuntimeActionFrame")
        if action_frame.robot_id != self.config.robot_id:
            raise ValueError(f"unexpected robot id {action_frame.robot_id!r}")
        if action_frame.names != self.motor_names:
            raise ValueError("action motor names do not match runtime surface")
        if action_frame.mode != CommandMode.POSITION:
            raise ValueError(f"unsupported command mode {action_frame.mode}")
        if len(action_frame.q) != len(self.motor_names):
            raise ValueError(
                f"expected {len(self.motor_names)} q targets, got {len(action_frame.q)}"
            )
        return [
            min(max(float(v), low), high)
            for v, low, high in zip(
                action_frame.q, self._action_low, self._action_high, strict=True
            )
        ]

    def _native_step_action(
        self, action_frame: MotorActionFrame | RuntimeActionFrame
    ) -> list[float]:
        if isinstance(action_frame, MotorActionFrame):
            raise ValueError("native action mode requires RuntimeActionFrame, got MotorActionFrame")
        if action_frame.space_id != NATIVE_ACTION_SPACE_ID:
            raise ValueError(f"unexpected runtime action space_id {action_frame.space_id!r}")
        if len(action_frame.values) != 7:
            raise ValueError(f"expected 7 runtime action values, got {len(action_frame.values)}")
        return [float(value) for value in action_frame.values]

    def _render_if_enabled(self) -> None:
        if not self.config.visualize:
            return
        render = getattr(self._backend, "render", None)
        if callable(render):
            render()

    def score(self) -> ScoreOutput:
        success = bool(self._last_success)
        return ScoreOutput(
            episode_id=self._episode_id,
            success=success,
            score=1.0 if success else float(self._last_reward),
            reason="LIBERO-PRO success" if success else "success not observed",
            metrics={
                "reward": self._last_reward,
                "done": self._last_done,
                "steps": self._sequence,
                "benchmark_name": self.config.benchmark_name,
                "task_name": self._backend.task_name,
                "language": self._backend.language,
                "init_state_index": self.config.init_state_index,
            },
        )

    def payload_bytes(self, payload_id: str) -> bytes:
        try:
            return self._payloads[payload_id]
        except KeyError as exc:
            raise FileNotFoundError(payload_id) from exc

    def _motor_state(self) -> MotorStateFrame:
        q = [
            *_float_list(self._last_obs.get("robot0_joint_pos", []))[:7],
            _mean_or_zero(_float_list(self._last_obs.get("robot0_gripper_qpos", []))),
        ]
        dq = [
            *_float_list(self._last_obs.get("robot0_joint_vel", []))[:7],
            _mean_or_zero(_float_list(self._last_obs.get("robot0_gripper_qvel", []))),
        ]
        q = (q + [0.0] * len(self.motor_names))[: len(self.motor_names)]
        dq = (dq + [0.0] * len(self.motor_names))[: len(self.motor_names)]
        return MotorStateFrame(
            robot_id=self.config.robot_id,
            names=self.motor_names,
            q=q,
            dq=dq,
            tau=[0.0] * len(self.motor_names),
            sequence=self._sequence,
            timestamp_s=time.time(),
        )

    def _observations(self, tick_id: int) -> list[ObservationFrame]:
        frames = [
            ObservationFrame(
                stream="robot_state",
                kind=ObservationKind.STATE,
                inline_text=f"tick={tick_id} reward={self._last_reward}",
                metadata={"sequence": self._sequence, "state": self._policy_robot_state()},
            )
        ]
        for camera_name in self.config.camera_names:
            image = self._last_obs.get(f"{camera_name}_image")
            if image is None:
                continue
            payload_id, payload = self._store_payload(camera_name, tick_id, image)
            frames.append(
                ObservationFrame(
                    stream=camera_name,
                    kind=ObservationKind.IMAGE,
                    encoding="npy",
                    shape=_shape_list(image),
                    dtype=str(getattr(image, "dtype", "")),
                    data_ref=f"/payloads/{payload_id}",
                    metadata={
                        "sequence": self._sequence,
                        "camera_name": camera_name,
                        "camera_source": "libero_pro_observation",
                        "image_convention": "opengl",
                        "fov_y_deg": 45.0,
                        "payload_bytes": len(payload),
                    },
                )
            )
        return frames

    def _policy_robot_state(self) -> list[float]:
        eef_pos = _padded(_float_list(self._last_obs.get("robot0_eef_pos", [])), 3)
        eef_axis_angle = _quat_to_axis_angle(
            _padded(_float_list(self._last_obs.get("robot0_eef_quat", [])), 4, fill=0.0)
        )
        gripper_qpos = _padded(_float_list(self._last_obs.get("robot0_gripper_qpos", [])), 2)
        return [*eef_pos, *eef_axis_angle, *gripper_qpos]

    def _store_payload(self, stream: str, tick_id: int, value: object) -> tuple[str, bytes]:
        payload_id = f"{stream}-{tick_id:06d}-{self._sequence:06d}.npy"
        payload = _npy_bytes(value)
        self._payloads[payload_id] = payload
        return payload_id, payload


def validate_assets(config: LiberoProRuntimeConfig) -> None:
    if config.allow_asset_bootstrap:
        bootstrap_assets(config)
    if not config.bddl_root.exists():
        raise FileNotFoundError(f"missing LIBERO-PRO BDDL root: {config.bddl_root}")
    if not config.init_states_root.exists():
        raise FileNotFoundError(f"missing LIBERO-PRO init-states root: {config.init_states_root}")
    if not _has_file(config.bddl_root, "*.bddl"):
        raise FileNotFoundError(f"no LIBERO-PRO BDDL files under {config.bddl_root}")
    if not (
        _has_file(config.init_states_root, "*.pt")
        or _has_file(config.init_states_root, "*.pth")
        or _has_file(config.init_states_root, "*.pruned_init")
        or _has_file(config.init_states_root, "*.init")
    ):
        raise FileNotFoundError(f"no LIBERO-PRO init-state tensors under {config.init_states_root}")


def bootstrap_assets(config: LiberoProRuntimeConfig) -> None:
    repo_id = os.environ.get("LIBERO_PRO_HF_REPO_ID")
    if not repo_id:
        raise RuntimeError(
            "LIBERO-PRO asset bootstrap requires LIBERO_PRO_HF_REPO_ID; "
            "prepare assets manually or set the explicit Hugging Face repo id"
        )
    try:
        from huggingface_hub import snapshot_download
    except ImportError as exc:
        raise RuntimeError("install huggingface_hub to use LIBERO-PRO asset bootstrap") from exc
    local_dir = config.bddl_root.parent
    snapshot_download(
        repo_id=repo_id,
        repo_type="dataset",
        local_dir=str(local_dir),
        local_dir_use_symlinks=False,
        allow_patterns=["bddl_files/**", "init_files/**"],
    )


def discover_libero_asset_roots() -> tuple[Path, Path] | None:
    """Return standard LIBERO package bddl/init roots when LIBERO is installed."""

    spec = importlib.util.find_spec("libero.libero")
    if spec is None:
        return None
    locations = spec.submodule_search_locations
    if locations:
        package_root = Path(next(iter(locations)))
    elif spec.origin:
        package_root = Path(spec.origin).parent
    else:
        return None
    bddl_root = package_root / "bddl_files"
    init_states_root = package_root / "init_files"
    if bddl_root.exists() and init_states_root.exists():
        return bddl_root, init_states_root
    return None


def ensure_libero_config(bddl_root: Path, init_states_root: Path) -> None:
    """Create LIBERO's config.yaml noninteractively when it is missing.

    The upstream LIBERO package prompts on first import if the config file is
    absent. Sidecar runs must be noninteractive, so we create the same default
    shape using the roots selected by the benchmark runner or CLI.
    """

    config_root = Path(os.environ.get("LIBERO_CONFIG_PATH", Path.home() / ".libero"))
    config_file = config_root / "config.yaml"
    if config_file.exists():
        return
    benchmark_root = bddl_root.parent
    config_root.mkdir(parents=True, exist_ok=True)
    config_file.write_text(
        "\n".join(
            [
                f"assets: {benchmark_root / 'assets'}",
                f"bddl_files: {bddl_root}",
                f"benchmark_root: {benchmark_root}",
                f"datasets: {benchmark_root.parent / 'datasets'}",
                f"init_states: {init_states_root}",
                "",
            ]
        )
    )


def _resolve_asset_roots(bddl_root: Path | None, init_states_root: Path | None) -> tuple[Path, Path]:
    if bddl_root is not None and init_states_root is not None:
        return bddl_root, init_states_root
    discovered = discover_libero_asset_roots()
    if discovered is None:
        missing = []
        if bddl_root is None:
            missing.append("--bddl-root")
        if init_states_root is None:
            missing.append("--init-states-root")
        raise SystemExit(
            "LIBERO asset roots were not provided and could not be discovered from an "
            f"installed libero package; pass {' and '.join(missing)} or install libero"
        )
    discovered_bddl, discovered_init = discovered
    return bddl_root or discovered_bddl, init_states_root or discovered_init


def _task_bddl_file(config: LiberoProRuntimeConfig, benchmark: object, task: object) -> Path:
    get_path = getattr(benchmark, "get_task_bddl_file_path", None)
    if callable(get_path):
        path = Path(str(get_path(config.task_index)))
        return path if path.is_absolute() else config.bddl_root / path
    bddl_file = getattr(task, "bddl_file", None) or getattr(task, "bddl_file_name", None)
    if bddl_file is not None:
        path = Path(str(bddl_file))
        return path if path.is_absolute() else config.bddl_root / path
    return config.bddl_root / f"{getattr(task, 'name', f'task_{config.task_index}')}.bddl"


def _env_controller(config: LiberoProRuntimeConfig) -> str:
    if config.action_mode == "native":
        return NATIVE_LIBERO_CONTROLLER
    return config.controller


def _reset_settle_steps(config: LiberoProRuntimeConfig) -> int:
    return NATIVE_RESET_SETTLE_STEPS if config.action_mode == "native" else 0


def _set_controller_use_delta(env: object) -> None:
    for robot in getattr(env, "robots", ()) or ():
        controller = getattr(robot, "controller", None)
        if controller is not None and hasattr(controller, "use_delta"):
            controller.use_delta = True


def _load_init_states(
    config: LiberoProRuntimeConfig, benchmark: object, task: object
) -> Sequence[object]:
    task_init_states_file = getattr(task, "init_states_file", None)
    problem_folder = getattr(task, "problem_folder", None)
    if task_init_states_file is not None:
        path = Path(str(task_init_states_file))
        if not path.is_absolute():
            path = config.init_states_root / str(problem_folder or "") / path
        if path.exists():
            return _torch_load_init_states(path)
    get_states = getattr(benchmark, "get_task_init_states", None)
    if callable(get_states):
        try:
            return cast("Sequence[object]", get_states(config.task_index))
        except Exception:
            if task_init_states_file is None:
                raise

    files = (
        sorted(config.init_states_root.rglob("*.pt"))
        + sorted(config.init_states_root.rglob("*.pth"))
        + sorted(config.init_states_root.rglob("*.pruned_init"))
        + sorted(config.init_states_root.rglob("*.init"))
    )
    if not files:
        raise FileNotFoundError(f"no LIBERO-PRO init-state tensors under {config.init_states_root}")
    return _torch_load_init_states(files[0])


def _torch_load_init_states(path: Path) -> Sequence[object]:
    import torch

    try:
        states = torch.load(path, map_location="cpu", weights_only=False)
    except TypeError:
        states = torch.load(path, map_location="cpu")
    return cast("Sequence[object]", states)


def _has_file(root: Path, pattern: str) -> bool:
    return any(root.glob(pattern)) or any(root.rglob(pattern))


def _action_bounds(env: LiberoEnv) -> tuple[list[float], list[float]]:
    action_spec = getattr(env, "action_spec", None)
    if action_spec is None:
        wrapped_env = getattr(env, "env", None)
        action_spec = getattr(wrapped_env, "action_spec", None)
    if action_spec is None:
        raise AttributeError("LIBERO-PRO environment does not expose action_spec")
    low, high = action_spec
    return _float_list(low), _float_list(high)


def _float_list(value: object) -> list[float]:
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        value = tolist()
    if isinstance(value, Sequence) and not isinstance(value, str | bytes):
        return [float(item) for item in value]
    return []


def _mean_or_zero(values: Sequence[float]) -> float:
    return float(sum(values) / len(values)) if values else 0.0


def _padded(values: Sequence[float], size: int, *, fill: float = 0.0) -> list[float]:
    return [*values, *([fill] * size)][:size]


def _quat_to_axis_angle(quat: Sequence[float]) -> list[float]:
    import math

    if len(quat) != 4 or not any(quat):
        return [0.0, 0.0, 0.0]
    x, y, z, w = quat
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return [0.0, 0.0, 0.0]
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    w = min(1.0, max(-1.0, w))
    angle = 2.0 * math.acos(w)
    scale = math.sqrt(max(0.0, 1.0 - w * w))
    if scale < 1e-8:
        return [0.0, 0.0, 0.0]
    return [x / scale * angle, y / scale * angle, z / scale * angle]


def _shape_list(value: object) -> list[int]:
    shape = getattr(value, "shape", None)
    if isinstance(shape, Sequence):
        return [int(item) for item in shape]
    return []


def _npy_bytes(value: object) -> bytes:
    import numpy as np

    buffer = BytesIO()
    np.save(buffer, np.asarray(value), allow_pickle=False)
    return buffer.getvalue()


def _success_from_info(info: dict[str, object]) -> bool | None:
    for key in ("success", "is_success", "task_success"):
        value = info.get(key)
        if isinstance(value, bool):
            return value
        if isinstance(value, int | float):
            return bool(value)
    return None
