#!/usr/bin/env python3
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

"""Run LeRobot LIBERO policy rollout gates through DimOS seams.

The default run evaluates the official `lerobot/VLA-JEPA-LIBERO` checkpoint over
the 50-episode LIBERO object matrix: task indices 0..9 crossed with init states
0..4. The pass condition is strict: success_rate > 0.50.

Pass `--policy-family xvla --episodes-limit 10` to run the X-VLA LIBERO gate
configuration with `lerobot/xvla-libero`; run once without `--live-policy-stream`
for the synchronous benchmark stage, then again with `--live-policy-stream` for
the module-native live stage.

For a lightweight native-action smoke test that does not download LeRobot, pass
`--fake-backend --episodes-limit 1 --no-enforce-gate`.
"""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
from dataclasses import asdict, is_dataclass
import json
from pathlib import Path
import sys
import time
import traceback
from typing import Protocol
from uuid import uuid4

REPO_ROOT = Path(__file__).resolve().parents[2]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
LIBERO_PRO_SIDECAR_SRC = REPO_ROOT / "packages" / "dimos-libero-pro-sidecar" / "src"

for package_src in (PROTOCOL_SRC, LIBERO_PRO_SIDECAR_SRC):
    sys.path.insert(0, str(package_src))

from dimos_libero_pro_sidecar.blueprint import libero_pro_runtime_blueprint
from dimos_libero_pro_sidecar.module import LiberoProRuntimeModule
from dimos_runtime_protocol import (
    EpisodeResetRequest,
    EpisodeResetResponse,
    ObservationFrame,
    RuntimeActionFrame,
    StepRequest,
    StepResponse,
)
import imageio.v2 as imageio
import numpy as np

from dimos.benchmark.runtime.artifacts import write_json
from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot_learning.policy_rollout.backends.backend import PolicyBackend
from dimos.robot_learning.policy_rollout.evaluation import (
    BenchmarkEpisodeResult,
    BenchmarkEpisodeSpec,
    BenchmarkEvaluationSummary,
    BenchmarkPolicyEvalModule,
    LiberoRobotPolicyObservationBuilder,
    PolicyEvalRuntimeSession,
    RuntimeStreamSnapshot,
    libero_object_episode_matrix,
)
from dimos.robot_learning.policy_rollout.models import (
    BackendBatch,
    BackendOutputEnvelope,
    PolicyBackendDescription,
    RobotPolicyObservation,
)
from dimos.robot_learning.policy_rollout.robot_policy_module import RobotPolicyModule
from dimos.robot_learning.policy_rollout.vla_jepa_libero_contract import (
    VLA_JEPA_LIBERO_ACTION_SPACE_ID,
)
from dimos.robot_learning.policy_rollout.xvla_libero_contract import (
    XVLA_LIBERO_ABSOLUTE_ACTION_SPACE_ID,
)
from dimos.simulation.runtime_client.shm_motor import MotorShmOwner

DEFAULT_CAMERAS = ("agentview", "robot0_eye_in_hand")
DEFAULT_ARTIFACT_DIR = REPO_ROOT / "artifacts" / "benchmark" / "lerobot-vla-jepa-libero"
DEFAULT_POLICY_FAMILY = "vla_jepa"
POLICY_FAMILY_DEFAULTS: Mapping[str, Mapping[str, object]] = {
    "vla_jepa": {
        "checkpoint": "lerobot/VLA-JEPA-LIBERO",
        "contract_type": "vla_jepa_libero",
        "action_space_id": VLA_JEPA_LIBERO_ACTION_SPACE_ID,
        "runtime_action_mode": "native",
        "expected_benchmark_episodes": 50,
        "expected_live_episodes": 10,
    },
    "xvla": {
        "checkpoint": "lerobot/xvla-libero",
        "contract_type": "xvla_libero",
        "action_space_id": XVLA_LIBERO_ABSOLUTE_ACTION_SPACE_ID,
        "runtime_action_mode": "native_absolute",
        "expected_benchmark_episodes": 10,
        "expected_live_episodes": 10,
        "documented_control_mode": "absolute",
    },
}


class DescribedPolicyModule(Protocol):
    def describe_backend(self) -> PolicyBackendDescription: ...

    def close(self) -> None: ...


class _LiberoProRuntime(Protocol):
    def reset(self, request: EpisodeResetRequest) -> EpisodeResetResponse: ...

    def step(self, request: StepRequest) -> StepResponse: ...

    def observation_snapshot(self) -> tuple[list[ObservationFrame], dict[str, np.ndarray]]: ...

    def stop(self) -> None: ...


class FixedActionBackend:
    """PolicyBackend test double that exercises contract conversion without LeRobot."""

    def __init__(self, action: Sequence[float], *, use_action_chunk: bool = False) -> None:
        if len(action) != 7:
            raise ValueError("fixed action must have exactly 7 values")
        self._action = tuple(float(value) for value in action)
        self._use_action_chunk = use_action_chunk
        self._initialized = False
        self._episode_resets = 0

    def initialize(self) -> None:
        self._initialized = True

    def reset_episode(self) -> None:
        self._episode_resets += 1

    def infer_batch(self, batch: BackendBatch) -> BackendOutputEnvelope:
        if not self._initialized:
            raise RuntimeError("FixedActionBackend was not initialized")
        output: tuple[float, ...] | tuple[tuple[float, ...], ...] = self._action
        if self._use_action_chunk:
            output = (self._action,)
        return BackendOutputEnvelope(
            output=output,
            metadata={
                "backend_type": "fixed_action",
                "batch_metadata": dict(batch.metadata),
                "episode_resets": self._episode_resets,
                "use_action_chunk": self._use_action_chunk,
            },
        )

    def close(self) -> None:
        self._initialized = False

    def describe(self) -> PolicyBackendDescription:
        return PolicyBackendDescription(
            backend_type="fixed_action",
            checkpoint_id=None,
            supports_episode_reset=True,
            metadata={"action": list(self._action), "episode_resets": self._episode_resets},
        )


class ModuleLiberoRuntimeSession(PolicyEvalRuntimeSession):
    """Policy-evaluation runtime session backed by the placed LIBERO runtime module."""

    def __init__(
        self,
        runtime: _LiberoProRuntime,
        coordinator: ModuleCoordinator,
        expected_image_streams: Sequence[str],
        stream_timeout_s: float = 5.0,
    ) -> None:
        self._runtime = runtime
        self._coordinator = coordinator
        self._expected_image_streams = tuple(expected_image_streams)
        self._stream_timeout_s = stream_timeout_s
        self._last_snapshot = RuntimeStreamSnapshot()

    def reset(self, request: EpisodeResetRequest) -> EpisodeResetResponse:
        response = self._runtime.reset(request)
        self._last_snapshot = self._take_snapshot()
        return response

    def step(self, request: StepRequest) -> StepResponse:
        response = self._runtime.step(request)
        self._last_snapshot = self._take_snapshot()
        return response

    def latest_observation_snapshot(self) -> RuntimeStreamSnapshot:
        return self._last_snapshot

    def close(self) -> None:
        self._coordinator.stop()

    def _take_snapshot(self) -> RuntimeStreamSnapshot:
        observations, stream_values = self._runtime.observation_snapshot()
        missing = [stream for stream in self._expected_image_streams if stream not in stream_values]
        if missing:
            raise RuntimeError(
                f"runtime module did not publish expected image streams {missing} "
                f"within {self._stream_timeout_s:.1f}s"
            )
        return RuntimeStreamSnapshot(observations=tuple(observations), values=stream_values)


class _LivePolicyCoordinator:
    """Module-native live policy path with stream/RPC policy-to-control wiring."""

    def __init__(self, *, args: argparse.Namespace) -> None:
        self._args = args
        self._action_names = [f"policy_action/{index}" for index in range(7)]
        self._owner = MotorShmOwner(f"policy_action_{uuid4().hex}", self._action_names)
        self._component = HardwareComponent(
            hardware_id="policy_action",
            hardware_type=HardwareType.WHOLE_BODY,
            joints=self._action_names,
            adapter_type="benchmark_runtime",
            address=self._owner.key,
            auto_enable=False,
            adapter_kwargs={
                "motor_names": self._action_names,
                "connect_timeout_s": 5.0,
            },
        )
        self._task_name = "policy_chunk_action"
        self._module_coordinator: ModuleCoordinator | None = None
        self._policy_module: RobotPolicyModule | None = None
        self._control_coordinator: ControlCoordinator | None = None
        self._policy_observation_transport: object | None = None
        self._last_command_sequence = 0
        self.chunk_count = 0
        self.consumed_actions = 0
        self.stale_deactivations = 0

    @property
    def last_command_sequence(self) -> int:
        return self._last_command_sequence

    def start(self) -> None:
        policy_blueprint = self._policy_blueprint()
        control_blueprint = ControlCoordinator.blueprint(
            tick_rate=float(self._args.control_step_hz),
            publish_joint_state=False,
            hardware=[self._component],
            tasks=[
                TaskConfig(
                    name=self._task_name,
                    type="policy_chunk",
                    joint_names=self._action_names,
                    auto_start=True,
                    params={
                        "accepted_action_space_id": _action_space_id(self._args),
                        "ticks_per_action": 1,
                        "execute_first_n": 1,
                        "stale_timeout_ticks": 1,
                        "action_mapping": "target",
                        "policy_trigger_method": "trigger_policy_action_chunk_inference",
                    },
                )
            ],
        )
        blueprint = (
            autoconnect(policy_blueprint, control_blueprint)
            .remappings(
                [
                    (
                        RobotPolicyModule,
                        "policy_action_chunk",
                        "robot_policy_action_chunk",
                    )
                ]
            )
            .global_config(viewer="none", n_workers=1)
        )
        self._module_coordinator = ModuleCoordinator.build(blueprint)
        self._policy_module = self._module_coordinator.get_instance(RobotPolicyModule)
        self._control_coordinator = self._module_coordinator.get_instance(ControlCoordinator)
        if self._policy_module is None or self._control_coordinator is None:
            raise RuntimeError(
                "module-native live policy blueprint did not deploy required modules"
            )
        self._policy_observation_transport = self._module_coordinator._transport_registry[
            ("policy_observation", RobotPolicyObservation)
        ]

    def _policy_blueprint(self):  # type: ignore[no-untyped-def]
        if self._args.fake_backend:
            return RobotPolicyModule.blueprint(
                backend_type="fixed_action",
                backend_params={
                    "action": _fixed_action_values(self._args.fixed_action),
                    "use_action_chunk": True,
                },
                contract_type=_contract_type(self._args),
                contract_params={},
            )
        return RobotPolicyModule.blueprint(
            backend_type="lerobot",
            backend_params={
                "policy_family": _policy_family(self._args),
                "checkpoint_id": _checkpoint(self._args),
                "device": self._args.device,
                "use_action_chunk": True,
            },
            contract_type=_contract_type(self._args),
            contract_params={},
        )

    def reset_episode(self, episode_id: str) -> None:
        if self._policy_module is None:
            raise RuntimeError("live policy modules have not been started")
        self._policy_module.reset(episode_id=episode_id)

    def update_observation(self, sample: RobotPolicyObservation) -> None:
        if self._policy_observation_transport is None:
            raise RuntimeError("live policy modules have not been started")
        publish = getattr(self._policy_observation_transport, "publish", None)
        if not callable(publish):
            raise RuntimeError("policy observation transport does not support publish")
        publish(sample)

    def describe_backend(self) -> PolicyBackendDescription:
        if self._policy_module is None:
            policy_module = _policy_module(self._args)
            try:
                return policy_module.describe_backend()
            finally:
                policy_module.close()
        return self._policy_module.describe_backend()

    def diagnostics(self) -> dict[str, object]:
        if self._control_coordinator is None:
            return {}
        diagnostics = self._control_coordinator.task_invoke(self._task_name, "diagnostics")
        if not isinstance(diagnostics, dict):
            return {}
        return {str(key): value for key, value in diagnostics.items()}

    def next_action_values(self, *, timeout_s: float) -> tuple[float, ...] | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            sequence, commands = self._owner.read_commands()
            if sequence > self._last_command_sequence:
                self._last_command_sequence = sequence
                self.consumed_actions += 1
                return tuple(float(command.q) for command in commands)
            time.sleep(0.001)
        self.stale_deactivations += 1
        return None

    def close(self) -> None:
        if self._module_coordinator is not None:
            self._module_coordinator.stop()
            self._module_coordinator = None
            self._policy_module = None
            self._control_coordinator = None
            self._policy_observation_transport = None
        self._owner.close()
        self._owner.unlink()


def run_policy_gate(args: argparse.Namespace) -> BenchmarkEvaluationSummary:
    artifact_dir = _repo_path(args.artifact_dir)
    artifact_dir.mkdir(parents=True, exist_ok=True)
    episodes = _selected_episodes(args.episodes_limit)
    policy_module = None if args.live_policy_stream else _policy_module(args)
    live: _LivePolicyCoordinator | None = None
    results: list[BenchmarkEpisodeResult] = []
    cleanup: dict[str, object] = {"episodes": []}
    try:
        if args.live_policy_stream:
            live = _LivePolicyCoordinator(args=args)
            live.start()
        for episode in episodes:
            episode_dir = artifact_dir / "episodes" / episode.episode_id
            if live is not None:
                result = _run_one_live_episode(args, episode, live, episode_dir, cleanup)
            else:
                if policy_module is None:
                    raise RuntimeError("synchronous policy module was not initialized")
                result = _run_one_episode(args, episode, policy_module, episode_dir, cleanup)
            results.append(result)
        summary = _summary(results, args.success_threshold)
        _write_aggregate_artifacts(
            artifact_dir,
            results,
            summary,
            live if live is not None else policy_module,
            args,
            cleanup,
        )
        if args.enforce_gate and not summary.passed:
            raise SystemExit(
                f"LeRobot LIBERO policy gate failed: "
                f"success_rate={summary.success_rate:.3f} <= {summary.success_threshold:.3f}"
            )
        return summary
    except Exception as exc:
        _write_setup_failure_artifacts(artifact_dir, live or policy_module, args, cleanup, exc)
        raise
    finally:
        if live is not None:
            live.close()
        if policy_module is not None:
            policy_module.close()


def _run_one_episode(
    args: argparse.Namespace,
    episode: BenchmarkEpisodeSpec,
    policy_module: RobotPolicyModule,
    episode_dir: Path,
    cleanup: dict[str, object],
) -> BenchmarkEpisodeResult:
    client = _start_runtime(args, episode)
    cleanup_entry: dict[str, object] = {
        "episode_id": episode.episode_id,
        "runtime": "module-native",
    }
    cast_cleanup = cleanup.setdefault("episodes", [])
    if isinstance(cast_cleanup, list):
        cast_cleanup.append(cleanup_entry)
    try:
        eval_module = BenchmarkPolicyEvalModule(
            runtime_session=client,
            robot_policy_module=policy_module,
            artifact_dir=str(episode_dir),
            max_steps=args.max_steps,
            success_threshold=args.success_threshold,
            close_policy_on_finish=False,
            video_dir=str(episode_dir / "videos") if args.save_videos else None,
            video_streams=args.camera_names if args.save_videos else (),
            video_fps=args.control_step_hz,
        )
        try:
            eval_module.run_episodes([episode])
            if not eval_module.last_results:
                raise RuntimeError(f"episode {episode.episode_id} produced no result")
            return eval_module.last_results[0]
        finally:
            eval_module.close()
    finally:
        _stop_runtime(client, episode_dir, cleanup_entry)


def _run_one_live_episode(
    args: argparse.Namespace,
    episode: BenchmarkEpisodeSpec,
    live: _LivePolicyCoordinator,
    episode_dir: Path,
    cleanup: dict[str, object],
) -> BenchmarkEpisodeResult:
    client = _start_runtime(args, episode)
    cleanup_entry: dict[str, object] = {
        "episode_id": episode.episode_id,
        "runtime": "module-native-live-policy-stream",
    }
    cast_cleanup = cleanup.setdefault("episodes", [])
    if isinstance(cast_cleanup, list):
        cast_cleanup.append(cleanup_entry)
    diagnostics: dict[str, object] = {}
    try:
        result, diagnostics = _run_live_episode_loop(
            args,
            episode,
            client,
            live,
            episode_dir,
        )
        episode_dir.mkdir(parents=True, exist_ok=True)
        write_json(episode_dir / "live_path_diagnostics.json", _json_ready(diagnostics))
        return result
    finally:
        cleanup_entry["live_path_diagnostics"] = diagnostics
        _stop_runtime(client, episode_dir, cleanup_entry)


def _run_live_episode_loop(
    args: argparse.Namespace,
    episode: BenchmarkEpisodeSpec,
    client: PolicyEvalRuntimeSession,
    live: _LivePolicyCoordinator,
    episode_dir: Path,
) -> tuple[BenchmarkEpisodeResult, dict[str, object]]:
    reset_response = client.reset(
        EpisodeResetRequest(
            episode_id=episode.episode_id,
            task_id=episode.task_id,
            seed=episode.seed,
            options={
                **dict(episode.options),
                "task_index": episode.task_index,
                "init_state_index": episode.init_state_index,
            },
        )
    )
    live.reset_episode(episode.episode_id)
    runtime_description = reset_response.runtime_description
    episode_dir.mkdir(parents=True, exist_ok=True)
    write_json(episode_dir / "runtime_description.json", _json_ready(runtime_description))
    sample_builder = LiberoRobotPolicyObservationBuilder()
    snapshot = client.latest_observation_snapshot()
    observations = snapshot.observations
    reward_sum = 0.0
    done = False
    success: bool | None = None
    last_values: tuple[float, ...] = ()
    observed_streams = snapshot.observed_streams
    video_streams = tuple(args.camera_names) if args.save_videos else ()
    video_frames: dict[str, list[np.ndarray]] = {stream: [] for stream in video_streams}
    steps = 0
    stale_waits = 0
    consecutive_stale_waits = 0

    for tick_id in range(args.max_steps):
        _append_video_frames(video_frames, snapshot.values)
        sample = sample_builder.build(
            episode=episode,
            tick_id=tick_id,
            observations=observations,
            runtime_description=runtime_description,
            reward=reward_sum,
            done=done,
            success=success,
            observation_values=snapshot.values,
        )
        live.update_observation(sample)
        command_values = live.next_action_values(timeout_s=float(args.live_chunk_timeout_s))
        if command_values is None:
            stale_waits += 1
            consecutive_stale_waits += 1
            if consecutive_stale_waits >= int(args.live_max_stale_waits):
                break
            continue
        consecutive_stale_waits = 0
        last_values = command_values
        step_response = client.step(
            StepRequest(
                episode_id=episode.episode_id,
                tick_id=tick_id,
                action=RuntimeActionFrame(
                    frame_type="runtime_action",
                    space_id=_action_space_id(args),
                    values=list(command_values),
                    tick_id=tick_id,
                ),
            )
        )
        steps = tick_id + 1
        reward_sum += step_response.reward
        done = step_response.done
        success = step_response.success
        snapshot = client.latest_observation_snapshot()
        observations = snapshot.observations
        observed_streams = snapshot.observed_streams
        if done:
            break

    _write_live_videos(
        episode_dir,
        episode.episode_id,
        video_frames,
        fps=int(args.control_step_hz),
    )
    episode_success = bool(success)
    failure_reason = None
    if not episode_success:
        if stale_waits >= int(args.live_max_stale_waits) and not last_values:
            failure_reason = "live_action_timeout"
        else:
            failure_reason = "done_without_success" if done else "max_steps_without_success"
    task_diagnostics = live.diagnostics()
    inference_status_counts = task_diagnostics.get("inference_status_counts", {})
    if not isinstance(inference_status_counts, dict):
        inference_status_counts = {}
    diagnostics: dict[str, object] = {
        "chunk_count": _int_diagnostic(task_diagnostics, "accepted_chunks", live.chunk_count),
        "consumed_actions": _int_diagnostic(
            task_diagnostics, "consumed_actions", live.consumed_actions
        ),
        "refill_triggers": _int_diagnostic(task_diagnostics, "refill_triggers", 0),
        "inference_status_counts": {
            str(key): int(value)
            for key, value in inference_status_counts.items()
            if isinstance(value, int)
        },
        "stale_waits": stale_waits,
        "live_max_stale_waits": int(args.live_max_stale_waits),
        "stale_deactivations": _int_diagnostic(
            task_diagnostics, "stale_deactivations", live.stale_deactivations
        ),
        "last_command_sequence": live.last_command_sequence,
    }
    return BenchmarkEpisodeResult(
        episode_id=episode.episode_id,
        task_id=episode.task_id,
        task_index=episode.task_index,
        init_state_index=episode.init_state_index,
        success=episode_success,
        steps=steps,
        reward_sum=reward_sum,
        done=done,
        failure_reason=failure_reason,
        action_shape=(len(last_values),),
        action_min=min(last_values) if last_values else None,
        action_max=max(last_values) if last_values else None,
        observed_streams=observed_streams,
    ), diagnostics


def _int_diagnostic(diagnostics: Mapping[str, object], key: str, default: int) -> int:
    value = diagnostics.get(key)
    return value if isinstance(value, int) else default


def _append_video_frames(
    video_frames: dict[str, list[np.ndarray]],
    stream_values: Mapping[str, object],
) -> None:
    for stream, frames in video_frames.items():
        value = stream_values.get(stream)
        if not isinstance(value, np.ndarray):
            continue
        if value.ndim != 3 or value.shape[2] != 3:
            continue
        frames.append(value.astype(np.uint8, copy=False))


def _write_live_videos(
    episode_dir: Path,
    episode_id: str,
    video_frames: dict[str, list[np.ndarray]],
    *,
    fps: int,
) -> None:
    if fps <= 0:
        raise ValueError("video fps must be positive")
    for stream, frames in video_frames.items():
        if not frames:
            continue
        stream_name = stream.replace("/", "_")
        video_path = episode_dir / "videos" / episode_id / f"{stream_name}.mp4"
        video_path.parent.mkdir(parents=True, exist_ok=True)
        imageio.mimsave(video_path, frames, fps=fps)  # type: ignore[arg-type]


def _selected_episodes(limit: int | None) -> list[BenchmarkEpisodeSpec]:
    episodes = libero_object_episode_matrix()
    return episodes if limit is None else episodes[:limit]


def _policy_module(args: argparse.Namespace) -> RobotPolicyModule:
    backend: PolicyBackend
    if args.fake_backend:
        backend = FixedActionBackend(
            _fixed_action_values(args.fixed_action),
            use_action_chunk=bool(args.live_policy_stream),
        )
        return RobotPolicyModule(
            backend=backend,
            contract_type=_contract_type(args),
            contract_params={},
        )
    else:
        return RobotPolicyModule(
            backend_type="lerobot",
            backend_params={
                "policy_family": _policy_family(args),
                "checkpoint_id": _checkpoint(args),
                "device": args.device,
                "use_action_chunk": bool(args.live_policy_stream),
            },
            contract_type=_contract_type(args),
            contract_params={},
        )


def _start_runtime(
    args: argparse.Namespace, episode: BenchmarkEpisodeSpec
) -> ModuleLiberoRuntimeSession:
    blueprint = libero_pro_runtime_blueprint(
        bddl_root=args.bddl_root,
        init_states_root=args.init_states_root,
        benchmark_name="libero_object",
        robot_id="panda",
        task_order_index=0,
        task_index=episode.task_index,
        init_state_index=episode.init_state_index,
        action_mode=_runtime_action_mode(args),
        camera_names=tuple(args.camera_names),
        camera_height=args.camera_height,
        camera_width=args.camera_width,
        control_freq=args.control_step_hz,
        horizon=args.max_steps,
        seed=args.seed,
        allow_asset_bootstrap=args.allow_asset_bootstrap,
        visualize=args.visualize,
    ).global_config(viewer="none", n_workers=1)
    coordinator = ModuleCoordinator.build(blueprint)
    runtime = coordinator.get_instance(LiberoProRuntimeModule)
    return ModuleLiberoRuntimeSession(
        runtime, coordinator, expected_image_streams=args.camera_names
    )


def _stop_runtime(
    client: ModuleLiberoRuntimeSession, episode_dir: Path, cleanup_entry: dict[str, object]
) -> None:
    client.close()
    episode_dir.mkdir(parents=True, exist_ok=True)
    cleanup_entry["runtime_stopped"] = True


def _summary(
    results: Sequence[BenchmarkEpisodeResult], success_threshold: float
) -> BenchmarkEvaluationSummary:
    successes = sum(1 for result in results if result.success)
    success_rate = successes / len(results) if results else 0.0
    return BenchmarkEvaluationSummary(
        episodes=len(results),
        successes=successes,
        success_rate=success_rate,
        success_threshold=success_threshold,
        passed=success_rate > success_threshold,
    )


def _write_aggregate_artifacts(
    artifact_dir: Path,
    results: Sequence[BenchmarkEpisodeResult],
    summary: BenchmarkEvaluationSummary,
    policy_module: DescribedPolicyModule,
    args: argparse.Namespace,
    cleanup: dict[str, object],
) -> None:
    write_json(artifact_dir / "summary.json", _json_ready(summary))
    _write_jsonl(artifact_dir / "episodes.jsonl", results)
    write_json(
        artifact_dir / "checkpoint_metadata.json", _json_ready(policy_module.describe_backend())
    )
    if results:
        _copy_if_present(
            artifact_dir / "episodes" / results[0].episode_id / "runtime_description.json",
            artifact_dir / "runtime_description.json",
        )
    write_json(
        artifact_dir / "run_config.json",
        {
            "policy_family": _policy_family(args),
            "checkpoint": _checkpoint(args),
            "contract_type": _contract_type(args),
            "action_space_id": _action_space_id(args),
            "runtime_action_mode": _runtime_action_mode(args),
            "gate_stage": _gate_stage(args),
            "documented_control_mode": _documented_control_mode(args),
            "fake_backend": args.fake_backend,
            "live_policy_stream": args.live_policy_stream,
            "live_chunk_timeout_s": args.live_chunk_timeout_s,
            "live_max_stale_waits": args.live_max_stale_waits,
            "episodes": len(results),
            "expected_gate_episodes": _expected_gate_episodes(args),
            "success_threshold": args.success_threshold,
            "enforce_gate": args.enforce_gate,
            "save_videos": args.save_videos,
            "video_streams": list(args.camera_names) if args.save_videos else [],
            "camera_names": list(args.camera_names),
            "camera_height": args.camera_height,
            "camera_width": args.camera_width,
            "max_steps": args.max_steps,
            "bddl_root": str(args.bddl_root),
            "init_states_root": str(args.init_states_root),
        },
    )
    write_json(artifact_dir / "cleanup_status.json", cleanup)


def _write_setup_failure_artifacts(
    artifact_dir: Path,
    policy_module: DescribedPolicyModule | None,
    args: argparse.Namespace,
    cleanup: dict[str, object],
    exc: Exception,
) -> None:
    close_policy_module = False
    if policy_module is None:
        policy_module = _policy_module(args)
        close_policy_module = True
    write_json(
        artifact_dir / "setup_error.json",
        {
            "error_type": type(exc).__name__,
            "message": str(exc),
            "traceback": traceback.format_exc(),
            "gate_verified": False,
            "policy_executed": False,
            "policy_family": _policy_family(args),
            "checkpoint": _checkpoint(args),
            "contract_type": _contract_type(args),
            "action_space_id": _action_space_id(args),
            "runtime_action_mode": _runtime_action_mode(args),
            "gate_stage": _gate_stage(args),
            "expected_gate_episodes": _expected_gate_episodes(args),
            "success_threshold": args.success_threshold,
            "hint": "Check per-episode sidecar logs and prepare LIBERO assets before rerunning.",
        },
    )
    write_json(
        artifact_dir / "checkpoint_metadata.json", _json_ready(policy_module.describe_backend())
    )
    write_json(
        artifact_dir / "run_config.json",
        {
            "policy_family": _policy_family(args),
            "checkpoint": _checkpoint(args),
            "contract_type": _contract_type(args),
            "gate_stage": _gate_stage(args),
            "documented_control_mode": _documented_control_mode(args),
            "fake_backend": args.fake_backend,
            "live_policy_stream": args.live_policy_stream,
            "live_chunk_timeout_s": args.live_chunk_timeout_s,
            "live_max_stale_waits": args.live_max_stale_waits,
            "expected_gate_episodes": _expected_gate_episodes(args),
            "success_threshold": args.success_threshold,
            "enforce_gate": args.enforce_gate,
            "save_videos": args.save_videos,
            "video_streams": list(args.camera_names) if args.save_videos else [],
            "camera_names": list(args.camera_names),
            "camera_height": args.camera_height,
            "camera_width": args.camera_width,
            "max_steps": args.max_steps,
            "bddl_root": str(args.bddl_root),
            "init_states_root": str(args.init_states_root),
            "allow_asset_bootstrap": args.allow_asset_bootstrap,
        },
    )
    write_json(artifact_dir / "cleanup_status.json", cleanup)
    if close_policy_module:
        policy_module.close()


def _fixed_action_values(raw: str) -> tuple[float, ...]:
    values = tuple(float(value.strip()) for value in raw.split(",") if value.strip())
    if len(values) != 7:
        raise ValueError("--fixed-action must contain exactly 7 comma-separated values")
    return values


def _policy_family(args: argparse.Namespace) -> str:
    family = str(getattr(args, "policy_family", DEFAULT_POLICY_FAMILY))
    if family not in POLICY_FAMILY_DEFAULTS:
        families = ", ".join(sorted(POLICY_FAMILY_DEFAULTS))
        raise ValueError(f"unsupported policy family {family!r}; expected one of {families}")
    return family


def _policy_defaults(args: argparse.Namespace) -> Mapping[str, object]:
    return POLICY_FAMILY_DEFAULTS[_policy_family(args)]


def _checkpoint(args: argparse.Namespace) -> str:
    checkpoint = getattr(args, "checkpoint", None)
    if checkpoint:
        return str(checkpoint)
    return str(_policy_defaults(args)["checkpoint"])


def _contract_type(args: argparse.Namespace) -> str:
    contract_type = getattr(args, "contract_type", None)
    if contract_type:
        return str(contract_type)
    return str(_policy_defaults(args)["contract_type"])


def _action_space_id(args: argparse.Namespace) -> str:
    return str(_policy_defaults(args)["action_space_id"])


def _runtime_action_mode(args: argparse.Namespace) -> str:
    return str(_policy_defaults(args)["runtime_action_mode"])


def _gate_stage(args: argparse.Namespace) -> str:
    stage = getattr(args, "gate_stage", None)
    if stage:
        return str(stage)
    return "live" if bool(getattr(args, "live_policy_stream", False)) else "benchmark"


def _expected_gate_episodes(args: argparse.Namespace) -> int:
    key = (
        "expected_live_episodes"
        if bool(getattr(args, "live_policy_stream", False))
        else "expected_benchmark_episodes"
    )
    return int(str(_policy_defaults(args)[key]))


def _documented_control_mode(args: argparse.Namespace) -> str:
    mode = _policy_defaults(args).get("documented_control_mode", "native")
    return str(mode)


def _repo_path(path: Path) -> Path:
    return path if path.is_absolute() else REPO_ROOT / path


def _resolve_asset_roots(
    bddl_root: Path | None, init_states_root: Path | None
) -> tuple[Path, Path]:
    explicit_bddl = _repo_path(bddl_root) if bddl_root is not None else None
    explicit_init = _repo_path(init_states_root) if init_states_root is not None else None
    if explicit_bddl is not None and explicit_init is not None:
        return explicit_bddl, explicit_init
    discovered = _runtime_venv_libero_asset_roots()
    if discovered is None:
        raise SystemExit(
            "LIBERO assets were not provided and could not be found in the prepared "
            "LIBERO runtime environment. Run `rtk uv sync` in "
            "packages/dimos-libero-pro-sidecar or pass --bddl-root and --init-states-root."
        )
    discovered_bddl, discovered_init = discovered
    return explicit_bddl or discovered_bddl, explicit_init or discovered_init


def _runtime_venv_libero_asset_roots() -> tuple[Path, Path] | None:
    runtime_project = LIBERO_PRO_SIDECAR_SRC.parent
    site_packages_roots = sorted((runtime_project / ".venv" / "lib").glob("python*/site-packages"))
    for site_packages in site_packages_roots:
        package_root = site_packages / "libero" / "libero"
        bddl_root = package_root / "bddl_files"
        init_states_root = package_root / "init_files"
        if bddl_root.exists() and init_states_root.exists():
            return bddl_root, init_states_root
    return None


def _write_jsonl(path: Path, rows: Sequence[object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(_json_ready(row), sort_keys=True) + "\n" for row in rows))


def _copy_if_present(source: Path, destination: Path) -> None:
    if not source.exists():
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(source.read_bytes())


def _json_ready(value: object) -> object:
    if is_dataclass(value) and not isinstance(value, type):
        return _json_ready(asdict(value))
    model_dump = getattr(value, "model_dump", None)
    if callable(model_dump):
        return model_dump(mode="json")
    if isinstance(value, dict):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_json_ready(item) for item in value]
    if isinstance(value, list):
        return [_json_ready(item) for item in value]
    return value


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact-dir", type=Path, default=DEFAULT_ARTIFACT_DIR)
    parser.add_argument("--max-steps", type=int, default=1000)
    parser.add_argument("--control-step-hz", type=int, default=20)
    parser.add_argument("--success-threshold", type=float, default=0.50)
    parser.add_argument(
        "--policy-family",
        choices=tuple(sorted(POLICY_FAMILY_DEFAULTS)),
        default=DEFAULT_POLICY_FAMILY,
        help="LeRobot policy family to load for real-policy gates.",
    )
    parser.add_argument(
        "--checkpoint",
        default=None,
        help="Policy checkpoint id/path. Defaults to the selected policy family checkpoint.",
    )
    parser.add_argument(
        "--contract-type",
        default=None,
        help="Robot policy contract type. Defaults to the selected policy family contract.",
    )
    parser.add_argument(
        "--gate-stage",
        choices=("benchmark", "live"),
        default=None,
        help="Artifact stage label. Defaults to benchmark or live from --live-policy-stream.",
    )
    parser.add_argument("--device", default=None)
    parser.add_argument(
        "--live-policy-stream",
        action="store_true",
        help="Run real policy chunks through RobotPolicyModule streams and ControlCoordinator.",
    )
    parser.add_argument(
        "--live-chunk-timeout-s",
        type=float,
        default=5.0,
        help="Seconds to wait for each ControlCoordinator-emitted live policy action.",
    )
    parser.add_argument(
        "--live-max-stale-waits",
        type=int,
        default=3,
        help="Abort an episode after this many consecutive live action timeouts.",
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--episodes-limit", type=int, default=None)
    parser.add_argument("--no-enforce-gate", dest="enforce_gate", action="store_false")
    parser.set_defaults(enforce_gate=True)
    parser.add_argument("--fake-backend", action="store_true")
    parser.add_argument("--fixed-action", default="0,0,0,0,0,0,0")
    parser.add_argument("--camera-name", action="append", dest="camera_names")
    parser.add_argument("--camera-height", type=int, default=128)
    parser.add_argument("--camera-width", type=int, default=128)
    parser.add_argument("--bddl-root", type=Path, default=None)
    parser.add_argument("--init-states-root", type=Path, default=None)
    parser.add_argument("--allow-asset-bootstrap", action="store_true")
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument(
        "--save-videos",
        action="store_true",
        help="Reserve video artifacts for real runs; full videos/images are not saved by default.",
    )
    return parser


def main() -> None:
    parser = _parser()
    args = parser.parse_args()
    args.checkpoint = _checkpoint(args)
    args.contract_type = _contract_type(args)
    args.gate_stage = _gate_stage(args)
    args.camera_names = tuple(args.camera_names or DEFAULT_CAMERAS)
    args.bddl_root, args.init_states_root = _resolve_asset_roots(
        args.bddl_root, args.init_states_root
    )
    summary = run_policy_gate(args)
    print(
        f"episodes={summary.episodes} successes={summary.successes} "
        f"success_rate={summary.success_rate:.3f} passed={summary.passed}"
    )


if __name__ == "__main__":
    main()
