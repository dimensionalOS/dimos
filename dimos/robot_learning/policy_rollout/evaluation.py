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

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass, field, is_dataclass
import json
from pathlib import Path
from typing import Protocol

from dimos_runtime_protocol import (
    EpisodeResetRequest,
    EpisodeResetResponse,
    ObservationFrame,
    ObservationKind,
    RuntimeActionFrame,
    RuntimeDescription,
    StepRequest,
    StepResponse,
)
import imageio.v2 as imageio
import numpy as np

from dimos.benchmark.runtime.artifacts import write_json
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.robot_learning.policy_rollout.models import (
    JsonObject,
    RobotPolicyAction,
    RobotPolicyObservation,
)
from dimos.robot_learning.policy_rollout.robot_policy_module import RobotPolicyModule
from dimos.spec.utils import Spec


@dataclass(frozen=True)
class BenchmarkEpisodeSpec:
    """One benchmark episode selected by the evaluation runner."""

    episode_id: str
    task_id: str
    task_index: int
    init_state_index: int
    seed: int | None = None
    options: JsonObject = field(default_factory=dict)


@dataclass(frozen=True)
class BenchmarkEpisodeResult:
    """Serializable per-episode policy evaluation artifact."""

    episode_id: str
    task_id: str
    task_index: int
    init_state_index: int
    success: bool
    steps: int
    reward_sum: float
    done: bool
    failure_reason: str | None
    action_shape: tuple[int, ...]
    action_min: float | None
    action_max: float | None
    observed_streams: tuple[str, ...]


@dataclass(frozen=True)
class BenchmarkEvaluationSummary:
    """Serializable aggregate policy evaluation artifact."""

    episodes: int
    successes: int
    success_rate: float
    success_threshold: float
    passed: bool


@dataclass(frozen=True)
class RuntimeStreamSnapshot:
    """Module-native stream values captured for one runtime reset or step tick."""

    observations: tuple[ObservationFrame, ...] = ()
    values: Mapping[str, object] = field(default_factory=dict)

    @property
    def observed_streams(self) -> tuple[str, ...]:
        streams: list[str] = []
        for stream in self.values:
            if stream not in streams:
                streams.append(stream)
        for frame in self.observations:
            if frame.stream not in streams:
                streams.append(frame.stream)
        return tuple(streams)


class PolicyEvalRuntimeSession:
    """Module-native runtime seam used by policy evaluation."""

    def reset(self, request: EpisodeResetRequest) -> EpisodeResetResponse:
        raise NotImplementedError

    def step(self, request: StepRequest) -> StepResponse:
        raise NotImplementedError

    def latest_observation_snapshot(self) -> RuntimeStreamSnapshot:
        raise NotImplementedError

    def close(self) -> None:
        pass


class PolicyActionSource(Spec, Protocol):
    def reset(self, episode_id: str | None = None) -> None: ...

    def infer_action(self, sample: RobotPolicyObservation) -> RobotPolicyAction: ...

    def close(self) -> None: ...

    def describe_backend(self) -> object: ...


class BenchmarkPolicyEvalModuleConfig(ModuleConfig):
    """Config for module-compatible benchmark policy evaluation."""

    artifact_dir: str = "artifacts/benchmark/policy-eval"
    max_steps: int = 1000
    success_threshold: float = 0.50
    close_policy_on_finish: bool = False
    video_dir: str | None = None
    video_streams: tuple[str, ...] = ()
    video_fps: int = 20


class BenchmarkPolicyEvalModule(Module):
    """DimOS module wrapper for lockstep benchmark policy evaluation.

    V1 keeps runtime and policy calls RPC-style/lockstep. The demo script may
    inject a local runtime client and policy module, while future blueprints can
    supply remote module clients behind the same protocol surface.
    """

    config: BenchmarkPolicyEvalModuleConfig
    robot_policy_module: PolicyActionSource | None

    def __init__(
        self,
        runtime_session: PolicyEvalRuntimeSession | None = None,
        robot_policy_module: PolicyActionSource | None = None,
        **kwargs: object,
    ) -> None:
        super().__init__(**kwargs)
        self._runtime_session = runtime_session
        self.robot_policy_module = robot_policy_module
        self._last_results: tuple[BenchmarkEpisodeResult, ...] = ()

    @property
    def last_results(self) -> tuple[BenchmarkEpisodeResult, ...]:
        return self._last_results

    def configure(
        self,
        *,
        runtime_session: PolicyEvalRuntimeSession,
        robot_policy_module: PolicyActionSource,
    ) -> None:
        self._runtime_session = runtime_session
        self.robot_policy_module = robot_policy_module

    @rpc
    def run_episodes(self, episodes: list[BenchmarkEpisodeSpec]) -> BenchmarkEvaluationSummary:
        policy_module = self.robot_policy_module
        if self._runtime_session is None or policy_module is None:
            raise RuntimeError(
                "BenchmarkPolicyEvalModule requires runtime session and policy client"
            )
        runner = BenchmarkPolicyEvalRunner(
            runtime_session=self._runtime_session,
            robot_policy_module=policy_module,
            artifact_dir=Path(self.config.artifact_dir),
            max_steps=self.config.max_steps,
            success_threshold=self.config.success_threshold,
            close_policy_on_finish=self.config.close_policy_on_finish,
            video_dir=Path(self.config.video_dir) if self.config.video_dir is not None else None,
            video_streams=self.config.video_streams,
            video_fps=self.config.video_fps,
        )
        summary = runner.run(episodes)
        self._last_results = runner.last_results
        return summary

    def close(self) -> None:
        self._close_module()

    @rpc
    def stop(self) -> None:
        if self.config.close_policy_on_finish and self.robot_policy_module is not None:
            self.robot_policy_module.close()
        super().stop()


def lerobot_libero_policy_eval_blueprint(
    *,
    checkpoint_id: str = "lerobot/VLA-JEPA-LIBERO",
    device: str | None = None,
    artifact_dir: str = "artifacts/benchmark/lerobot-vla-jepa-libero",
    max_steps: int = 1000,
    success_threshold: float = 0.50,
) -> Blueprint:
    """Create the module-shaped LeRobot LIBERO policy-evaluation blueprint.

    The runtime sidecar is still created by the benchmark/demo orchestration per
    episode; this blueprint captures the policy module and evaluation module
    configuration for DimOS module composition and future runtime-client remaps.
    """

    backend_params: dict[str, object] = {"checkpoint_id": checkpoint_id}
    if device is not None:
        backend_params["device"] = device
    return autoconnect(
        RobotPolicyModule.blueprint(
            backend_type="lerobot",
            backend_params=backend_params,
            contract_type="vla_jepa_libero",
            contract_params={},
        ),
        BenchmarkPolicyEvalModule.blueprint(
            artifact_dir=artifact_dir,
            max_steps=max_steps,
            success_threshold=success_threshold,
        ),
    )


class LiberoRobotPolicyObservationBuilder:
    """Convert LIBERO runtime observations into RobotPolicyObservation instances."""

    def build(
        self,
        *,
        episode: BenchmarkEpisodeSpec,
        tick_id: int,
        observations: Sequence[ObservationFrame],
        runtime_description: RuntimeDescription,
        observation_values: Mapping[str, object],
        reward: float = 0.0,
        done: bool = False,
        success: bool | None = None,
    ) -> RobotPolicyObservation:
        del episode, tick_id
        values: dict[str, object] = dict(observation_values)
        for frame in observations:
            if frame.kind == "state" or frame.kind == ObservationKind.STATE:
                state = frame.metadata.get("state", frame.metadata.get("values"))
                if state is not None:
                    values[frame.stream] = state
        language = runtime_description.metadata.get("language")
        task = language if isinstance(language, str) else None
        metadata: dict[str, object] = {
            "reward": reward,
            "done": done,
            "success": success,
            "observed_streams": tuple(frame.stream for frame in observations),
            "runtime_metadata": dict(runtime_description.metadata),
        }
        if task is not None:
            metadata["language"] = task
        return RobotPolicyObservation(
            observations=values,
            metadata=metadata,
        )


class BenchmarkPolicyEvalRunner:
    """Own benchmark lifecycle and execute robot-policy actions through runtime client.

    RobotPolicyModule owns inference only. This runner owns episode selection,
    runtime reset/step timing, policy reset calls, metrics, success gates, and
    artifact output.
    """

    def __init__(
        self,
        *,
        runtime_session: PolicyEvalRuntimeSession,
        robot_policy_module: PolicyActionSource,
        artifact_dir: Path,
        max_steps: int,
        success_threshold: float = 0.50,
        close_policy_on_finish: bool = True,
        video_dir: Path | None = None,
        video_streams: Sequence[str] = (),
        video_fps: int = 20,
    ) -> None:
        if max_steps <= 0:
            raise ValueError("max_steps must be positive")
        if video_fps <= 0:
            raise ValueError("video_fps must be positive")
        self._runtime_session = runtime_session
        self._robot_policy_module = robot_policy_module
        self._artifact_dir = artifact_dir
        self._max_steps = max_steps
        self._success_threshold = success_threshold
        self._close_policy_on_finish = close_policy_on_finish
        self._video_dir = video_dir
        self._video_streams = tuple(video_streams)
        self._video_fps = video_fps
        self._sample_builder = LiberoRobotPolicyObservationBuilder()
        self._last_results: tuple[BenchmarkEpisodeResult, ...] = ()

    @property
    def last_results(self) -> tuple[BenchmarkEpisodeResult, ...]:
        """Most recent episode results written by this runner."""

        return self._last_results

    def run(self, episodes: Sequence[BenchmarkEpisodeSpec]) -> BenchmarkEvaluationSummary:
        """Run all episodes and write required benchmark artifacts."""

        results: list[BenchmarkEpisodeResult] = []
        runtime_description: RuntimeDescription | None = None
        try:
            for episode in episodes:
                result, runtime_description = self._run_episode(episode)
                results.append(result)
        finally:
            if self._close_policy_on_finish:
                self._robot_policy_module.close()

        self._last_results = tuple(results)
        summary = self._summary(results)
        self._write_artifacts(results, summary, runtime_description)
        return summary

    def _run_episode(
        self, episode: BenchmarkEpisodeSpec
    ) -> tuple[BenchmarkEpisodeResult, RuntimeDescription]:
        reset_response = self._runtime_session.reset(
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
        self._robot_policy_module.reset(episode_id=episode.episode_id)

        runtime_description = reset_response.runtime_description
        snapshot = self._runtime_session.latest_observation_snapshot()
        if not snapshot.observed_streams:
            raise RuntimeError("runtime session produced no observation streams after reset")
        observations = snapshot.observations
        reward_sum = 0.0
        done = False
        success: bool | None = None
        failure_reason: str | None = None
        last_action: RobotPolicyAction | None = None
        observed_streams = snapshot.observed_streams
        video_frames: dict[str, list[np.ndarray]] = {stream: [] for stream in self._video_streams}
        steps = 0

        for tick_id in range(self._max_steps):
            self._append_video_frames(video_frames, snapshot.values)
            sample = self._sample_builder.build(
                episode=episode,
                tick_id=tick_id,
                observations=observations,
                runtime_description=runtime_description,
                reward=reward_sum,
                done=done,
                success=success,
                observation_values=snapshot.values,
            )
            action = self._robot_policy_module.infer_action(sample)
            frame = _runtime_action_frame(action, tick_id=tick_id)
            step_response = self._runtime_session.step(
                StepRequest(
                    episode_id=episode.episode_id,
                    tick_id=tick_id,
                    action=frame,
                )
            )
            steps = tick_id + 1
            last_action = action
            reward_sum += step_response.reward
            done = step_response.done
            success = step_response.success
            snapshot = self._runtime_session.latest_observation_snapshot()
            observations = snapshot.observations
            observed_streams = snapshot.observed_streams
            if done:
                break

        episode_success = bool(success)
        if not episode_success:
            failure_reason = "done_without_success" if done else "max_steps_without_success"

        action_values = last_action.values if last_action is not None else ()
        self._write_videos(episode.episode_id, video_frames)
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
            action_shape=(len(action_values),),
            action_min=min(action_values) if action_values else None,
            action_max=max(action_values) if action_values else None,
            observed_streams=observed_streams,
        ), runtime_description

    def _append_video_frames(
        self, video_frames: dict[str, list[np.ndarray]], stream_values: Mapping[str, object]
    ) -> None:
        for stream in self._video_streams:
            value = stream_values.get(stream)
            if not isinstance(value, np.ndarray):
                continue
            if value.ndim != 3 or value.shape[2] != 3:
                continue
            video_frames[stream].append(value.astype(np.uint8, copy=False))

    def _write_videos(self, episode_id: str, video_frames: dict[str, list[np.ndarray]]) -> None:
        if self._video_dir is None:
            return
        for stream, frames in video_frames.items():
            if not frames:
                continue
            stream_name = stream.replace("/", "_")
            video_path = self._video_dir / episode_id / f"{stream_name}.mp4"
            video_path.parent.mkdir(parents=True, exist_ok=True)
            imageio.mimsave(video_path, frames, fps=self._video_fps)  # type: ignore[arg-type]

    def _summary(self, results: Sequence[BenchmarkEpisodeResult]) -> BenchmarkEvaluationSummary:
        successes = sum(1 for result in results if result.success)
        success_rate = successes / len(results) if results else 0.0
        return BenchmarkEvaluationSummary(
            episodes=len(results),
            successes=successes,
            success_rate=success_rate,
            success_threshold=self._success_threshold,
            passed=success_rate > self._success_threshold,
        )

    def _write_artifacts(
        self,
        results: Sequence[BenchmarkEpisodeResult],
        summary: BenchmarkEvaluationSummary,
        runtime_description: RuntimeDescription | None,
    ) -> None:
        self._artifact_dir.mkdir(parents=True, exist_ok=True)
        write_json(self._artifact_dir / "summary.json", _json_ready(summary))
        if runtime_description is not None:
            write_json(self._artifact_dir / "runtime_description.json", runtime_description)
        write_json(
            self._artifact_dir / "checkpoint_metadata.json",
            _json_ready(self._robot_policy_module.describe_backend()),
        )
        _write_jsonl(self._artifact_dir / "episodes.jsonl", results)


def libero_object_episode_matrix(
    *, init_state_indices: Sequence[int] = (0, 1, 2, 3, 4)
) -> list[BenchmarkEpisodeSpec]:
    """Return the required 50-episode LIBERO object gate matrix."""

    return [
        BenchmarkEpisodeSpec(
            episode_id=f"libero_object_task{task_index}_init{init_state_index}",
            task_id="libero_object",
            task_index=task_index,
            init_state_index=init_state_index,
        )
        for task_index in range(10)
        for init_state_index in init_state_indices
    ]


def _runtime_action_frame(action: RobotPolicyAction, *, tick_id: int) -> RuntimeActionFrame:
    return RuntimeActionFrame(
        frame_type="runtime_action",
        space_id=action.space_id,
        values=list(action.values),
        sequence=action.sequence,
        tick_id=tick_id if action.sequence is None else None,
    )


def _write_jsonl(path: Path, rows: Sequence[object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(_json_ready(row), sort_keys=True) + "\n" for row in rows))


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
