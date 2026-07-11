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

from dataclasses import dataclass, field
import json
from pathlib import Path

from dimos_runtime_protocol import (
    EpisodeResetRequest,
    EpisodeResetResponse,
    MotorStateFrame,
    ObservationFrame,
    ObservationKind,
    RuntimeDescription,
    StepRequest,
    StepResponse,
)
import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.robot_learning.policy_rollout.evaluation import (
    BenchmarkEpisodeSpec,
    BenchmarkPolicyEvalModule,
    BenchmarkPolicyEvalRunner,
    PolicyEvalRuntimeSession,
    RuntimeStreamSnapshot,
    lerobot_libero_policy_eval_blueprint,
)
from dimos.robot_learning.policy_rollout.models import (
    PolicyBackendDescription,
    RobotPolicyAction,
    RobotPolicyObservation,
)
from dimos.robot_learning.policy_rollout.robot_policy_module import RobotPolicyModule


@dataclass
class FakeRuntimeSession(PolicyEvalRuntimeSession):
    success_by_episode: dict[str, bool]
    missing_reset_streams: bool = False
    reset_requests: list[EpisodeResetRequest] = field(default_factory=list)
    step_requests: list[StepRequest] = field(default_factory=list)
    latest_snapshot: RuntimeStreamSnapshot = field(default_factory=RuntimeStreamSnapshot)

    def reset(self, request: EpisodeResetRequest) -> EpisodeResetResponse:
        self.reset_requests.append(request)
        self.latest_snapshot = (
            RuntimeStreamSnapshot() if self.missing_reset_streams else _snapshot("agentview")
        )
        return EpisodeResetResponse(
            episode_id=request.episode_id,
            runtime_description=_runtime_description(),
            observations=[],
        )

    def step(self, request: StepRequest) -> StepResponse:
        self.step_requests.append(request)
        success = self.success_by_episode[request.episode_id]
        self.latest_snapshot = _snapshot("agentview", "eye_in_hand")
        return StepResponse(
            episode_id=request.episode_id,
            tick_id=request.tick_id,
            motor_state=MotorStateFrame(robot_id="panda", names=[], q=[], dq=[], tau=[]),
            observations=[],
            reward=1.0 if success else 0.0,
            done=True,
            success=success,
        )

    def latest_observation_snapshot(self) -> RuntimeStreamSnapshot:
        return self.latest_snapshot


@dataclass
class FakeRobotPolicyModule:
    fail_on_infer: bool = False
    reset_episode_ids: list[str | None] = field(default_factory=list)
    samples: list[RobotPolicyObservation] = field(default_factory=list)
    closed: bool = False

    def reset(self, episode_id: str | None = None) -> None:
        self.reset_episode_ids.append(episode_id)

    def infer_action(self, sample: RobotPolicyObservation) -> RobotPolicyAction:
        if self.fail_on_infer:
            raise ValueError("contract mismatch")
        self.samples.append(sample)
        return RobotPolicyAction(
            space_id="libero.ee_delta_6d_gripper.normalized.v1",
            values=(0.0, 0.1, -0.1, 0.2, -0.2, 0.3, 1.0),
        )

    def close(self) -> None:
        self.closed = True

    def describe_backend(self) -> PolicyBackendDescription:
        return PolicyBackendDescription(
            backend_type="fake", checkpoint_id="lerobot/VLA-JEPA-LIBERO"
        )


def test_runner_owns_runtime_reset_step_policy_reset_and_artifacts(tmp_path: Path) -> None:
    runtime = FakeRuntimeSession(success_by_episode={"ep-0": True})
    policy = FakeRobotPolicyModule()
    runner = BenchmarkPolicyEvalRunner(
        runtime_session=runtime,
        robot_policy_module=policy,  # type: ignore[arg-type]
        artifact_dir=tmp_path,
        max_steps=3,
    )

    summary = runner.run(
        [
            BenchmarkEpisodeSpec(
                episode_id="ep-0",
                task_id="libero_object",
                task_index=0,
                init_state_index=1,
                seed=7,
            )
        ]
    )

    assert summary.episodes == 1
    assert summary.successes == 1
    assert summary.passed
    assert policy.reset_episode_ids == ["ep-0"]
    assert policy.closed
    assert runtime.reset_requests[0].options["task_index"] == 0
    assert runtime.reset_requests[0].options["init_state_index"] == 1
    assert len(runtime.step_requests) == 1
    action = runtime.step_requests[0].action
    assert action.frame_type == "runtime_action"
    assert action.tick_id == 0
    assert action.space_id == "libero.ee_delta_6d_gripper.normalized.v1"
    assert "agentview" in policy.samples[0].observations
    assert policy.samples[0].metadata["language"] == "pick up the object"

    summary_json = json.loads((tmp_path / "summary.json").read_text())
    assert summary_json["success_rate"] == 1.0
    assert (tmp_path / "runtime_description.json").exists()
    assert not (tmp_path / "contract_description.json").exists()
    assert (tmp_path / "checkpoint_metadata.json").exists()
    episode_records = (tmp_path / "episodes.jsonl").read_text().splitlines()
    assert len(episode_records) == 1
    episode_record = json.loads(episode_records[0])
    assert episode_record["action_shape"] == [7]
    assert episode_record["observed_streams"] == ["agentview", "eye_in_hand"]


def test_runner_continues_after_policy_episode_failure(tmp_path: Path) -> None:
    runtime = FakeRuntimeSession(success_by_episode={"ep-0": False, "ep-1": True})
    policy = FakeRobotPolicyModule()
    runner = BenchmarkPolicyEvalRunner(
        runtime_session=runtime,
        robot_policy_module=policy,  # type: ignore[arg-type]
        artifact_dir=tmp_path,
        max_steps=1,
    )

    summary = runner.run(
        [
            BenchmarkEpisodeSpec("ep-0", "libero_object", 0, 0),
            BenchmarkEpisodeSpec("ep-1", "libero_object", 1, 0),
        ]
    )

    assert summary.episodes == 2
    assert summary.successes == 1
    assert not summary.passed
    assert [request.episode_id for request in runtime.reset_requests] == ["ep-0", "ep-1"]
    assert [request.episode_id for request in runtime.step_requests] == ["ep-0", "ep-1"]
    episode_records = [
        json.loads(line) for line in (tmp_path / "episodes.jsonl").read_text().splitlines()
    ]
    assert episode_records[0]["failure_reason"] == "done_without_success"
    assert episode_records[1]["success"] is True


def test_runner_aborts_setup_or_contract_error(tmp_path: Path) -> None:
    runtime = FakeRuntimeSession(success_by_episode={"ep-0": True})
    policy = FakeRobotPolicyModule(fail_on_infer=True)
    runner = BenchmarkPolicyEvalRunner(
        runtime_session=runtime,
        robot_policy_module=policy,  # type: ignore[arg-type]
        artifact_dir=tmp_path,
        max_steps=1,
    )

    with pytest.raises(ValueError, match="contract mismatch"):
        runner.run([BenchmarkEpisodeSpec("ep-0", "libero_object", 0, 0)])

    assert len(runtime.step_requests) == 0
    assert policy.closed


def test_runner_fails_when_runtime_session_has_no_stream_snapshot(tmp_path: Path) -> None:
    runtime = FakeRuntimeSession(success_by_episode={"ep-0": True}, missing_reset_streams=True)
    policy = FakeRobotPolicyModule()
    runner = BenchmarkPolicyEvalRunner(
        runtime_session=runtime,
        robot_policy_module=policy,  # type: ignore[arg-type]
        artifact_dir=tmp_path,
        max_steps=1,
    )

    with pytest.raises(RuntimeError, match="no observation streams"):
        runner.run([BenchmarkEpisodeSpec("ep-0", "libero_object", 0, 0)])

    assert len(runtime.step_requests) == 0
    assert policy.closed


def test_runner_writes_video_frames_when_enabled(tmp_path: Path, mocker: MockerFixture) -> None:
    video_writer = mocker.patch("dimos.robot_learning.policy_rollout.evaluation.imageio.mimsave")
    runtime = FakeRuntimeSession(success_by_episode={"ep-0": True})
    policy = FakeRobotPolicyModule()
    runner = BenchmarkPolicyEvalRunner(
        runtime_session=runtime,
        robot_policy_module=policy,  # type: ignore[arg-type]
        artifact_dir=tmp_path,
        max_steps=1,
        video_dir=tmp_path / "videos",
        video_streams=("agentview",),
        video_fps=12,
    )

    runner.run([BenchmarkEpisodeSpec("ep-0", "libero_object", 0, 0)])

    video_writer.assert_called_once()
    assert video_writer.call_args.args[0] == tmp_path / "videos" / "ep-0" / "agentview.mp4"
    assert len(video_writer.call_args.args[1]) == 1
    assert video_writer.call_args.kwargs["fps"] == 12


def test_module_run_episodes_uses_injected_clients_and_writes_artifacts(
    tmp_path: Path,
) -> None:
    runtime = FakeRuntimeSession(success_by_episode={"ep-0": True})
    policy = FakeRobotPolicyModule()
    module = BenchmarkPolicyEvalModule(
        runtime_session=runtime,
        robot_policy_module=policy,  # type: ignore[arg-type]
        artifact_dir=str(tmp_path),
        max_steps=2,
    )

    try:
        summary = module.run_episodes([BenchmarkEpisodeSpec("ep-0", "libero_object", 0, 1, seed=7)])

        assert summary.episodes == 1
        assert summary.successes == 1
        assert summary.passed
        assert len(module.last_results) == 1
        assert module.last_results[0].episode_id == "ep-0"
        assert policy.reset_episode_ids == ["ep-0"]
        assert not policy.closed
        assert runtime.reset_requests[0].options["task_index"] == 0
        assert runtime.reset_requests[0].options["init_state_index"] == 1
        assert len(runtime.step_requests) == 1
        assert (tmp_path / "summary.json").exists()
        assert (tmp_path / "runtime_description.json").exists()
        assert not (tmp_path / "contract_description.json").exists()
        assert (tmp_path / "checkpoint_metadata.json").exists()
        episode_records = (tmp_path / "episodes.jsonl").read_text().splitlines()
        assert len(episode_records) == 1
        assert json.loads(episode_records[0])["success"] is True
    finally:
        module.stop()


def test_module_run_episodes_uses_configured_clients(tmp_path: Path) -> None:
    runtime = FakeRuntimeSession(success_by_episode={"ep-0": True})
    policy = FakeRobotPolicyModule()
    module = BenchmarkPolicyEvalModule(artifact_dir=str(tmp_path), max_steps=1)

    try:
        module.configure(
            runtime_session=runtime,
            robot_policy_module=policy,  # type: ignore[arg-type]
        )
        summary = module.run_episodes([BenchmarkEpisodeSpec("ep-0", "libero_object", 0, 0)])

        assert summary.successes == 1
        assert len(module.last_results) == 1
        assert len(runtime.step_requests) == 1
        assert not policy.closed
    finally:
        module.stop()


def test_module_run_episodes_requires_runtime_and_policy_clients(
    tmp_path: Path,
) -> None:
    module = BenchmarkPolicyEvalModule(artifact_dir=str(tmp_path))

    try:
        with pytest.raises(RuntimeError, match="requires runtime session and policy client"):
            module.run_episodes([BenchmarkEpisodeSpec("ep-0", "libero_object", 0, 0)])
    finally:
        module.stop()


def test_lerobot_libero_policy_eval_blueprint_configures_modules() -> None:
    blueprint = lerobot_libero_policy_eval_blueprint(
        checkpoint_id="checkpoint",
        device="cpu",
        artifact_dir="artifacts/out",
        max_steps=12,
        success_threshold=0.6,
    )

    atoms_by_module = {atom.module: atom for atom in blueprint.blueprints}
    policy_atom = atoms_by_module[RobotPolicyModule]
    eval_atom = atoms_by_module[BenchmarkPolicyEvalModule]

    assert policy_atom.kwargs["backend_type"] == "lerobot"
    assert policy_atom.kwargs["backend_params"] == {
        "checkpoint_id": "checkpoint",
        "device": "cpu",
    }
    assert policy_atom.kwargs["contract_type"] == "vla_jepa_libero"
    assert eval_atom.kwargs["artifact_dir"] == "artifacts/out"
    assert eval_atom.kwargs["max_steps"] == 12
    assert eval_atom.kwargs["success_threshold"] == 0.6


def _runtime_description() -> RuntimeDescription:
    return RuntimeDescription(
        runtime_id="libero-pro",
        backend="libero-pro",
        capabilities=["native-runtime-action"],
        robot_surfaces=[],
        control_step_hz=20,
        observation_streams=["agentview", "eye_in_hand", "robot_state"],
        metadata={"language": "pick up the object"},
    )


def _snapshot(*streams: str) -> RuntimeStreamSnapshot:
    values = {stream: np.zeros((4, 4, 3), dtype=np.uint8) for stream in streams}
    return RuntimeStreamSnapshot(
        observations=tuple(_observation(stream) for stream in streams),
        values=values,
    )


def _observation(stream: str) -> ObservationFrame:
    return ObservationFrame(
        stream=stream,
        kind=ObservationKind.IMAGE,
        shape=[128, 128, 3],
        dtype="uint8",
    )
