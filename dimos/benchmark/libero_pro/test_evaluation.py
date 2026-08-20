"""Orchestration tests for one fresh LIBERO-PRO trial."""

from pathlib import Path
from types import SimpleNamespace

import cloudpickle
from pytest_mock import MockerFixture

from dimos.benchmark.evaluation.protocol import PolicyArtifact, PolicyExecution
from dimos.benchmark.libero_pro.assets import PreparedAssets
import dimos.benchmark.libero_pro.evaluation as evaluation
from dimos.benchmark.libero_pro.models import LiberoTaskManifest
from dimos.benchmark.libero_pro.podman import ContainerEndpoints

CASE = Path(__file__).parent / "cases" / "goal-task-0-single-trial" / "task.json"


def test_evaluation_protocol_exposes_only_minimal_harness_contract() -> None:
    protocol = evaluation.EVALUATION_PROTOCOL

    assert "policy(app: Dimos) -> None" in protocol
    assert "submit_policy(policy)" in protocol
    assert "PolicyCandidate" in protocol
    assert "candidate.evidence.timeline()" in protocol
    assert "freeze_policy(candidate)" in protocol
    assert "exact task input" in protocol

    forbidden = (
        "segment_best",
        "GraspExecutionModule",
        "planning_group",
        "collision-checked",
        "skillbook",
        "verifier",
        "diagnoser",
        "privileged",
        "ASCII art",
    )
    assert all(term not in protocol for term in forbidden)


def test_trial_starts_clock_and_prepared_policy_only_after_blueprint_is_ready(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    events: list[str] = []
    manifest = LiberoTaskManifest.model_validate_json(CASE.read_bytes())
    assets = PreparedAssets(tmp_path / "task.bddl", tmp_path / "states.pt")
    policy = PolicyArtifact("def policy(app): ...\n", cloudpickle.dumps(lambda: None), "digest")

    class FakeContainer:
        def __init__(self, *_args: object, **_kwargs: object) -> None:
            pass

        def start(self) -> ContainerEndpoints:
            events.append("container.start")
            return ContainerEndpoints("policy", "control", "token")

        def stop(self) -> None:
            events.append("container.stop")

    class FakeControl:
        def __init__(self, endpoint: str, token: str) -> None:
            assert (endpoint, token) == ("control", "token")

        def wait_ready(self) -> None:
            events.append("control.ready")

        def initialize(self, _manifest: LiberoTaskManifest, index: int) -> None:
            assert index == 0
            events.append("control.initialize")

        def start(self) -> None:
            events.append("control.start")

        def wait_terminal(self, _timeout: float) -> SimpleNamespace:
            events.append("control.wait")
            return SimpleNamespace(
                success=True,
                score=1.0,
                reward=1.0,
                terminal_reason="success",
                policy_ticks=210,
                backend_ticks=215,
                error="",
            )

        def cancel(self) -> None:
            events.append("control.cancel")

        def close(self) -> None:
            events.append("control.close")

    class FakeExecution:
        def start(self) -> None:
            events.append("policy.start")

        def finish(self, *, grace_s: float = 1.0) -> PolicyExecution:
            del grace_s
            events.append("policy.finish")
            return PolicyExecution(
                "policy_error",
                10.5,
                "RPC closed after native success",
                "gripper result: succeeded",
            )

    class FakeRuntime:
        def prepare(
            self,
            _policy: PolicyArtifact,
            *,
            memory_path: Path,
            startup_timeout_s: float,
        ) -> FakeExecution:
            assert memory_path.name == "recording.db"
            assert startup_timeout_s == 30.0
            events.append("policy.prepare")
            return FakeExecution()

    coordinator = mocker.Mock()
    coordinator.start_rpc_service.side_effect = lambda: events.append("blueprint.ready")

    def stop_blueprint() -> None:
        events.append("blueprint.stop")
        (tmp_path / "trial" / "trial.mp4").write_bytes(b"video")

    coordinator.stop.side_effect = stop_blueprint
    mocker.patch.object(evaluation, "LiberoPodmanContainer", FakeContainer)
    mocker.patch.object(evaluation, "EvaluationControlClient", FakeControl)
    mocker.patch.object(
        evaluation, "libero_trial_blueprint", return_value=mocker.sentinel.blueprint
    )
    mocker.patch.object(
        evaluation.ModuleCoordinator,
        "build",
        side_effect=lambda _blueprint: (events.append("blueprint.build"), coordinator)[1],
    )

    trial, native = evaluation._run_trial(
        manifest,
        assets,
        policy,
        init_index=0,
        path=tmp_path / "trial",
        runtime=FakeRuntime(),  # type: ignore[arg-type]
        run_id="scored",
    )

    assert trial.outcome.status == "policy_error"
    assert trial.policy_output == "gripper result: succeeded"
    assert (tmp_path / "trial" / "policy-output.log").read_text() == trial.policy_output
    assert native["score"] == 1.0
    assert native["policy_execution_status"] == "policy_error"
    assert events == [
        "container.start",
        "control.ready",
        "control.initialize",
        "blueprint.build",
        "blueprint.ready",
        "policy.prepare",
        "control.start",
        "policy.start",
        "control.wait",
        "policy.finish",
        "blueprint.stop",
        "control.close",
        "container.stop",
    ]
