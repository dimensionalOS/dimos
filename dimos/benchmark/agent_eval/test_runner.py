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

from datetime import UTC, datetime
import json
from pathlib import Path
import time
from types import SimpleNamespace

from pydantic import BaseModel

from dimos.agents.code_policy import CodePolicySessionReceipt
from dimos.benchmark.agent_eval.backend import BackendReadiness
from dimos.benchmark.agent_eval.config import select_destination
from dimos.benchmark.agent_eval.models import (
    ArtifactReference,
    DimSimBackendOptions,
    InfrastructureTimeouts,
    ResetReceipt,
    ResolvedSmokeConfig,
)
from dimos.benchmark.agent_eval.pi_adapter import PythonExecBroker
from dimos.benchmark.agent_eval.runner import (
    LocalAgentEvalRunner,
    PiTurn,
    _validate_reset,
)
from dimos.benchmark.dimsim.bundle import generate_smoke_release
from dimos.benchmark.dimsim.fixture import apartment_oracle_fixture


def _tool() -> dict[str, object]:
    return {
        "name": "python_exec",
        "description": (
            "Execute one synchronous Python program in the persistent policy session. "
            "This is a trusted, unsandboxed runtime."
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "code": {"type": "string"},
                "timeout_s": {"type": "number", "default": 110.0},
            },
            "required": ["code"],
        },
    }


class FakeMcp:
    def wait_for_ready(self, timeout: float) -> bool:
        return timeout > 0

    def list_tools(self) -> list[dict[str, object]]:
        return [_tool(), {"name": "move", "inputSchema": {"type": "object"}}]

    def call_tool(self, name, arguments=None):
        return {"content": [{"type": "text", "text": "ok"}]}


class FakeCodePolicy:
    def __init__(self) -> None:
        self.cancelled = False

    def reset_session(self, timeout_s: float) -> CodePolicySessionReceipt:
        return CodePolicySessionReceipt(
            session_id="code_policy_session_" + "c" * 32,
            reset_at=datetime.now(UTC),
            previous_session_id=None,
        )

    def interrupt_active(self, timeout_s: float) -> bool:
        return True

    def motion_active(self, timeout_s: float) -> bool:
        return False

    def cancel_motion(self, timeout_s: float) -> None:
        self.cancelled = True

    def close(self) -> None:
        pass


class NativeResult(BaseModel):
    evaluation_id: str
    passed: bool
    reason: str


class FakeBackend:
    def __init__(self, selected, native: NativeResult | None) -> None:
        self.selected = selected
        self.native = native
        self.cancelled = False
        self.cleaned = False

    def readiness(self, timeout_s):
        return BackendReadiness(backend="fake", ready=True, detail="ready")

    def reset(self, request, timeout_s):
        source = self.selected.contract.source
        return ResetReceipt(
            attempt_id=request.attempt_id,
            operation_id=request.operation_id,
            task_id=request.task_id,
            episode=request.episode,
            requested_pose=request.start_pose,
            applied_pose=request.start_pose,
            reset_generation=1,
            verified_source_revisions={
                "scene_id": source.scene_id,
                "scene_revision": source.scene_revision,
                "reset_revision": source.reset_revision,
                "upstream_revision": source.upstream_revision,
                "profile_revision": source.profile_revision,
            },
            source_digest=source.oracle_view_digest,
            initial_predicate_satisfied=False,
            acknowledged_at=datetime.now(UTC),
        )

    def start_evaluation(self, request, timeout_s):
        return type("Handle", (), {"operation_id": request.operation_id})()

    def wait_result(self, handle, timeout_s):
        if self.native is None:
            time.sleep(min(timeout_s, 0.03))
        else:
            time.sleep(0.01)
        return self.native

    def cancel(self, handle, timeout_s):
        self.cancelled = True
        if self.native is None:
            self.native = NativeResult(
                evaluation_id="eval-cancelled",
                passed=False,
                reason="cancelled",
            )

    def cleanup(self):
        self.cleaned = True


class FakePi:
    def __init__(
        self,
        attempt_path: Path,
        turns: list[int],
        broker: PythonExecBroker,
    ) -> None:
        self.session_id = "pi_session_" + "d" * 32
        self.turns = turns
        self.broker = broker
        self.policy_call_count = 0
        self.aborted = False
        self.disposed = False
        session = attempt_path / "pi-session"
        session.mkdir()
        self.path = session / "session.jsonl"
        self.path.write_text('{"type":"session"}\n')
        prompt = attempt_path / "pi-prompt"
        prompt.mkdir()
        self.system_prompt = prompt / "system.txt"
        self.initial_prompt = prompt / "initial.txt"
        self.system_prompt.write_text("Use python_exec.")
        self.initial_prompt.write_text("Navigate to the bathtub.")

    def prompt(self, prompt: str, timeout_s: float) -> PiTurn:
        increment = self.turns.pop(0) if self.turns else 0
        for _ in range(increment):
            self.broker.request("python_exec", {"code": "print('navigate')"})
            self.policy_call_count += 1
        return PiTurn(final_text="I think I succeeded", policy_call_count=self.policy_call_count)

    def abort(self, timeout_s: float) -> None:
        self.aborted = True

    def dispose(self) -> None:
        self.disposed = True

    def artifact_references(self) -> tuple[ArtifactReference, ...]:
        import hashlib

        return tuple(
            ArtifactReference(
                path=str(path.relative_to(self.path.parents[1])),
                sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                size_bytes=len(path.read_bytes()),
            )
            for path in (self.path, self.system_prompt, self.initial_prompt)
        )


class FakePiFactory:
    def __init__(self, turns: list[int]) -> None:
        self.turns = turns
        self.session: FakePi | None = None
        self.public_prompt: str | None = None

    def create(self, **kwargs):
        self.public_prompt = kwargs["public_prompt"]
        broker = PythonExecBroker(
            attempt_id=kwargs["attempt_path"].name,
            pi_session_id="pi_session_" + "d" * 32,
            code_policy_session_id=kwargs["code_policy_session_id"],
            mcp=kwargs["mcp"],
            call_log=kwargs["call_log"],
        )
        self.session = FakePi(kwargs["attempt_path"], list(self.turns), broker)
        return self.session


def _setup(tmp_path: Path, native: NativeResult | None, turns: list[int]):
    release = tmp_path / "release"
    generate_smoke_release(apartment_oracle_fixture(), release)
    task = next(
        json.loads(line)
        for line in (release / "public" / "tasks.jsonl").read_text().splitlines()
        if json.loads(line)["category"] == "destination"
    )
    selected = select_destination(release, task["task_id"])
    config = ResolvedSmokeConfig(
        release_root=str(release),
        task_id=task["task_id"],
        output_root=str(tmp_path / "attempts"),
        mcp_endpoint="http://localhost:9990/mcp",
        pi_model="gpt-5.6-luna",
        pi_thinking_level="medium",
        auth_mode="environment",
        credential_binding_sha256="e" * 64,
        timeouts=InfrastructureTimeouts(
            readiness_s=1.0,
            mcp_call_s=1.0,
            reset_s=1.0,
            evaluation_start_s=1.0,
            cancellation_s=1.0,
        ),
        episode_timeout_s=0.2,
        dimsim=DimSimBackendOptions(
            endpoint="http://localhost:8090",
            expected_scene_id="dimsim-apartment",
        ),
    )
    backend = FakeBackend(selected, native)
    policy = FakeCodePolicy()
    factory = FakePiFactory(turns)
    runner = LocalAgentEvalRunner(
        config=config,
        selected=selected,
        backend=backend,
        mcp=FakeMcp(),
        code_policy=policy,
        pi_factory=factory,
    )
    return runner, backend, policy, factory


def test_runner_native_success_ignores_pi_final_text_and_retains_artifacts(tmp_path) -> None:
    runner, backend, policy, factory = _setup(
        tmp_path,
        NativeResult(evaluation_id="eval-1", passed=True, reason="predicate_satisfied"),
        [1],
    )

    result = runner.run()

    assert result.exit_code == 0
    assert result.outcome.task_result == "passed"
    assert (result.attempt_path / "dimsim-result.v1.json").is_file()
    assert (result.attempt_path / "attempt-manifest.v1.json").is_file()
    assert backend.cancelled and backend.cleaned and policy.cancelled
    assert factory.session is not None and factory.session.aborted and factory.session.disposed
    assert factory.public_prompt == runner.selected.public.text
    assert runner.selected.contract.contract.target_entity_id not in factory.public_prompt
    assert str(runner.selected.contract.contract.threshold_m) not in factory.public_prompt
    manifest = json.loads((result.attempt_path / "attempt-manifest.v1.json").read_text())
    call = json.loads((result.attempt_path / "code-policy-calls.jsonl").read_text())
    task = json.loads((result.attempt_path / "task.v1.json").read_text())
    assert manifest["attempt_id"] == call["attempt_id"]
    assert manifest["pi_session_id"] == call["pi_session_id"]
    assert manifest["code_policy_session_id"] == call["code_policy_session_id"]
    assert manifest["task_id"] == task["task_id"]


def test_runner_accepts_backend_validated_applied_pose_tolerance(tmp_path) -> None:
    runner, _, _, _ = _setup(tmp_path, None, [])
    source = runner.selected.contract.source
    applied = runner.selected.start_pose.model_copy(
        update={"x_m": runner.selected.start_pose.x_m + 0.01}
    )
    reset = SimpleNamespace(
        verified_source_revisions={
            "scene_id": source.scene_id,
            "scene_revision": source.scene_revision,
            "reset_revision": source.reset_revision,
            "upstream_revision": source.upstream_revision,
            "profile_revision": source.profile_revision,
        },
        source_digest=source.oracle_view_digest,
        requested_pose=runner.selected.start_pose,
        applied_pose=applied,
        initial_predicate_satisfied=False,
    )

    _validate_reset(runner.selected, reset)


def test_two_no_policy_turns_is_valid_completed_failure(tmp_path) -> None:
    runner, _, _, _ = _setup(tmp_path, None, [0, 0])

    result = runner.run()

    assert result.exit_code == 0
    assert result.outcome.task_result == "failed"
    assert result.outcome.reason == "two_consecutive_no_policy_calls"
