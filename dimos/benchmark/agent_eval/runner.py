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

"""Single-attempt attached-service agent evaluation state machine."""

from __future__ import annotations

from concurrent.futures import (
    FIRST_COMPLETED,
    Future,
    ThreadPoolExecutor,
    TimeoutError as FutureTimeoutError,
    wait,
)
from datetime import UTC, datetime
import hashlib
from pathlib import Path
import time
from typing import Any, Literal, Protocol

from pydantic import BaseModel, JsonValue

from dimos.agents.code_policy import CodePolicySessionReceipt
from dimos.benchmark.agent_eval.backend import (
    BackendEvaluationRequest,
    BackendResetRequest,
    SimulatorBackend,
)
from dimos.benchmark.agent_eval.config import SelectedDestination
from dimos.benchmark.agent_eval.models import (
    ArtifactReference,
    AttemptManifest,
    BackendEpisodeReference,
    NativeResultReference,
    NormalizedOutcome,
    ResolvedSmokeConfig,
)
from dimos.benchmark.agent_eval.pi_adapter import (
    CodePolicyCallLog,
    McpBinding,
    wait_for_python_exec,
)
from dimos.benchmark.agent_eval.store import AttemptStore, new_operation_id

NEUTRAL_CONTINUATION = "Continue working on the task."
MAX_TURNS_SAFETY_CEILING = 100


class CodePolicyControl(Protocol):
    def reset_session(self, timeout_s: float) -> CodePolicySessionReceipt: ...

    def interrupt_active(self, timeout_s: float) -> bool: ...

    def motion_active(self, timeout_s: float) -> bool: ...

    def cancel_motion(self, timeout_s: float) -> None: ...

    def close(self) -> None: ...


class PiTurn(BaseModel):
    final_text: str = ""
    policy_call_count: int


class PiSession(Protocol):
    session_id: str

    def prompt(self, prompt: str, timeout_s: float) -> PiTurn: ...

    def abort(self, timeout_s: float) -> None: ...

    def dispose(self) -> None: ...

    def artifact_references(self) -> tuple[ArtifactReference, ...]: ...


class PiSessionFactory(Protocol):
    def create(
        self,
        *,
        attempt_path: Path,
        public_prompt: str,
        code_policy_session_id: str,
        call_log: CodePolicyCallLog,
        mcp: McpBinding,
    ) -> PiSession: ...


class RunnerResult(BaseModel):
    attempt_path: Path
    outcome: NormalizedOutcome
    exit_code: int


class LocalAgentEvalRunner:
    """Run exactly one generated destination against already running services."""

    def __init__(
        self,
        *,
        config: ResolvedSmokeConfig,
        selected: SelectedDestination,
        backend: SimulatorBackend,
        mcp: McpBinding,
        code_policy: CodePolicyControl,
        pi_factory: PiSessionFactory,
    ) -> None:
        self.config = config
        self.selected = selected
        self.backend = backend
        self.mcp = mcp
        self.code_policy = code_policy
        self.pi_factory = pi_factory

    def run(self) -> RunnerResult:
        store = AttemptStore(Path(self.config.output_root))
        started = time.monotonic()
        artifacts: list[ArtifactReference] = []
        pi: PiSession | None = None
        handle = None
        terminal_native: Any | None = None
        terminal_stage = "preparation"
        terminal_reason = "infrastructure_failure"
        attempt_status: Literal["completed", "failed"] = "failed"
        task_result: Literal["passed", "failed", "not_evaluated"] = "not_evaluated"
        call_log: CodePolicyCallLog | None = None
        try:
            store.append_event("attempt-created")
            artifacts.append(store.write_artifact("task.v1.json", self.selected.public))
            inventory = wait_for_python_exec(
                self.config.mcp_endpoint,
                self.mcp,
                self.config.timeouts.readiness_s,
            )
            artifacts.append(store.write_artifact("mcp-inventory.v1.json", inventory))
            store.append_event("mcp-ready")
            readiness = self.backend.readiness(self.config.timeouts.readiness_s)
            if not readiness.ready:
                raise RuntimeError(readiness.detail)
            store.append_event("backend-ready", payload={"backend": readiness.backend})

            policy_receipt = self.code_policy.reset_session(self.config.timeouts.mcp_call_s)
            store.append_event(
                "code-policy-reset",
                payload={"session_id": policy_receipt.session_id},
            )
            episode = BackendEpisodeReference(
                backend=readiness.backend,
                episode_id=f"episode-{store.attempt_id}",
                opaque={"deadline_s": self.config.episode_timeout_s},
            )
            reset_operation = new_operation_id()
            reset = self.backend.reset(
                BackendResetRequest(
                    attempt_id=store.attempt_id,
                    operation_id=reset_operation,
                    task_id=self.selected.public.task_id,
                    episode=episode,
                    start_pose=self.selected.start_pose,
                    source_revisions=_selected_source_revisions(self.selected),
                ),
                self.config.timeouts.reset_s,
            )
            _validate_reset(self.selected, reset)
            store.append_event(
                "simulator-reset",
                operation_id=reset_operation,
                payload={"reset_generation": reset.reset_generation},
            )
            evaluation_operation = new_operation_id()
            handle = self.backend.start_evaluation(
                BackendEvaluationRequest(
                    attempt_id=store.attempt_id,
                    operation_id=evaluation_operation,
                    task_id=self.selected.public.task_id,
                    episode=episode,
                    contract_digest=self.selected.contract_sha256,
                    contract_payload=self.selected.contract.contract.model_dump(mode="json"),
                ),
                self.config.timeouts.evaluation_start_s,
            )
            store.append_event(
                "evaluation-started",
                operation_id=evaluation_operation,
            )

            call_log = CodePolicyCallLog(store.path / "code-policy-calls.jsonl")
            pi = self.pi_factory.create(
                attempt_path=store.path,
                public_prompt=self.selected.public.text,
                code_policy_session_id=policy_receipt.session_id,
                call_log=call_log,
                mcp=self.mcp,
            )
            store.append_event("pi-started", payload={"session_id": pi.session_id})
            terminal_stage = "episode"
            terminal_native, terminal_reason = self._execute_episode(
                store,
                pi,
                handle,
            )
            attempt_status = "completed"
            if terminal_native is not None and _native_passed(terminal_native):
                task_result = "passed"
            else:
                task_result = "failed"
            store.append_event(
                "terminal-detected",
                operation_id=evaluation_operation,
                payload={"reason": terminal_reason, "task_result": task_result},
            )
        except KeyboardInterrupt:
            terminal_stage = "interrupted"
            terminal_reason = "user_interrupted"
            store.append_event("user-interrupted")
        except Exception as exc:
            terminal_reason = _diagnostic(exc)
            store.append_event(
                "infrastructure-failure",
                payload={"diagnostic": terminal_reason},
            )
        finally:
            if terminal_native is None and handle is not None and attempt_status == "completed":
                try:
                    self.backend.cancel(handle, self.config.timeouts.cancellation_s)
                    terminal_native = self.backend.wait_result(
                        handle, self.config.timeouts.cancellation_s
                    )
                except Exception as exc:
                    attempt_status = "failed"
                    task_result = "not_evaluated"
                    terminal_stage = "terminal-native-result"
                    terminal_reason = _diagnostic(exc)
                if terminal_native is None and attempt_status == "completed":
                    attempt_status = "failed"
                    task_result = "not_evaluated"
                    terminal_stage = "terminal-native-result"
                    terminal_reason = "backend did not retain a native terminal result"
            if terminal_native is not None:
                artifacts.append(
                    store.write_artifact(
                        "dimsim-result.v1.json",
                        _native_json(terminal_native),
                    )
                )
            cleanup_errors = self._cleanup(store, pi, handle)
            if cleanup_errors and attempt_status == "completed":
                attempt_status = "failed"
                task_result = "not_evaluated"
                terminal_stage = "cleanup"
                terminal_reason = "; ".join(cleanup_errors)
            if call_log is not None:
                call_log.close()
                artifacts.append(_reference(store.path, "code-policy-calls.jsonl"))
            if pi is not None:
                artifacts.extend(pi.artifact_references())
            store.append_event("attempt-finalizing")
            artifacts.append(_reference(store.path, "events.jsonl"))

            native_reference = None
            native_artifact = next(
                (item for item in artifacts if item.path == "dimsim-result.v1.json"),
                None,
            )
            if native_artifact is not None and terminal_native is not None:
                native_reference = NativeResultReference(
                    backend=self.backend.__class__.__name__,
                    artifact=native_artifact,
                    native_result_id=_native_result_id(terminal_native),
                )
            manifest = AttemptManifest(
                attempt_id=store.attempt_id,
                created_at=datetime.now(UTC),
                trust_mode="trusted-unsandboxed-simulation",
                source_release_root=self.config.release_root,
                release_id=self.selected.manifest.release_id,
                task_id=self.selected.public.task_id,
                contract_sha256=self.selected.contract_sha256,
                expected_outcome_id=self.selected.outcome.outcome_id,
                expected_outcome_sha256=self.selected.outcome_sha256,
                code_policy_session_id=(
                    policy_receipt.session_id if "policy_receipt" in locals() else None
                ),
                pi_session_id=pi.session_id if pi is not None else None,
                verified_source_revisions=(
                    reset.verified_source_revisions if "reset" in locals() else {}
                ),
                artifacts=tuple(artifacts),
            )
            artifacts.append(store.write_artifact("attempt-manifest.v1.json", manifest))
            required_complete = attempt_status == "completed" and _has_required_artifacts(artifacts)
            if attempt_status == "completed" and not required_complete:
                attempt_status = "failed"
                task_result = "not_evaluated"
                terminal_stage = "artifacts"
                terminal_reason = "required artifacts are incomplete"
            outcome = NormalizedOutcome(
                attempt_id=store.attempt_id,
                attempt_status=attempt_status,
                task_result=task_result,
                terminal_stage=terminal_stage,
                reason=terminal_reason,
                required_artifacts_complete=required_complete,
                native_result=native_reference,
                finished_at=datetime.now(UTC),
                duration_s=max(0.0, time.monotonic() - started),
            )
            store.write_outcome(outcome)
            store.close()
        return RunnerResult(
            attempt_path=store.path,
            outcome=outcome,
            exit_code=0 if outcome.attempt_status == "completed" else 1,
        )

    def _execute_episode(
        self,
        store: AttemptStore,
        pi: PiSession,
        handle: Any,
    ) -> tuple[Any | None, str]:
        deadline = time.monotonic() + self.config.episode_timeout_s
        no_policy_turns = 0
        prompt = self.selected.public.text
        executor = ThreadPoolExecutor(max_workers=2)
        terminal_recorded = False
        try:
            native_future = executor.submit(
                self.backend.wait_result,
                handle,
                self.config.episode_timeout_s,
            )

            def finish(native: Any | None, reason: str) -> tuple[Any, str]:
                nonlocal terminal_recorded
                if native is None:
                    self.backend.cancel(handle, self.config.timeouts.cancellation_s)
                    if native_future.done():
                        native = native_future.result()
                    else:
                        try:
                            native = native_future.result(
                                timeout=self.config.timeouts.cancellation_s
                            )
                        except FutureTimeoutError:
                            native = None
                    if native is None:
                        native = self.backend.wait_result(
                            handle, self.config.timeouts.cancellation_s
                        )
                if native is None:
                    raise RuntimeError("backend did not retain a native terminal result")
                store.append_event(
                    "native-terminal-recorded",
                    operation_id=handle.operation_id,
                    payload={
                        "native_result_id": _native_result_id(native),
                        "native_reason": _native_reason(native),
                    },
                )
                terminal_recorded = True
                pi.abort(self.config.timeouts.cancellation_s)
                self.backend.cancel(handle, self.config.timeouts.cancellation_s)
                return native, reason

            for turn_index in range(MAX_TURNS_SAFETY_CEILING):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return finish(None, "episode_timeout")
                before = _policy_call_count(pi)
                pi_future = executor.submit(pi.prompt, prompt, remaining)
                pending: set[Future[Any]] = {native_future, pi_future}
                done, _ = wait(
                    pending,
                    timeout=remaining,
                    return_when=FIRST_COMPLETED,
                )
                if native_future in done:
                    native = native_future.result()
                    if native is not None:
                        return finish(native, _native_reason(native))
                    return finish(None, "episode_timeout")
                if pi_future not in done:
                    return finish(None, "episode_timeout")
                turn = pi_future.result()
                after = max(_policy_call_count(pi), turn.policy_call_count)
                if after <= before:
                    no_policy_turns += 1
                else:
                    no_policy_turns = 0
                store.append_event(
                    "pi-turn-ended",
                    payload={
                        "turn": turn_index + 1,
                        "policy_calls": after - before,
                    },
                )
                if no_policy_turns >= 2:
                    return finish(None, "two_consecutive_no_policy_calls")
                while self.code_policy.motion_active(
                    min(1.0, max(0.001, deadline - time.monotonic()))
                ):
                    if native_future.done():
                        native = native_future.result()
                        return finish(
                            native,
                            _native_reason(native) if native is not None else "episode_timeout",
                        )
                    if time.monotonic() >= deadline:
                        return finish(None, "episode_timeout")
                prompt = NEUTRAL_CONTINUATION
                store.append_event("pi-continuation")
            return finish(None, "pathological_turn_ceiling")
        finally:
            if not terminal_recorded:
                pi.abort(self.config.timeouts.cancellation_s)
                self.backend.cancel(handle, self.config.timeouts.cancellation_s)
            executor.shutdown(wait=True, cancel_futures=True)

    def _cleanup(
        self,
        store: AttemptStore,
        pi: PiSession | None,
        handle: Any | None,
    ) -> list[str]:
        errors: list[str] = []
        actions = (
            ("pi-abort", lambda: pi.abort(self.config.timeouts.cancellation_s) if pi else None),
            (
                "code-policy-interrupt",
                lambda: self.code_policy.interrupt_active(self.config.timeouts.cancellation_s),
            ),
            (
                "evaluation-cancel",
                lambda: self.backend.cancel(handle, self.config.timeouts.cancellation_s)
                if handle
                else None,
            ),
            (
                "motion-cancel",
                lambda: self.code_policy.cancel_motion(self.config.timeouts.cancellation_s),
            ),
        )
        for kind, action in actions:
            try:
                action()
                store.append_event(kind)
            except Exception as exc:
                errors.append(f"{kind}: {_diagnostic(exc)}")
        if pi is not None:
            try:
                pi.dispose()
            except Exception as exc:
                errors.append(f"pi-dispose: {_diagnostic(exc)}")
        try:
            self.backend.cleanup()
        except Exception as exc:
            errors.append(f"backend-cleanup: {_diagnostic(exc)}")
        try:
            self.code_policy.close()
        except Exception as exc:
            errors.append(f"dimos-control-close: {_diagnostic(exc)}")
        return errors


def _validate_reset(selected: SelectedDestination, reset: Any) -> None:
    source = selected.contract.source
    expected = {
        "scene_id": source.scene_id,
        "scene_revision": source.scene_revision,
        "reset_revision": source.reset_revision,
        "upstream_revision": source.upstream_revision,
        "profile_revision": source.profile_revision,
    }
    if reset.verified_source_revisions != expected:
        raise ValueError("post-reset source revisions do not match selected task")
    if reset.source_digest != source.oracle_view_digest:
        raise ValueError("post-reset source digest does not match selected task")
    if reset.requested_pose != selected.start_pose:
        raise ValueError("authoritative reset request does not match selected start pose")
    if reset.initial_predicate_satisfied:
        raise ValueError("selected task predicate is already satisfied after reset")


def _selected_source_revisions(selected: SelectedDestination) -> dict[str, str]:
    source = selected.contract.source
    return {
        "scene_id": source.scene_id,
        "profile_revision": source.profile_revision,
        "reset_revision": source.reset_revision,
        "upstream_revision": source.upstream_revision,
    }


def _policy_call_count(pi: PiSession) -> int:
    value = getattr(pi, "policy_call_count", 0)
    return value if isinstance(value, int) else 0


def _native_passed(native: Any) -> bool:
    return getattr(native, "passed", False) is True


def _native_reason(native: Any) -> str:
    value = getattr(native, "reason", None)
    return value if isinstance(value, str) and value else "native_terminal"


def _native_result_id(native: Any) -> str:
    for name in ("result_id", "evaluation_id"):
        value = getattr(native, name, None)
        if isinstance(value, str) and value:
            return value
    return "native-result"


def _native_json(native: Any) -> JsonValue:
    if isinstance(native, BaseModel):
        return native.model_dump(mode="json")
    if isinstance(native, dict):
        return native
    raise TypeError("native backend result is not serializable")


def _reference(root: Path, relative_path: str) -> ArtifactReference:
    data = (root / relative_path).read_bytes()
    return ArtifactReference(
        path=relative_path,
        sha256=hashlib.sha256(data).hexdigest(),
        size_bytes=len(data),
    )


def _has_required_artifacts(artifacts: list[ArtifactReference]) -> bool:
    paths = {item.path for item in artifacts}
    return {
        "task.v1.json",
        "mcp-inventory.v1.json",
        "events.jsonl",
        "code-policy-calls.jsonl",
        "dimsim-result.v1.json",
        "attempt-manifest.v1.json",
        "pi-prompt/system.txt",
        "pi-prompt/initial.txt",
    } <= paths and any(path.startswith("pi-session/") for path in paths)


def _diagnostic(exc: Exception) -> str:
    return f"{type(exc).__name__}: {exc}".replace("\n", " ")[:1024]
