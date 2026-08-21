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

"""Fixed Pi exploration and clean, agent-free CodePolicy execution."""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import hashlib
import io
import json
import multiprocessing
from pathlib import Path
import pickle
import shutil
import time
from typing import Any, Literal, TextIO

import cloudpickle  # type: ignore[import-untyped]

from dimos.agents.code_policy_core import validate_policy_callable
from dimos.agents.code_policy_server import CodePolicyMcpServer
from dimos.benchmark.evaluation.models import ArtifactReference, RuntimeCondition, RuntimeIdentity
from dimos.benchmark.evaluation.pi_process import PI_VERSION, PiCliRunner, PiRunError
from dimos.benchmark.evaluation.progress import ProgressSink, StatusProgress, emit_progress
from dimos.benchmark.evaluation.protocol import (
    DebugTrialSubmitter,
    ExplorationOutcome,
    PolicyArtifact,
    PolicyCandidate,
    PolicyExecution,
    PolicyExecutionHandle,
    PolicyRequestError,
    PolicySnapshot,
    TrialEvidence,
    TrialRun,
)

CODE_POLICY_PROFILE: Literal["code-policy-v1"] = "code-policy-v1"
TURN_TIMEOUT_SECONDS = 600.0
DEFAULT_MAX_SUBMISSIONS = 5
MAX_POLICY_OUTPUT = 32_000

SYSTEM_INSTRUCTIONS = """You are a CodePolicy exploration agent.

Use the single `python_exec` tool to solve the supplied robotics task. Python
executes in a persistent trusted environment. Define a synchronous function
with the exact signature `def policy(app: Dimos) -> None`, then call
`submit_policy(policy)` to test it. Every accepted submission returns an immutable
PolicyCandidate and starts a fresh
debug environment and a fresh policy-only DimOS blueprint. Inspect the returned
candidate's bounded evidence summary, then drill into its timeline, filtered logs,
frames, policy output, read-only Memory2 recording, or raw artifacts as needed.
You may submit at most five trials. Before finishing, call
`freeze_policy(candidate)` exactly once to select the task-level policy evaluated
later without an agent. Freezing is irreversible and closes submissions. Do not
connect to DimOS directly from the exploration REPL.
"""


class CodePolicyRuntimeFactory:
    """The one fixed CodePolicy runtime supplied to complete Evaluations."""

    def __init__(
        self,
        *,
        api_key: str,
        workspace: Path,
        condition: RuntimeCondition,
        progress: ProgressSink | None = None,
    ) -> None:
        self.api_key = api_key
        self.workspace = workspace
        self.condition = condition
        self.progress = progress
        self._exploration_count = 0
        self._prompt_evidence: list[ArtifactReference] = []
        self._runtime_artifacts: list[ArtifactReference] = []

    @property
    def identity(self) -> RuntimeIdentity:
        return RuntimeIdentity(
            profile=CODE_POLICY_PROFILE,
            driver_version=PI_VERSION,
            model=self.condition.model,
            thinking_level=self.condition.thinking_level,
        )

    @property
    def prompt_evidence(self) -> tuple[ArtifactReference, ...]:
        return tuple(self._prompt_evidence)

    @property
    def runtime_artifacts(self) -> tuple[ArtifactReference, ...]:
        return tuple(self._runtime_artifacts)

    def explore(
        self,
        *,
        evaluation_protocol: str,
        task_input: str,
        submit_debug_trial: DebugTrialSubmitter,
        max_submissions: int = DEFAULT_MAX_SUBMISSIONS,
    ) -> ExplorationOutcome:
        if not evaluation_protocol.strip() or not task_input.strip():
            raise ValueError("evaluation_protocol and task_input must be non-empty")
        if max_submissions != DEFAULT_MAX_SUBMISSIONS:
            raise ValueError(
                f"code-policy-v1 requires exactly {DEFAULT_MAX_SUBMISSIONS} submissions"
            )

        self._exploration_count += 1
        relative = Path("runtime") / f"exploration-{self._exploration_count:04d}"
        path = self.workspace / relative
        path.mkdir(parents=True)
        manager = _SubmissionManager(
            workspace=self.workspace,
            path=path,
            relative_path=relative,
            submit_debug_trial=submit_debug_trial,
            max_submissions=max_submissions,
            record_artifact=self._runtime_artifacts.append,
        )
        server = CodePolicyMcpServer(manager.submit, freeze_handler=manager.freeze)
        server.start()
        try:
            user_message = _assemble_user_message(evaluation_protocol, task_input)
            self._record_prompt(path, relative, evaluation_protocol, task_input, user_message)
            working = path / "working"
            working.mkdir()
            cli, extension = _pi_paths()
            runner = PiCliRunner(
                cli=cli,
                extension=extension,
                model=self.condition.model,
                thinking_level=self.condition.thinking_level,
                timeout_s=TURN_TIMEOUT_SECONDS,
                progress=self.progress,
            )
            try:
                result = runner.run(
                    prompt=user_message,
                    system_prompt=SYSTEM_INSTRUCTIONS,
                    mcp_url=server.mcp_url,
                    api_key=self.api_key,
                    run_dir=working,
                )
            except PiRunError as exc:
                self._record_text(path, relative, "stderr.log", exc.stderr, "Pi stderr")
                self._record_pi_transcript(
                    path=path,
                    relative=relative,
                    source=exc.transcript_path,
                    runner=runner,
                    mcp_url=server.mcp_url,
                )
                raise
            self._record_pi_transcript(
                path=path,
                relative=relative,
                source=result.transcript_path,
                runner=runner,
                mcp_url=server.mcp_url,
            )
            if result.stderr:
                self._record_text(path, relative, "stderr.log", result.stderr, "Pi stderr")
            policy = manager.frozen_policy
            error = None if policy is not None else "Pi did not freeze a policy candidate"
            outcome = ExplorationOutcome(
                status="valid" if policy is not None else "invalid",
                policy=policy,
                trials=tuple(manager.trials),
                final_text=result.final_text,
                tool_call_count=result.tool_call_count,
                duration_seconds=result.duration_seconds,
                error=error,
            )
            manifest = path / "exploration.json"
            manifest.write_text(
                json.dumps(
                    {
                        "status": outcome.status,
                        "frozen_candidate_id": (
                            manager.frozen_candidate.id
                            if manager.frozen_candidate is not None
                            else None
                        ),
                        "policy_sha256": policy.sha256 if policy is not None else None,
                        "submission_count": manager.accepted_count,
                        "error": error,
                    },
                    indent=2,
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
            self._runtime_artifacts.append(
                _artifact(relative / manifest.name, "Exploration manifest", "application/json")
            )
            return outcome
        finally:
            server.stop()
            shutil.rmtree(path / "working", ignore_errors=True)

    def prepare(
        self,
        policy: PolicyArtifact,
        *,
        memory_path: Path,
        startup_timeout_s: float,
    ) -> PolicyExecutionHandle:
        if startup_timeout_s <= 0:
            raise ValueError("startup_timeout_s must be positive")
        context = multiprocessing.get_context("spawn")
        messages, worker_messages = context.Pipe(duplex=False)
        start_event = context.Event()
        process = context.Process(
            target=_execute_policy_worker,
            args=(
                policy.serialized,
                str(memory_path),
                str(memory_path.parent / "policy-output.log"),
                worker_messages,
                start_event,
            ),
            daemon=True,
        )
        try:
            process.start()
            worker_messages.close()
            if not messages.poll(startup_timeout_s):
                _stop_process(process)
                raise TimeoutError(f"policy startup timed out after {startup_timeout_s:g}s")
            message, error = messages.recv()
            if message != "ready":
                _stop_process(process)
                messages.close()
                raise RuntimeError(error or "policy worker failed before readiness")
            return _PolicyExecutionProcess(
                process,
                messages,
                start_event,
                memory_path.parent / "policy-output.log",
            )
        except BaseException:
            if process.is_alive():
                _stop_process(process)
            messages.close()
            worker_messages.close()
            raise

    def _record_prompt(
        self,
        path: Path,
        relative: Path,
        evaluation_protocol: str,
        task_input: str,
        user_message: str,
    ) -> None:
        components = (
            ("runtime-system.txt", "runtime", SYSTEM_INSTRUCTIONS),
            ("evaluation-protocol.txt", "evaluation", evaluation_protocol),
            ("task-input.txt", "evaluation", task_input),
            ("assembled-user-message.txt", "runtime", user_message),
        )
        manifest_components: list[dict[str, str]] = []
        for filename, owner, value in components:
            target = path / filename
            target.write_text(value, encoding="utf-8")
            reference = _artifact(relative / filename, filename, "text/plain")
            self._prompt_evidence.append(reference)
            self._runtime_artifacts.append(reference)
            manifest_components.append(
                {
                    "path": reference.path,
                    "owner": owner,
                    "sha256": hashlib.sha256(value.encode()).hexdigest(),
                }
            )
        manifest = path / "prompt-assembly.json"
        manifest.write_text(
            json.dumps(
                {
                    "schema_version": "1.0",
                    "runtime_profile": CODE_POLICY_PROFILE,
                    "components": manifest_components,
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        reference = _artifact(relative / manifest.name, "Prompt assembly", "application/json")
        self._prompt_evidence.append(reference)
        self._runtime_artifacts.append(reference)

    def _record_text(
        self,
        path: Path,
        relative: Path,
        filename: str,
        value: str,
        label: str,
    ) -> None:
        if not value:
            return
        (path / filename).write_text(value, encoding="utf-8")
        self._runtime_artifacts.append(_artifact(relative / filename, label, "text/plain"))

    def _record_pi_transcript(
        self,
        *,
        path: Path,
        relative: Path,
        source: Path | None,
        runner: PiCliRunner,
        mcp_url: str,
    ) -> None:
        if source is None:
            return
        transcript = path / "pi-transcript.jsonl"
        shutil.copy2(source, transcript)
        self._runtime_artifacts.append(
            _artifact(relative / transcript.name, "Pi transcript", "application/x-ndjson")
        )
        viewer = path / "pi-transcript.html"
        try:
            runner.export_transcript(
                transcript_path=source,
                output_path=viewer,
                mcp_url=mcp_url,
                api_key=self.api_key,
            )
        except Exception as exc:
            viewer.unlink(missing_ok=True)
            message = f"{type(exc).__name__}: {exc}".replace(self.api_key, "[REDACTED]")
            emit_progress(
                self.progress,
                StatusProgress(channel="pi", message=f"transcript export failed: {message}"),
            )
            self._record_text(
                path,
                relative,
                "pi-transcript-export-error.log",
                message + "\n",
                "Pi transcript export error",
            )
            return
        self._runtime_artifacts.append(
            _artifact(relative / viewer.name, "Pi transcript viewer", "text/html")
        )


class _SubmissionManager:
    def __init__(
        self,
        *,
        workspace: Path,
        path: Path,
        relative_path: Path,
        submit_debug_trial: DebugTrialSubmitter,
        max_submissions: int,
        record_artifact: Any,
    ) -> None:
        self.workspace = workspace
        self.path = path
        self.relative_path = relative_path
        self.submit_debug_trial = submit_debug_trial
        self.max_submissions = max_submissions
        self.record_artifact = record_artifact
        self.candidates: list[PolicyCandidate] = []
        self._policies: dict[str, PolicyArtifact] = {}
        self.frozen_candidate: PolicyCandidate | None = None
        self.accepted_count = 0

    @property
    def trials(self) -> list[TrialRun]:
        return [candidate.trial for candidate in self.candidates]

    @property
    def frozen_policy(self) -> PolicyArtifact | None:
        if self.frozen_candidate is None:
            return None
        return self._policies[self.frozen_candidate.id]

    def submit(self, source: str, serialized: bytes) -> PolicyCandidate:
        if self.frozen_candidate is not None:
            raise PolicyRequestError("submissions are closed after policy freezing")
        if self.accepted_count >= self.max_submissions:
            raise PolicyRequestError(f"submission budget exhausted ({self.max_submissions})")
        try:
            policy_callable = cloudpickle.loads(serialized)
            validate_policy_callable(policy_callable)
        except (EOFError, TypeError, ValueError, pickle.UnpicklingError) as exc:
            raise PolicyRequestError(str(exc)) from exc
        self.accepted_count += 1
        submission_number = self.accepted_count
        relative = self.relative_path / f"submission-{submission_number:04d}"
        submission_path = self.workspace / relative
        submission_path.mkdir()
        source_path = submission_path / "policy.py"
        serialized_path = submission_path / "policy.pkl"
        source_path.write_text(source.rstrip() + "\n", encoding="utf-8")
        serialized_path.write_bytes(serialized)
        policy = PolicyArtifact(
            source=source.rstrip() + "\n",
            serialized=serialized,
            sha256=hashlib.sha256(serialized).hexdigest(),
        )
        self.record_artifact(_artifact(relative / "policy.py", "Policy source", "text/x-python"))
        self.record_artifact(
            _artifact(relative / "policy.pkl", "Serialized policy", "application/octet-stream")
        )
        trial_path = submission_path / "trial"
        trial_path.mkdir()
        trial = self.submit_debug_trial(policy, submission_number, trial_path)
        candidate_id = f"candidate-{submission_number:04d}-{policy.sha256[:12]}"
        candidate = PolicyCandidate(
            id=candidate_id,
            policy=PolicySnapshot(source=policy.source, sha256=policy.sha256),
            evidence=TrialEvidence(
                candidate_id=candidate_id,
                trial=trial,
                remaining_submissions=self.max_submissions - self.accepted_count,
            ),
        )
        self._policies[candidate_id] = policy
        self.candidates.append(candidate)
        return candidate

    def freeze(self, candidate_id: str) -> PolicyCandidate:
        if self.frozen_candidate is not None:
            raise PolicyRequestError("policy candidate is already frozen")
        candidate = next(
            (candidate for candidate in self.candidates if candidate.id == candidate_id), None
        )
        if candidate is None:
            raise PolicyRequestError("candidate does not belong to this exploration")
        self.frozen_candidate = candidate
        return candidate


def _execute_policy_worker(
    serialized: bytes,
    memory_path: str,
    output_path: str,
    messages: Any,
    start_event: Any,
) -> None:
    with _BoundedPolicyWriter(Path(output_path)) as output:
        try:
            try:
                policy = cloudpickle.loads(serialized)
            except Exception as exc:
                messages.send(
                    ("infrastructure_error", f"policy load failed: {type(exc).__name__}: {exc}")
                )
                return
            try:
                # Keep process-heavy runtime imports inside the spawned worker.
                from dimos.memory2.store.sqlite import SqliteStore
                from dimos.porcelain.dimos import Dimos

                memory = SqliteStore(path=memory_path, must_exist=True, read_only=True)
                memory.start()
                app = Dimos.connect(memory=memory)
            except Exception as exc:
                messages.send(
                    (
                        "infrastructure_error",
                        f"DimOS connection failed: {type(exc).__name__}: {exc}",
                    )
                )
                return
            messages.send(("ready", None))
            start_event.wait()
            try:
                with redirect_stdout(output), redirect_stderr(output):
                    result = policy(app)
                if result is not None:
                    raise TypeError("policy(app) must return None")
            except Exception as exc:
                messages.send(("policy_error", f"{type(exc).__name__}: {exc}"))
            else:
                messages.send(("completed", None))
            finally:
                app.stop()
        finally:
            messages.close()


class _PolicyExecutionProcess:
    def __init__(
        self,
        process: Any,
        messages: Any,
        start_event: Any,
        output_path: Path,
    ) -> None:
        self._process = process
        self._messages = messages
        self._start_event = start_event
        self._output_path = output_path
        self._started_at: float | None = None
        self._finished: PolicyExecution | None = None

    def start(self) -> None:
        if self._started_at is not None:
            raise RuntimeError("policy execution already started")
        self._started_at = time.monotonic()
        self._start_event.set()

    def finish(self, *, grace_s: float = 1.0) -> PolicyExecution:
        if grace_s < 0:
            raise ValueError("grace_s must be non-negative")
        if self._finished is not None:
            return self._finished
        if self._started_at is None:
            raise RuntimeError("policy execution has not started")
        if self._messages.poll(grace_s):
            try:
                status, error = self._messages.recv()
            except EOFError:
                status = "infrastructure_error"
                error = "policy worker closed its result channel without a result"
            self._process.join(grace_s)
            if self._process.is_alive():
                _stop_process(self._process)
        else:
            stopped = self._process.is_alive()
            if stopped:
                _stop_process(self._process)
                status, error = "stopped", None
            else:
                self._process.join()
                status = "infrastructure_error"
                error = f"policy worker exited with code {self._process.exitcode} without a result"
        self._messages.close()
        output = self._output_path.read_text(encoding="utf-8")
        self._finished = PolicyExecution(
            status=status,
            duration_seconds=time.monotonic() - self._started_at,
            error=error,
            output=output,
        )
        return self._finished


class _BoundedPolicyWriter(io.TextIOBase):
    def __init__(self, path: Path) -> None:
        self._path = path
        self._handle: TextIO | None = None
        self._remaining = MAX_POLICY_OUTPUT
        self._truncated = False

    def __enter__(self) -> _BoundedPolicyWriter:
        self._handle = self._path.open("w", encoding="utf-8")
        return self

    def __exit__(
        self,
        *_args: object,
    ) -> None:
        if self._handle is not None:
            self._handle.close()

    def write(self, value: str) -> int:
        if self._handle is None:
            raise RuntimeError("policy output is not open")
        if self._remaining <= 0:
            if not self._truncated:
                self._handle.write("\n... [policy output truncated]")
                self._handle.flush()
                self._truncated = True
            return len(value)
        if len(value) <= self._remaining:
            written = value
        else:
            written = value[: self._remaining] + "\n... [policy output truncated]"
            self._truncated = True
        self._handle.write(written)
        self._handle.flush()
        self._remaining -= len(written)
        return len(value)

    def flush(self) -> None:
        if self._handle is not None:
            self._handle.flush()


def _stop_process(process: Any) -> None:
    if not process.is_alive():
        process.join()
        return
    process.terminate()
    process.join(5)
    if process.is_alive():
        process.kill()
        process.join()


def _assemble_user_message(evaluation_protocol: str, task_input: str) -> str:
    return (
        "# Evaluation protocol\n\n"
        f"{evaluation_protocol.strip()}\n\n"
        "# Task input\n\n"
        f"{task_input.strip()}\n"
    )


def _artifact(path: Path, label: str, media_type: str) -> ArtifactReference:
    return ArtifactReference(path=path.as_posix(), label=label, media_type=media_type)


def _pi_paths() -> tuple[Path, Path]:
    package = Path(__file__).resolve().parents[3] / "packages" / "pi-code-policy-extension"
    cli = package / "node_modules" / "@earendil-works" / "pi-coding-agent" / "dist" / "cli.js"
    extension = package / "dist" / "python-exec.js"
    return cli, extension
