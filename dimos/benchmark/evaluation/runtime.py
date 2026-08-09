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

"""The versioned Pi implementation of the CodePolicy agent runtime."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
from typing import Literal

from pydantic import BaseModel

from dimos.agents.code_policy_core import (
    CodePolicyEnvironment,
    CodePolicySessionConfig,
    FrozenMemoryEnvironment,
    LiveDimosEnvironment,
    latest_policy_source,
    validate_policy_source,
)
from dimos.agents.code_policy_server import CodePolicyMcpServer
from dimos.benchmark.evaluation.models import (
    ArtifactReference,
    CodePolicyAgentConfig,
    RuntimeIdentity,
)
from dimos.benchmark.evaluation.pi_process import (
    PI_VERSION,
    PiCliRunner,
    PiRunError,
    PiRunResult,
)
from dimos.benchmark.evaluation.progress import ProgressSink
from dimos.benchmark.evaluation.protocol import (
    AgentOutcome,
    PolicyOutcome,
    PolicyValidation,
)

CODE_POLICY_PROFILE = "code-policy-v1"
TURN_TIMEOUT_SECONDS = 600.0
SYSTEM_INSTRUCTIONS = """You are a CodePolicy agent.

Use the single `python_exec` tool to solve the supplied task. Python executes in a
persistent trusted, unsandboxed environment, so imports, variables, and functions
persist between tool calls. Follow the evaluation protocol exactly.
"""
POLICY_SYSTEM_INSTRUCTIONS = (
    SYSTEM_INSTRUCTIONS
    + """
Author the requested result as one self-contained Python cell containing a
module-level synchronous `def policy()` with no parameters. The cell must include
all imports, constants, and helper functions it needs. You may redefine and test
`policy()` freely; the latest successful conforming cell is the submitted policy.
The policy return value must be JSON-serializable.
"""
)


class CodePolicyRuntimeFactory:
    """Create evaluation-owned sessions with one fixed CodePolicy profile."""

    def __init__(
        self,
        *,
        config: CodePolicyAgentConfig,
        api_key: str,
        workspace: Path,
        progress: ProgressSink | None = None,
    ) -> None:
        self.config = config
        self.api_key = api_key
        self.workspace = workspace
        self.progress = progress
        self._session_count = 0
        self._prompt_evidence: list[ArtifactReference] = []
        self._runtime_artifacts: list[ArtifactReference] = []

    @property
    def identity(self) -> RuntimeIdentity:
        return RuntimeIdentity(
            profile=self.config.profile,
            driver_version=PI_VERSION,
            model=self.config.model,
            thinking_level=self.config.thinking_level,
        )

    @property
    def prompt_evidence(self) -> tuple[ArtifactReference, ...]:
        return tuple(self._prompt_evidence)

    @property
    def runtime_artifacts(self) -> tuple[ArtifactReference, ...]:
        return tuple(self._runtime_artifacts)

    def open_session(self, environment: BaseModel) -> CodePolicyRuntimeSession:
        if not isinstance(environment, (FrozenMemoryEnvironment, LiveDimosEnvironment)):
            raise TypeError(f"Unsupported CodePolicy environment: {type(environment).__name__}")
        self._session_count += 1
        session_path = Path("runtime") / f"session-{self._session_count:04d}"
        return CodePolicyRuntimeSession(
            factory=self,
            environment=environment,
            relative_path=session_path,
        )

    def _record_prompt_evidence(self, references: list[ArtifactReference]) -> None:
        self._prompt_evidence.extend(references)
        self._runtime_artifacts.extend(references)

    def _record_runtime_artifact(self, reference: ArtifactReference) -> None:
        self._runtime_artifacts.append(reference)


class CodePolicyRuntimeSession:
    """One lifecycle-bounded, single-turn CodePolicy interaction."""

    def __init__(
        self,
        *,
        factory: CodePolicyRuntimeFactory,
        environment: CodePolicyEnvironment,
        relative_path: Path,
    ) -> None:
        self.factory = factory
        self.environment = environment
        self.relative_path = relative_path
        self.path = factory.workspace / relative_path
        self.server: CodePolicyMcpServer | None = None
        self._ran = False

    def __enter__(self) -> CodePolicyRuntimeSession:
        self.path.mkdir(parents=True)
        self.server = CodePolicyMcpServer(CodePolicySessionConfig(environment=self.environment))
        self.server.start()
        return self

    def __exit__(self, *_args: object) -> None:
        if self.server is not None:
            self.server.stop()
            self.server = None
        shutil.rmtree(self.path / "working", ignore_errors=True)

    def answer(self, *, evaluation_protocol: str, task_input: str) -> AgentOutcome:
        self._consume(evaluation_protocol, task_input)
        user_message = _assemble_user_message(evaluation_protocol, task_input)
        evidence = self._write_prompt_evidence(
            evaluation_protocol,
            task_input,
            user_message,
            SYSTEM_INSTRUCTIONS,
        )
        self.factory._record_prompt_evidence(evidence)
        result = self._run_pi_round(
            prompt=user_message,
            system_prompt=SYSTEM_INSTRUCTIONS,
            round_number=1,
            policy_mode=False,
        )
        return AgentOutcome(
            final_text=result.final_text,
            tool_call_count=result.tool_call_count,
            duration_seconds=result.duration_seconds,
        )

    def author_policy(
        self,
        *,
        evaluation_protocol: str,
        task_input: str,
        max_rounds: int,
    ) -> PolicyOutcome:
        if max_rounds < 1:
            raise ValueError("max_rounds must be positive")
        self._consume(evaluation_protocol, task_input)
        user_message = _assemble_user_message(evaluation_protocol, task_input)
        evidence = self._write_prompt_evidence(
            evaluation_protocol,
            task_input,
            user_message,
            POLICY_SYSTEM_INSTRUCTIONS,
        )
        self.factory._record_prompt_evidence(evidence)

        assert self.server is not None
        validations: list[PolicyValidation] = []
        source: str | None = None
        result_value = None
        final_text = ""
        tool_call_count = 0
        duration_seconds = 0.0
        prompt = user_message
        for round_number in range(1, max_rounds + 1):
            result = self._run_pi_round(
                prompt=prompt,
                system_prompt=POLICY_SYSTEM_INSTRUCTIONS,
                round_number=round_number,
                policy_mode=True,
            )
            final_text = result.final_text
            tool_call_count += result.tool_call_count
            duration_seconds += result.duration_seconds
            source = latest_policy_source(self.server.session.execution_records)
            if source is None:
                validation = PolicyValidation(
                    round_number=round_number,
                    valid=False,
                    candidate_found=False,
                    error="Define a module-level synchronous zero-argument policy() function.",
                    duration_seconds=0.0,
                )
            else:
                executed = validate_policy_source(self.server.session.config, source)
                duration_seconds += executed.duration_seconds
                validation = PolicyValidation(
                    round_number=round_number,
                    valid=executed.valid,
                    candidate_found=True,
                    error=executed.error,
                    duration_seconds=executed.duration_seconds,
                )
                if executed.valid:
                    result_value = executed.result
            validations.append(validation)
            if validation.valid:
                break
            if round_number < max_rounds:
                assert validation.error is not None
                prompt = _assemble_repair_message(
                    evaluation_protocol=evaluation_protocol,
                    task_input=task_input,
                    source=source,
                    error=validation.error,
                )

        status: Literal["valid", "invalid"] = "valid" if validations[-1].valid else "invalid"
        self._write_policy_artifacts(source, validations, status, result_value)
        return PolicyOutcome(
            status=status,
            source=source,
            result=result_value,
            validations=tuple(validations),
            final_text=final_text,
            tool_call_count=tool_call_count,
            duration_seconds=duration_seconds,
        )

    def _consume(self, evaluation_protocol: str, task_input: str) -> None:
        if self.server is None:
            raise RuntimeError("CodePolicy session must be entered before use")
        if self._ran:
            raise RuntimeError("code-policy-v1 sessions accept exactly one top-level operation")
        if not evaluation_protocol.strip() or not task_input.strip():
            raise ValueError("evaluation protocol and task input must be non-empty")
        self._ran = True

    def _run_pi_round(
        self,
        *,
        prompt: str,
        system_prompt: str,
        round_number: int,
        policy_mode: bool,
    ) -> PiRunResult:
        assert self.server is not None
        working = self.path / "working" / f"round-{round_number:04d}"
        working.mkdir(parents=True)
        cli, extension = _pi_paths()
        runner = PiCliRunner(
            cli=cli,
            extension=extension,
            model=self.factory.config.model,
            thinking_level=self.factory.config.thinking_level,
            timeout_s=TURN_TIMEOUT_SECONDS,
            progress=self.factory.progress,
        )
        try:
            result = runner.run(
                prompt=prompt,
                system_prompt=system_prompt,
                mcp_url=self.server.mcp_url,
                api_key=self.factory.api_key,
                run_dir=working,
            )
        except PiRunError as exc:
            self._record_stderr(exc.stderr, round_number if policy_mode else None)
            raise
        if result.transcript_path is not None:
            filename = (
                f"pi-transcript-round-{round_number:04d}.jsonl"
                if policy_mode
                else "pi-transcript.jsonl"
            )
            target = self.path / filename
            shutil.copy2(result.transcript_path, target)
            self.factory._record_runtime_artifact(
                self._artifact(target, "Pi transcript", "application/x-ndjson")
            )
        if result.stderr:
            self._record_stderr(result.stderr, round_number if policy_mode else None)
        return result

    def _record_stderr(self, stderr: str, round_number: int | None = None) -> None:
        if not stderr:
            return
        filename = "stderr.log" if round_number is None else f"stderr-round-{round_number:04d}.log"
        target = self.path / filename
        target.write_text(stderr, encoding="utf-8")
        self.factory._record_runtime_artifact(self._artifact(target, "Pi stderr", "text/plain"))

    def _write_prompt_evidence(
        self,
        evaluation_protocol: str,
        task_input: str,
        user_message: str,
        system_instructions: str,
    ) -> list[ArtifactReference]:
        components = (
            ("runtime-system.txt", "runtime", system_instructions),
            ("evaluation-protocol.txt", "evaluation", evaluation_protocol),
            ("task-input.txt", "evaluation", task_input),
            ("assembled-user-message.txt", "runtime", user_message),
        )
        manifest_components: list[dict[str, str]] = []
        references: list[ArtifactReference] = []
        for filename, owner, text in components:
            path = self.path / filename
            path.write_text(text, encoding="utf-8")
            manifest_components.append(
                {
                    "path": path.relative_to(self.factory.workspace).as_posix(),
                    "owner": owner,
                    "sha256": hashlib.sha256(text.encode()).hexdigest(),
                }
            )
            references.append(self._artifact(path, filename, "text/plain"))
        manifest = self.path / "prompt-assembly.json"
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
        references.append(self._artifact(manifest, "Prompt assembly", "application/json"))
        return references

    def _write_policy_artifacts(
        self,
        source: str | None,
        validations: list[PolicyValidation],
        status: str,
        result: object,
    ) -> None:
        if source is not None:
            policy = self.path / "policy.py"
            policy.write_text(source.rstrip() + "\n", encoding="utf-8")
            self.factory._record_runtime_artifact(
                self._artifact(policy, "Policy source", "text/x-python")
            )
        history = self.path / "policy-validation.json"
        history.write_text(
            json.dumps(
                [
                    {
                        "round": item.round_number,
                        "valid": item.valid,
                        "candidate_found": item.candidate_found,
                        "error": item.error,
                        "duration_seconds": item.duration_seconds,
                    }
                    for item in validations
                ],
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        self.factory._record_runtime_artifact(
            self._artifact(history, "Policy validation history", "application/json")
        )
        if status == "valid":
            result_path = self.path / "policy-result.json"
            result_path.write_text(
                json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            self.factory._record_runtime_artifact(
                self._artifact(result_path, "Policy result", "application/json")
            )

    def _artifact(self, path: Path, label: str, media_type: str) -> ArtifactReference:
        return ArtifactReference(
            path=path.relative_to(self.factory.workspace).as_posix(),
            label=label,
            media_type=media_type,
        )


def _assemble_user_message(evaluation_protocol: str, task_input: str) -> str:
    return (
        "# Evaluation protocol\n\n"
        f"{evaluation_protocol.strip()}\n\n"
        "# Task input\n\n"
        f"{task_input.strip()}\n"
    )


def _assemble_repair_message(
    *,
    evaluation_protocol: str,
    task_input: str,
    source: str | None,
    error: str,
) -> str:
    candidate = source if source is not None else "(no conforming policy candidate was found)"
    return (
        _assemble_user_message(evaluation_protocol, task_input)
        + "\n# Mechanical validation failure\n\n"
        + f"```text\n{error}\n```\n\n"
        + "# Current candidate\n\n"
        + f"```python\n{candidate}\n```\n\n"
        + "Redefine policy() in one self-contained cell and finish when it is ready.\n"
    )


def _pi_paths() -> tuple[Path, Path]:
    package = Path(__file__).resolve().parents[3] / "packages" / "pi-code-policy-extension"
    cli = package / "node_modules" / "@earendil-works" / "pi-coding-agent" / "dist" / "cli.js"
    extension = package / "dist" / "python-exec.js"
    return cli, extension
