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

from pydantic import BaseModel

from dimos.agents.code_policy_core import (
    CodePolicyEnvironment,
    CodePolicySessionConfig,
    FrozenMemoryEnvironment,
    LiveDimosEnvironment,
)
from dimos.agents.code_policy_server import CodePolicyMcpServer
from dimos.benchmark.evaluation.models import (
    ArtifactReference,
    CodePolicyAgentConfig,
    RuntimeIdentity,
)
from dimos.benchmark.evaluation.pi_process import PI_VERSION, PiCliRunner, PiRunError
from dimos.benchmark.evaluation.progress import ProgressSink
from dimos.benchmark.evaluation.protocol import AgentOutcome

CODE_POLICY_PROFILE = "code-policy-v1"
TURN_TIMEOUT_SECONDS = 600.0
SYSTEM_INSTRUCTIONS = """You are a CodePolicy agent.

Use the single `python_exec` tool to solve the supplied task. Python executes in a
persistent trusted, unsandboxed environment, so imports, variables, and functions
persist between tool calls. Follow the evaluation protocol exactly.
"""


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

    def run(self, *, evaluation_protocol: str, task_input: str) -> AgentOutcome:
        if self.server is None:
            raise RuntimeError("CodePolicy session must be entered before run()")
        if self._ran:
            raise RuntimeError("code-policy-v1 sessions accept exactly one initial turn")
        if not evaluation_protocol.strip() or not task_input.strip():
            raise ValueError("evaluation protocol and task input must be non-empty")
        self._ran = True

        user_message = _assemble_user_message(evaluation_protocol, task_input)
        evidence = self._write_prompt_evidence(evaluation_protocol, task_input, user_message)
        self.factory._record_prompt_evidence(evidence)
        working = self.path / "working"
        working.mkdir()
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
                prompt=user_message,
                system_prompt=SYSTEM_INSTRUCTIONS,
                mcp_url=self.server.mcp_url,
                api_key=self.factory.api_key,
                run_dir=working,
            )
        except PiRunError as exc:
            self._record_stderr(exc.stderr)
            raise
        if result.transcript_path is not None:
            target = self.path / "pi-transcript.jsonl"
            shutil.copy2(result.transcript_path, target)
            self.factory._record_runtime_artifact(
                self._artifact(target, "Pi transcript", "application/x-ndjson")
            )
        if result.stderr:
            self._record_stderr(result.stderr)
        return AgentOutcome(
            final_text=result.final_text,
            tool_call_count=result.tool_call_count,
            duration_seconds=result.duration_seconds,
        )

    def _record_stderr(self, stderr: str) -> None:
        if not stderr:
            return
        target = self.path / "stderr.log"
        target.write_text(stderr, encoding="utf-8")
        self.factory._record_runtime_artifact(self._artifact(target, "Pi stderr", "text/plain"))

    def _write_prompt_evidence(
        self,
        evaluation_protocol: str,
        task_input: str,
        user_message: str,
    ) -> list[ArtifactReference]:
        components = (
            ("runtime-system.txt", "runtime", SYSTEM_INSTRUCTIONS),
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


def _pi_paths() -> tuple[Path, Path]:
    package = Path(__file__).resolve().parents[3] / "packages" / "pi-code-policy-extension"
    cli = package / "node_modules" / "@earendil-works" / "pi-coding-agent" / "dist" / "cli.js"
    extension = package / "dist" / "python-exec.js"
    return cli, extension
