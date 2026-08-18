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

"""Public contracts for native evaluations and CodePolicy execution."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import json
from pathlib import Path
from typing import TYPE_CHECKING, Literal, Protocol, runtime_checkable

from pydantic import BaseModel

from dimos.benchmark.evaluation.models import (
    ArtifactReference,
    EvaluationReport,
    RuntimeIdentity,
)
from dimos.benchmark.evaluation.progress import ProgressSink

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store


@dataclass(frozen=True)
class PolicyArtifact:
    """A task-level callable captured during exploration."""

    source_path: Path
    serialized_path: Path
    sha256: str


@dataclass(frozen=True)
class PolicyExecution:
    status: Literal["completed", "policy_error", "stopped", "infrastructure_error"]
    duration_seconds: float
    error: str | None
    output: str


@dataclass(frozen=True)
class TrialOutcome:
    success: bool
    reward: float | None
    status: Literal["completed", "policy_error", "timed_out", "infrastructure_error"]
    error: str | None
    duration_seconds: float


@dataclass(frozen=True)
class TrialRun:
    """Read-only postmortem handle for one fully stopped policy trial."""

    run_id: str
    outcome: TrialOutcome
    artifacts: Path
    log_path: Path
    memory_path: Path
    policy_output: str

    def read_logs(
        self,
        *,
        module: str | None = None,
        tail: int | None = None,
    ) -> tuple[dict[str, object], ...]:
        from dimos.core.log_viewer import read_log

        records = [json.loads(line) for line in read_log(self.log_path, count=None)]
        if module is not None:
            records = [record for record in records if record.get("module") == module]
        if tail is not None:
            if tail < 0:
                raise ValueError("tail must be non-negative")
            records = records[-tail:] if tail else []
        return tuple(records)

    def open_memory(self) -> Store:
        """Open the completed trial's Memory2 recording read-only."""
        from dimos.memory2.store.sqlite import SqliteStore

        return SqliteStore(path=str(self.memory_path), must_exist=True, read_only=True)


DebugTrialSubmitter = Callable[[PolicyArtifact, int, Path], TrialRun]


@runtime_checkable
class EvaluationRuntime(Protocol):
    """Evaluated-behavior runtime selected by an Evaluation."""

    @property
    def identity(self) -> RuntimeIdentity: ...

    @property
    def prompt_evidence(self) -> tuple[ArtifactReference, ...]: ...

    @property
    def runtime_artifacts(self) -> tuple[ArtifactReference, ...]: ...


@dataclass(frozen=True)
class ExplorationOutcome:
    status: Literal["valid", "invalid"]
    policy: PolicyArtifact | None
    trials: tuple[TrialRun, ...]
    final_text: str
    tool_call_count: int
    duration_seconds: float
    error: str | None = None


@dataclass(frozen=True)
class LiveAgentOutcome:
    final_text: str
    tool_call_count: int
    duration_seconds: float


@runtime_checkable
class LiveAgentExecutionHandle(Protocol):
    """A prepared live model session waiting behind an evaluator start gate."""

    def start(self) -> None: ...

    def failure(self) -> BaseException | None: ...

    def finish(self) -> LiveAgentOutcome: ...


@runtime_checkable
class LiveAgentRuntime(EvaluationRuntime, Protocol):
    def prepare(
        self,
        *,
        prompt: str,
        system_prompt: str,
        memory_path: Path,
        episode_timeout_s: float,
    ) -> LiveAgentExecutionHandle: ...


@runtime_checkable
class CodePolicyRuntime(EvaluationRuntime, Protocol):
    """Fixed Pi runtime injected into complete native evaluations."""

    def explore(
        self,
        *,
        evaluation_protocol: str,
        task_input: str,
        submit_debug_trial: DebugTrialSubmitter,
        max_submissions: int = 5,
    ) -> ExplorationOutcome: ...

    def prepare(
        self,
        policy: PolicyArtifact,
        *,
        memory_path: Path,
        startup_timeout_s: float,
    ) -> PolicyExecutionHandle: ...


@runtime_checkable
class PolicyExecutionHandle(Protocol):
    """A loaded policy waiting behind an evaluator-owned start gate."""

    def start(self) -> None: ...

    def finish(self, *, grace_s: float = 1.0) -> PolicyExecution: ...


@dataclass(frozen=True)
class EvaluationContext:
    run_id: str
    spec_dir: Path
    workspace: Path
    runtime: EvaluationRuntime
    progress: ProgressSink | None


@runtime_checkable
class Evaluation(Protocol):
    """A complete benchmark integration with native result semantics."""

    name: str
    runtime_profile: Literal["code-policy-v1", "live-agent-v1"]
    config_model: type[BaseModel]

    def run(self, config: BaseModel, context: EvaluationContext) -> EvaluationReport: ...
