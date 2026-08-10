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

from dimos.benchmark.evaluation.models import EvaluationReport
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
    status: Literal["completed", "policy_error", "timed_out", "infrastructure_error"]
    duration_seconds: float
    error: str | None = None


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


@dataclass(frozen=True)
class ExplorationOutcome:
    status: Literal["valid", "invalid"]
    policy: PolicyArtifact | None
    trials: tuple[TrialRun, ...]
    final_text: str
    tool_call_count: int
    duration_seconds: float
    error: str | None = None


@runtime_checkable
class CodePolicyRuntime(Protocol):
    """Fixed Pi runtime injected into complete native evaluations."""

    def explore(
        self,
        *,
        evaluation_protocol: str,
        task_input: str,
        submit_debug_trial: DebugTrialSubmitter,
        max_submissions: int = 5,
    ) -> ExplorationOutcome: ...

    def execute(
        self,
        policy: PolicyArtifact,
        *,
        timeout_s: float,
    ) -> PolicyExecution: ...


@dataclass(frozen=True)
class EvaluationContext:
    run_id: str
    spec_dir: Path
    workspace: Path
    runtime: CodePolicyRuntime
    progress: ProgressSink | None


@runtime_checkable
class Evaluation(Protocol):
    """A complete benchmark integration with native result semantics."""

    name: str
    config_model: type[BaseModel]

    def run(self, config: BaseModel, context: EvaluationContext) -> EvaluationReport: ...
