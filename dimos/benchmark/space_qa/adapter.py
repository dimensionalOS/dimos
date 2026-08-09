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

"""What an external benchmark implements to reach the agent-evaluation path."""

from __future__ import annotations

from collections.abc import Iterable
import hashlib
from typing import Protocol, runtime_checkable

from pydantic import BaseModel, ConfigDict, Field, model_validator

from dimos.benchmark.agent_eval.models import (
    EvalCase,
    ExternalEvaluatorRef,
    NoEnvironmentSource,
    VerbatimPromptTask,
)


class BenchmarkModel(BaseModel):
    """Strict immutable base for the records an adapter passes around."""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


class BenchmarkItem(BenchmarkModel):
    """One upstream question, addressed by its row position in the upstream file.

    Deliberately carries no answer field: every field here is allowed to reach
    the agent verbatim, so the grading key must not travel on this record. That
    is the whole guarantee — keeping the key out of the process the agent runs
    in is the execution path's business, not this type's.
    """

    ordinal: int = Field(ge=0)
    question: str = Field(min_length=1)

    @property
    def question_sha256(self) -> str:
        """Derived, never supplied, so it cannot drift from the question it names."""
        return hashlib.sha256(self.question.encode("utf-8")).hexdigest()


class SubsetSpec(BenchmarkModel):
    """Which slice of a suite to run, reproducibly."""

    seed: int
    groups: int = Field(ge=1)


class ItemScore(BenchmarkModel):
    """One graded reply. Both optionals are None when the reply did not parse."""

    ordinal: int = Field(ge=0)
    parsed_answer: int | None = None
    correct: bool | None = None

    @model_validator(mode="after")
    def unparsed_stays_ungraded(self) -> ItemScore:
        if (self.parsed_answer is None) != (self.correct is None):
            raise ValueError("an unparsed reply must stay ungraded, and a graded one must parse")
        return self


@runtime_checkable
class BenchmarkAdapter(Protocol):
    """One external benchmark wired into the agent-eval execution path."""

    name: str
    revision: str

    def iter_items(self, subset: SubsetSpec) -> Iterable[BenchmarkItem]: ...

    def to_case(self, item: BenchmarkItem) -> EvalCase: ...

    def score(self, item: BenchmarkItem, raw_final_text: str) -> ItemScore: ...


def build_case(adapter: BenchmarkAdapter, item: BenchmarkItem) -> EvalCase:
    """The one shape an adapted item takes: no environment, prompt verbatim, graded upstream.

    The case id carries the row it came from and a digest of the exact question
    text, so a result can be traced back and a shifted upstream file cannot pass
    itself off as the sampled one.
    """
    return EvalCase(
        case_id=f"{adapter.name}-{item.ordinal:05d}-{item.question_sha256[:8]}",
        source=NoEnvironmentSource(),
        task=VerbatimPromptTask(prompt=item.question),
        validator=ExternalEvaluatorRef(benchmark=adapter.name, revision=adapter.revision),
    )
