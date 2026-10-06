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

"""One frozen SPACE task adapted to DimOS cases, with no acquisition on import."""

from __future__ import annotations

from collections.abc import Iterator, Sequence
from dataclasses import dataclass
from typing import overload

from dimos.evals.agents.base import Agent
from dimos.evals.agents.text_question import TextQuestion
from dimos.evals.environments.prompt import Prompt
from dimos.evals.suites.lib.space.constants import SELECTED_INDICES, SMOKE_INDEX
from dimos.evals.suites.lib.space.data import Example, SpacePaths, load_examples
from dimos.evals.suites.lib.space.upstream import OfficialSpace
from dimos.evals.types import EvalCase, Outcome

CASE_TIMEOUT_S = 60.0


class SpacePrompt(Prompt):
    def preflight(self, agent: Agent) -> None:
        super().preflight(agent)
        if not isinstance(agent, TextQuestion):
            raise ValueError("SPACE text QA requires the tool-free TextQuestion agent")
        if agent.config.system_prompt:
            raise ValueError("SPACE questions must be sent without an added system prompt")


@dataclass(frozen=True)
class Grade:
    scorer: OfficialSpace
    example: Example

    def __call__(self, outcome: Outcome) -> float:
        if outcome.trajectory.extra.ended_by != "answer":
            raise RuntimeError(f"SPACE case did not complete: {outcome.trajectory.extra.ended_by}")
        return self.scorer.score(self.example.qa, outcome.trajectory.final_answer).accuracy / 100.0


def load_cases(paths: SpacePaths, *, smoke: bool = False) -> tuple[EvalCase, ...]:
    examples = load_examples(paths, (SMOKE_INDEX,) if smoke else SELECTED_INDICES)
    scorer = OfficialSpace(paths.source)
    scorer.prepare()
    return tuple(
        EvalCase(
            id=example.case_id,
            inputs=example.question,
            environment=SpacePrompt(),
            grade=Grade(scorer, example),
            tags=frozenset({"space", "text", "map_sketching"}),
            timeout_s=CASE_TIMEOUT_S,
        )
        for example in examples
    )


class SpaceSuite(Sequence[EvalCase]):
    """Keep module discovery cheap; missing external data fails at execution."""

    def __len__(self) -> int:
        return len(SELECTED_INDICES)

    def __iter__(self) -> Iterator[EvalCase]:
        return iter(load_cases(SpacePaths()))

    @overload
    def __getitem__(self, index: int) -> EvalCase: ...

    @overload
    def __getitem__(self, index: slice) -> Sequence[EvalCase]: ...

    def __getitem__(self, index: int | slice) -> EvalCase | Sequence[EvalCase]:
        return load_cases(SpacePaths())[index]
