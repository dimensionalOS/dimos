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

"""The SPACE text-QA suite as an adapter.

What this class scores is bookkeeping, not the verdict. A run's score is the
``results.json`` that ``space.evaluate_qas.main`` writes: SPACE drives its own
loop, applies its own parser and keeps its own answer key. This adapter grades
the same replies a second time — through that same upstream parser, never a
local reimplementation — so ``run_space_task`` can hold the two records against
each other and fail the run when they disagree.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any

from dimos.benchmark.agent_eval.models import EvalCase
from dimos.benchmark.space_qa.adapter import BenchmarkItem, ItemScore, SubsetSpec, build_case
from dimos.benchmark.space_qa.data import SpaceRow, load_task_rows
from dimos.benchmark.space_qa.sampling import sample_group_rows
from dimos.benchmark.space_qa.source import SPACE_REVISION
from dimos.benchmark.space_qa.tasks import SpaceTextTask


class SpaceQAAdapter:
    """One SPACE text task, addressed by row position in its own ``qas.json``."""

    def __init__(self, task: SpaceTextTask, rows: Sequence[SpaceRow]) -> None:
        if len(rows) != task.expected_rows:
            raise ValueError(
                f"{task.name} expects {task.expected_rows} rows, got {len(rows)}",
            )
        self.name = f"space-{task.name}"
        self.revision = SPACE_REVISION
        self._task = task
        self._rows = tuple(rows)

    @classmethod
    def from_data_root(cls, task: SpaceTextTask, data_root: Path) -> SpaceQAAdapter:
        return cls(task, load_task_rows(task, data_root))

    def iter_items(self, subset: SubsetSpec) -> tuple[BenchmarkItem, ...]:
        ordinals = sample_group_rows(
            len(self._rows),
            group_size=self._task.group_size,
            groups=subset.groups,
            seed=subset.seed,
        )
        return tuple(
            BenchmarkItem(ordinal=ordinal, question=str(self._rows[ordinal]["question"]))
            for ordinal in ordinals
        )

    def to_case(self, item: BenchmarkItem) -> EvalCase:
        return build_case(self, item)

    def score(self, item: BenchmarkItem, raw_final_text: str) -> ItemScore:
        """Grade one reply the way ``evaluate_qas`` does: upstream parse, then exact match."""
        expected = int(self._row(item.ordinal)["answer"])
        parsed = parse_official_answer(raw_final_text)
        if parsed is None:
            return ItemScore(ordinal=item.ordinal)
        return ItemScore(ordinal=item.ordinal, parsed_answer=parsed, correct=parsed == expected)

    def upstream_rows(self, items: Iterable[BenchmarkItem]) -> list[SpaceRow]:
        """The selected rows, verbatim, for the subset file SPACE itself reads."""
        return [dict(self._row(item.ordinal)) for item in items]

    def _row(self, ordinal: int) -> SpaceRow:
        if not 0 <= ordinal < len(self._rows):
            raise KeyError(f"ordinal {ordinal} was not issued by {self.name}")
        return self._rows[ordinal]


def parse_official_answer(raw_final_text: str) -> int | None:
    """Read a reply with SPACE's own parser, so this side cannot drift from the scorer.

    ``QA_Agent.parse_answer_from_response`` reads no instance state, so a bare
    instance reuses it verbatim without running ``__init__``, which builds a
    live API client from a key this path never holds. The parser can also
    return a non-integer answer; SPACE would score that as a miss, and here it
    is reported as unparsed — the per-question record keeps the raw value.
    """
    from space.agents.qa_agent import QA_Agent  # type: ignore[import-not-found]

    parser: Any = object.__new__(QA_Agent)
    answer = parser.parse_answer_from_response(raw_final_text)
    if isinstance(answer, bool) or not isinstance(answer, int):
        return None
    return int(answer)
