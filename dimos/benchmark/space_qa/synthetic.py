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

"""A generated multiple-choice suite: a second adapter that needs no data at all."""

from __future__ import annotations

import json
import random
import string

from dimos.benchmark.agent_eval.models import EvalCase
from dimos.benchmark.space_qa.adapter import (
    BenchmarkItem,
    ItemScore,
    SubsetSpec,
    build_case,
)
from dimos.benchmark.space_qa.sampling import DEFAULT_GROUP_SIZE, sample_group_rows

GRID_SIDE = 3
DEFAULT_TOTAL_GROUPS = 5


class SyntheticGridAdapter:
    """Grid questions generated from a seed, in the row-group shape real suites have.

    Nothing is downloaded and nothing is read from disk, so this suite runs in
    the default test tier; `revision` pins the generator that produced it.
    """

    name: str = "synthetic-grid"
    revision: str = "synthetic-v1"

    def __init__(self, *, seed: int = 0, total_groups: int = DEFAULT_TOTAL_GROUPS) -> None:
        if total_groups < 1:
            raise ValueError(f"total_groups must be at least 1, got {total_groups}")
        rng = random.Random(seed)
        rows = [row for _ in range(total_groups) for row in _generate_group(rng)]
        self._questions = tuple(question for question, _answer in rows)
        self._answers = tuple(answer for _question, answer in rows)

    def iter_items(self, subset: SubsetSpec) -> tuple[BenchmarkItem, ...]:
        ordinals = sample_group_rows(len(self._questions), groups=subset.groups, seed=subset.seed)
        return tuple(
            BenchmarkItem(ordinal=ordinal, question=self._questions[ordinal])
            for ordinal in ordinals
        )

    def to_case(self, item: BenchmarkItem) -> EvalCase:
        return build_case(self, item)

    def score(self, item: BenchmarkItem, raw_final_text: str) -> ItemScore:
        expected = self._expected(item.ordinal)
        parsed = parse_json_answer(raw_final_text)
        if parsed is None:
            return ItemScore(ordinal=item.ordinal)
        return ItemScore(ordinal=item.ordinal, parsed_answer=parsed, correct=parsed == expected)

    def _expected(self, ordinal: int) -> int:
        if not 0 <= ordinal < len(self._answers):
            raise KeyError(f"ordinal {ordinal} was not issued by {self.name}")
        return self._answers[ordinal]


def parse_json_answer(raw_final_text: str) -> int | None:
    """Read the reply the way this suite's own scorer would: last JSON object with an answer.

    Private to the synthetic suite: an adapter for a real benchmark must
    reimplement its upstream scorer's parsing, not borrow this one.
    """
    decoder = json.JSONDecoder()
    parsed: int | None = None
    for index, character in enumerate(raw_final_text):
        if character != "{":
            continue
        try:
            value = decoder.raw_decode(raw_final_text, index)[0]
        except ValueError:
            continue
        if isinstance(value, dict) and "answer" in value:
            option = _as_option(value["answer"])
            if option is not None:
                parsed = option
    return parsed


def _as_option(value: object) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        try:
            return int(value.strip())
        except ValueError:
            return None
    return None


def _generate_group(rng: random.Random) -> list[tuple[str, int]]:
    """One stimulus asked once per answer slot, so the right option is never in a fixed place."""
    target = rng.choice(string.ascii_uppercase)
    cells = rng.sample(
        [(row, column) for row in range(1, GRID_SIDE + 1) for column in range(1, GRID_SIDE + 1)],
        DEFAULT_GROUP_SIZE,
    )
    grid = _render_grid(target, cells[0], rng)
    options = [f"row {row}, column {column}" for row, column in cells]
    rows: list[tuple[str, int]] = []
    for slot in rng.sample(range(DEFAULT_GROUP_SIZE), DEFAULT_GROUP_SIZE):
        ordered = options[1:]
        ordered.insert(slot, options[0])
        rows.append((_render_question(grid, target, ordered), slot + 1))
    return rows


def _render_grid(target: str, cell: tuple[int, int], rng: random.Random) -> str:
    filler = [letter for letter in string.ascii_uppercase if letter != target]
    lines = []
    for row in range(1, GRID_SIDE + 1):
        letters = [
            target if (row, column) == cell else rng.choice(filler)
            for column in range(1, GRID_SIDE + 1)
        ]
        lines.append(" ".join(letters))
    return "\n".join(lines)


def _render_question(grid: str, target: str, options: list[str]) -> str:
    choices = "\n".join(f"{slot}. {option}" for slot, option in enumerate(options, start=1))
    return (
        f"Grid:\n{grid}\n\n"
        f'Which cell holds the letter "{target}"? Rows count from the top, columns '
        f"from the left.\n{choices}\n\n"
        'Reply with a JSON object of the form {"answer": <option number>}.'
    )
