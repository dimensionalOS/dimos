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

"""Open-phrase answers scored on the OpenEQA 1-5 scale.

The mark is an integer 1 through 5. Credit reported to the runner is
``(mark - 1) / 4``, so 4 is the pass line at 0.75 and 5 is 1.0.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

from dimos.evals.types import Outcome

JUDGE_MODEL = "openai:gpt-5.6-luna"
OPEN_ANSWER_PASS = 0.75
_MARKS = (1.0, 2.0, 3.0, 4.0, 5.0)
_CHECKS = Path(__file__).with_name("open_answer_checks.json")

OPEN_EQA_RUBRIC = """\
Score the predicted answer to this question against the reference.

Question: {inputs}
Reference answer: {reference_outputs}
Predicted answer: {outputs}

Use one integer:
5 correct, including a paraphrase
4 mostly correct, with a small error or an extra detail the reference does not state
3 partly correct
2 mostly wrong, with a little relevant detail
1 completely wrong, or about something else
"""


def mark_of(value: float) -> int:
    """The OpenEQA integer a judge returned, or ``ValueError`` when it is not 1-5."""
    for item in _MARKS:
        if math.isclose(value, item):
            return int(item)
    raise ValueError(f"OpenEQA mark must be 1, 2, 3, 4, or 5, got {value}")


def credit(mark: float) -> float:
    """Map an OpenEQA mark to the runner's 0-1 score."""
    return (mark_of(mark) - 1) / 4


def agreement(checks: Sequence[OpenCheck], marks: Sequence[float]) -> float:
    """Fraction of checks whose judge mark equals the human mark."""
    if not checks or len(checks) != len(marks):
        raise ValueError("need one mark per check")
    matched = sum(mark_of(mark) == check.human for check, mark in zip(checks, marks, strict=True))
    return matched / len(checks)


def _evaluator(model: str) -> Callable[..., Any]:
    from openevals.llm import create_llm_as_judge

    return create_llm_as_judge(
        prompt=OPEN_EQA_RUBRIC,
        model=model,
        choices=list(_MARKS),
        use_reasoning=False,
    )


def _mark(question: str, reference: str, answer: str, *, model: str) -> float:
    result = _evaluator(model)(inputs=question, outputs=answer, reference_outputs=reference)
    if isinstance(result, list):
        result = result[0]
    return float(result["score"])


def grade_open_answer(
    question: str, reference: str, *, model: str = JUDGE_MODEL
) -> Callable[[Outcome], float]:
    """Grade the final reply. The judge sees the question, the reference, and the reply.

    The model client is built when the case is graded, not when the suite is imported.
    """

    def grade(outcome: Outcome) -> float:
        return credit(_mark(question, reference, outcome.trajectory.final_answer, model=model))

    return grade


def score_checks(checks: Sequence[OpenCheck] | None = None, *, model: str = JUDGE_MODEL) -> float:
    """Ask the judge to mark the calibration set and return its agreement with the human marks.

    This calls the model once per check. Unit tests do not call it.
    """
    rows = load_checks() if checks is None else checks
    marks = [_mark(row.question, row.reference, row.answer, model=model) for row in rows]
    return agreement(rows, marks)


@dataclass(frozen=True, kw_only=True)
class OpenCheck:
    """One human-marked reply used to check the judge before adding questions."""

    id: str
    question: str
    reference: str
    answer: str
    human: int

    def __post_init__(self) -> None:
        mark_of(self.human)


def load_checks(path: Path = _CHECKS) -> tuple[OpenCheck, ...]:
    """The calibration replies committed next to this module."""
    rows = json.loads(path.read_text())
    return tuple(OpenCheck(**row) for row in rows)
