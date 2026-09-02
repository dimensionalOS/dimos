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

"""Scoring helpers: plain functions over typed values, graded credit in one line.

Scores are floats in ``[0, 1]``. Msg types support arithmetic, so physical
scorers stay one-liners::

    lambda s: ramp((GOAL - s.streams.odom.last().data.position).length(), band=0.5)

LLM-based scoring wraps ``openevals`` — a function library (nothing to
subclass): factories return evaluators called with
``inputs/outputs/reference_outputs`` returning ``{"key", "score", "comment"}``.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import TYPE_CHECKING, TypeVar

if TYPE_CHECKING:
    from dimos.evals.types import Outcome

T = TypeVar("T")


def exact(expected: T, got: T) -> float:
    return float(expected == got)


# -- parsers (model text -> typed answer) -----------------------------------------


def first_number(text: str) -> float:
    """Pull the first number out of a model reply ("about 12.5 meters" -> 12.5)."""
    import re

    match = re.search(r"-?\d+(?:\.\d+)?", text)
    if match is None:
        raise ValueError(f"no number in reply: {text[:80]!r}")
    return float(match.group())


def yes_no(text: str) -> str:
    """Normalize a reply to "yes"/"no"."""
    t = text.strip().lower()
    if t.startswith(("yes", "no")):
        return "yes" if t.startswith("yes") else "no"
    raise ValueError(f"not a yes/no reply: {text[:80]!r}")


def choice(options: Sequence[str]) -> Callable[[str], str]:
    """Parser for a multiple-choice reply: the last option the model names, so
    that reasoning before the answer does not decide it. Longest option first,
    so "northeast" wins over "north"."""
    import re

    pattern = re.compile(r"\b(" + "|".join(sorted(options, key=len, reverse=True)) + r")\b", re.I)

    def parse(text: str) -> str:
        # "north-west" must read as northwest, not as west.
        found = pattern.findall(re.sub(r"(?<=[A-Za-z])-(?=[A-Za-z])", "", text))
        if not found:
            raise ValueError(f"no option from {list(options)} in reply: {text[:80]!r}")
        return str(found[-1]).lower()

    return parse


def grade_choice(choices: Sequence[str], answer: str) -> Callable[[Outcome], float]:
    """Exact match on the last choice the reply names; naming none scores 0."""
    parse = choice(choices)

    def grade(o: Outcome) -> float:
        try:
            got = parse(o.trajectory.final_answer)
        except ValueError:
            return 0.0
        return exact(answer.lower(), got)

    return grade


def within(band: float) -> Callable[[float, float], float]:
    """1.0 at exact, linear to 0.0 at ``band`` away."""
    return lambda expected, got: max(0.0, 1.0 - abs(got - expected) / band)


def ramp(distance: float, band: float) -> float:
    """Distance (meters) -> [0, 1] credit inside ``band``."""
    return max(0.0, 1.0 - distance / band)


def judge(rubric: str, *, model: str = "openai:gpt-5.6-luna") -> Callable[[str, str], float]:
    """LLM-as-judge with partial credit via openevals ``continuous=True``.

    ``rubric`` may reference ``{inputs}``, ``{outputs}``, ``{reference_outputs}``.
    """
    from openevals.llm import create_llm_as_judge

    evaluator = create_llm_as_judge(prompt=rubric, model=model, continuous=True)

    def _score(expected: str, got: str) -> float:
        result = evaluator(inputs="", outputs=got, reference_outputs=expected)
        if isinstance(result, list):
            result = result[0]
        return float(result["score"])

    return _score


# -- reducers over a series (e.g. a score per recorded pose) --------------------------


def final(scores: Sequence[float]) -> float:
    return scores[-1]


def floor(scores: Sequence[float]) -> float:
    """Worst moment wins — "never left the zone"."""
    return min(scores)


def mean(scores: Sequence[float]) -> float:
    return sum(scores) / len(scores)
