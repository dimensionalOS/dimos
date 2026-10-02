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
import math
from typing import TypeVar

T = TypeVar("T")


def exact(expected: T, got: T) -> float:
    return float(expected == got)


def rank_order(expected: Sequence[str], got: Sequence[str]) -> float:
    """Fraction of correctly ordered pairs in a complete, unique-label ranking."""
    if len(expected) < 2 or len(set(expected)) != len(expected):
        raise ValueError("Expected ranking must contain at least two unique labels")
    if len(got) != len(expected) or set(got) != set(expected):
        return 0.0
    positions = {label: i for i, label in enumerate(got)}
    correct = sum(
        positions[left] < positions[right]
        for i, left in enumerate(expected)
        for right in expected[i + 1 :]
    )
    return correct / (len(expected) * (len(expected) - 1) / 2)


def numeric(expected: float, got: float, *, tolerance: float, band: float) -> float:
    """Compare numbers: full credit within tolerance, linear to zero at band.

    Parse model text separately, e.g. with ``first_number``. Non-finite
    observations receive zero; invalid scoring parameters raise ValueError.
    """
    if not all(math.isfinite(v) for v in (expected, tolerance, band)) or not 0 <= tolerance < band:
        raise ValueError("Require finite reference and 0 <= tolerance < band")
    if not math.isfinite(got):
        return 0.0
    error = abs(got - expected)
    # Allow only a few floating-point ULPs, capped relative to the score band.
    rounding = min(4 * max(math.ulp(got), math.ulp(expected)), (band - tolerance) * 1e-12)
    if error <= tolerance or abs(error - tolerance) <= rounding:
        return 1.0
    if error >= band or abs(error - band) <= rounding:
        return 0.0
    return max(0.0, min(1.0, (band - error) / (band - tolerance)))


# -- parsers (model text -> typed answer) -----------------------------------------


def ranking(text: str) -> tuple[str, ...]:
    """Parse single-letter labels, contiguous or separated by commas/whitespace.

    Vocabulary, completeness, and uniqueness are checked by ``rank_order``.
    """
    return tuple(c for c in text.strip().upper() if c != "," and not c.isspace())


def first_number(text: str) -> float:
    """The number a reply answers with: the only number on its last line ("...\\n\\n4",
    "≈ 3.1 m²", "Answer: 4"), else the first number anywhere ("about 12.5 meters")."""
    import re

    plain = re.sub(r"(?<=\d),(?=\d{3}\b)", "", text)  # 20,834 -> 20834
    on_last_line = re.findall(_NUMBER, _last_line(plain))
    if len(on_last_line) == 1:
        return float(on_last_line[0])
    match = re.search(_NUMBER, plain)
    if match is None:
        raise ValueError(f"no number in reply: {text[:80]!r}")
    return float(match.group())


def yes_no(text: str) -> str:
    """Normalize a reply to "yes"/"no": the only yes/no on its last line, else its opening word."""
    import re

    on_last_line = re.findall(r"\b(yes|no)\b", _last_line(text).lower())
    if len(on_last_line) == 1:
        return str(on_last_line[0])
    t = text.strip().lower().lstrip("*_`#\"' ")
    if t.startswith(("yes", "no")):
        return "yes" if t.startswith("yes") else "no"
    raise ValueError(f"not a yes/no reply: {text[:80]!r}")


def choice(options: Sequence[str], *, case_sensitive: bool = False) -> Callable[[str], str]:
    """Parser for a multiple-choice reply: the last option the model names, so
    that reasoning before the answer does not decide it. Longest option first,
    so "northeast" wins over "north". ``case_sensitive`` for lettered options
    ("A", "B"), where the article "a" must not count."""
    import re

    words = "|".join(re.escape(o) for o in sorted(options, key=len, reverse=True))
    pattern = re.compile(rf"\b({words})\b", 0 if case_sensitive else re.I)

    def parse(text: str) -> str:
        # "north-west" must read as northwest, not as west.
        found = pattern.findall(re.sub(r"(?<=[A-Za-z])-(?=[A-Za-z])", "", text))
        if not found:
            raise ValueError(f"no option from {list(options)} in reply: {text[:80]!r}")
        return str(found[-1]) if case_sensitive else str(found[-1]).lower()

    return parse


_NUMBER = r"-?\d+(?:\.\d+)?"


def _last_line(text: str) -> str:
    lines = [line.strip("*_` .!\t") for line in text.splitlines() if line.strip()]
    return lines[-1] if lines else ""


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
