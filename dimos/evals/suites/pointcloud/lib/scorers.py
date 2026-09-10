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

"""Scorers for the point-cloud suites' open-ended answers: a list of places,
or the word for "there are none"."""

from __future__ import annotations

from collections.abc import Callable, Sequence
import math

from dimos.evals.scorers import within


def coord_list(text: str) -> list[tuple[float, ...]]:
    """Parse an open-ended "list the places" reply into coordinate tuples.

    One tuple per line ("9.0,4.1,+0.25" -> ``(9.0, 4.1, 0.25)``), stray prose
    around the numbers tolerated the way :func:`first_number` tolerates it. A
    line needs at least two numbers to count as a coordinate, so "0 areas"
    reads as prose rather than as a point. The literal negative answer --
    ``none`` or ``level``, case-insensitive -- is the empty list.
    """
    import re

    number = re.compile(r"-?\d+(?:\.\d+)?")
    coords = [
        tuple(float(n) for n in found)
        for line in text.splitlines()
        if len(found := number.findall(line)) >= 2
    ]
    if coords:
        return coords
    if re.search(r"\b(none|level)\b", text, re.I):
        return []
    raise ValueError(f"neither coordinates nor none/level in reply: {text[:80]!r}")


def matched_set(
    radius: float, value_band: float | None = None
) -> Callable[[Sequence[tuple[float, ...]], Sequence[tuple[float, ...]]], float]:
    """F1 over point sets -- the scorer for "list every X, or say there are none".

    Predicted points are greedily paired with expected ones, nearest xy first,
    within ``radius`` meters; the score is ``2 * matched / (n_pred + n_true)``,
    so both misses and spurious extras cost. Two empty lists score 1.0 --
    saying "none" when there is none is the right answer. Naming anything at
    all against an empty expected scores 0.0.

    With ``value_band`` set, each pair is weighted by :func:`within` on the
    third element (a rise, a height offset), so a real feature found at the
    wrong size earns partial credit rather than full.
    """
    value_score = within(value_band) if value_band is not None else None

    def score(expected: Sequence[tuple[float, ...]], got: Sequence[tuple[float, ...]]) -> float:
        if not expected and not got:
            return 1.0
        if not expected or not got:
            return 0.0
        pairs: list[tuple[float, int, int]] = []
        for j, p in enumerate(got):
            for i, e in enumerate(expected):
                d = math.hypot(p[0] - e[0], p[1] - e[1])
                if d <= radius:
                    pairs.append((d, i, j))
        used_e: set[int] = set()
        used_p: set[int] = set()
        matched = 0.0
        for _, i, j in sorted(pairs):
            if i in used_e or j in used_p:
                continue
            used_e.add(i)
            used_p.add(j)
            if value_score is None:
                matched += 1.0
            elif len(expected[i]) > 2 and len(got[j]) > 2:
                matched += value_score(expected[i][2], got[j][2])
        return 2.0 * matched / (len(got) + len(expected))

    return score
