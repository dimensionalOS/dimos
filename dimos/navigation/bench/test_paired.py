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

from __future__ import annotations

from collections import Counter
from dataclasses import replace

import pytest

from dimos.navigation.bench.ground_truth import Difficulty
from dimos.navigation.bench.paired import Attempt, Tally, level_tallies, pair, tally
from dimos.navigation.bench.suite import Case, Level


def _case(seed: int, level: Level | None = None) -> Case:
    return Case(
        family="office",
        seed=seed,
        params={},
        start=(1.0, 1.0, 0.0),
        goal=(4.0, 1.0, 0.0),
        tag="door_width",
        id=f"door_width-s{seed}-x",
        split="dev",
        difficulty=Difficulty(0.5, 1, 1.0, 0),
        route_length=3.0,
        scene_digest="d",
        level=level,
    )


def _side(*attempts: tuple[Case, str | None]) -> dict[str, Attempt]:
    return {case.id: Attempt(case, outcome) for case, outcome in attempts}


def test_pairs_valid_cases_on_both_sides_and_counts_the_rest() -> None:
    a, b, c, d, e, f = (_case(seed) for seed in range(1, 7))
    moved = replace(c, scene_digest="other")
    base = _side((a, "success"), (b, "stuck"), (c, "success"), (d, None))
    new = _side((b, "success"), (moved, "success"), (d, "success"), (e, "success"), (f, "fall"))
    result = pair(base, new)
    assert [p.case.id for p in result.pairs] == [b.id]
    assert result.left_out["base"] == Counter(
        {"only on this side": 1, "invalid": 1, "scene differs": 1}
    )
    assert result.left_out["new"] == Counter(
        {"only on this side": 2, "invalid": 1, "scene differs": 1}
    )
    assert result.scene_differs == [c.id]


def test_tally_counts_flips_and_failure_types_per_level() -> None:
    control, wide = Level("door_width", 0.0, control=True), Level("door_width", 0.6)
    cases = [_case(seed, control if seed < 3 else wide) for seed in range(1, 7)]
    base_outcomes = ["stuck", "fall", "success", "success", "stalled", "success"]
    new_outcomes = ["success", "success", "stuck", "success", "stalled", "success"]
    result = pair(
        _side(*zip(cases, base_outcomes, strict=True)),
        _side(*zip(cases, new_outcomes, strict=True)),
    )
    t = tally(result.pairs)
    assert (t.cases, t.base_successes, t.new_successes, t.change) == (6, 3, 4, 1)
    assert (t.became_success, t.became_failure, t.unchanged) == (2, 1, 3)
    assert t.base_failures == Counter({"stuck": 1, "fall": 1, "stalled": 1})
    assert t.new_failures == Counter({"stuck": 1, "stalled": 1})
    assert [p.case.id for p in result.pairs if p.changed] == [c.id for c in cases[:3]]
    levels = level_tallies(result.pairs)
    assert [(lv.value, lv.tally.cases, lv.tally.became_success) for lv in levels] == [
        (None, 2, 2),
        (0.6, 4, 0),
    ]


@pytest.mark.parametrize(
    ("up", "down", "p", "verdict"),
    [
        (6, 0, 0.03125, "better"),
        (5, 1, 0.21875, "no clear change"),
        (0, 10, 0.001953125, "worse"),
        (0, 0, 1.0, "no clear change"),
    ],
)
def test_mcnemar_is_exact_on_the_discordant_pairs(
    up: int, down: int, p: float, verdict: str
) -> None:
    t = Tally(20, 10, 10 + up - down, up, down, Counter(), Counter())
    assert t.p == pytest.approx(p)
    assert t.verdict == verdict
