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

"""Two sides' outcomes paired case by case, with an exact McNemar test of the success flips."""

from __future__ import annotations

from collections import Counter
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Literal

from scipy.stats import binomtest

from dimos.navigation.bench.suite import Case

SIGNIFICANCE = 0.05

Verdict = Literal["better", "worse", "no clear change"]
Reason = Literal["only on this side", "invalid", "scene differs"]


@dataclass(frozen=True)
class Attempt:
    """A case's latest episode on one side. No outcome means the episode is invalid."""

    case: Case
    outcome: str | None


@dataclass(frozen=True)
class Pair:
    """One case scored on both sides."""

    case: Case
    base: str
    new: str

    @property
    def flip(self) -> int:
        """1 when the case became a success, -1 when it stopped being one, else 0."""
        return int(self.new == "success") - int(self.base == "success")

    @property
    def changed(self) -> bool:
        return self.base != self.new


@dataclass(frozen=True)
class Pairing:
    """The cases valid on both sides with the same scene, and why the rest were left out."""

    pairs: list[Pair]
    left_out: dict[str, Counter[Reason]]
    scene_differs: list[str]


@dataclass(frozen=True)
class Tally:
    """Paired success counts over a group of cases."""

    cases: int
    base_successes: int
    new_successes: int
    became_success: int
    became_failure: int
    base_failures: Counter[str]
    new_failures: Counter[str]

    @property
    def change(self) -> int:
        return self.new_successes - self.base_successes

    @property
    def unchanged(self) -> int:
        return self.cases - self.became_success - self.became_failure

    @property
    def p(self) -> float:
        """Exact McNemar p value: the two-sided binomial test on the discordant pairs."""
        flips = self.became_success + self.became_failure
        return float(binomtest(self.became_success, flips).pvalue) if flips else 1.0

    @property
    def verdict(self) -> Verdict:
        if self.p >= SIGNIFICANCE:
            return "no clear change"
        return "better" if self.change > 0 else "worse"


@dataclass(frozen=True)
class LevelTally:
    """One level of a probe's swept variable. A control level has no value."""

    value: float | None
    tally: Tally


def pair(base: Mapping[str, Attempt], new: Mapping[str, Attempt]) -> Pairing:
    """Pair the cases valid on both sides with the same scene, counting the rest by reason."""
    pairs, scene_differs = [], []
    left_out: dict[str, Counter[Reason]] = {"base": Counter(), "new": Counter()}
    for name, side, other in (("base", base, new), ("new", new, base)):
        left_out[name]["only on this side"] = len(side.keys() - other.keys())
    for case_id in sorted(base.keys() & new.keys()):
        b, n = base[case_id], new[case_id]
        reason: Reason
        if b.outcome is None or n.outcome is None:
            reason = "invalid"
        elif b.case.scene_digest != n.case.scene_digest:
            reason = "scene differs"
            scene_differs.append(case_id)
        else:
            pairs.append(Pair(n.case, b.outcome, n.outcome))
            continue
        left_out["base"][reason] += 1
        left_out["new"][reason] += 1
    return Pairing(pairs, {name: +counts for name, counts in left_out.items()}, scene_differs)


def tally(pairs: list[Pair]) -> Tally:
    return Tally(
        cases=len(pairs),
        base_successes=sum(p.base == "success" for p in pairs),
        new_successes=sum(p.new == "success" for p in pairs),
        became_success=sum(p.flip > 0 for p in pairs),
        became_failure=sum(p.flip < 0 for p in pairs),
        base_failures=Counter(p.base for p in pairs if p.base != "success"),
        new_failures=Counter(p.new for p in pairs if p.new != "success"),
    )


def level_tallies(pairs: list[Pair]) -> list[LevelTally]:
    """A tally per level, the control first, then by value. Empty for cases without levels."""
    groups: dict[float | None, list[Pair]] = {}
    for p in pairs:
        level = p.case.level
        if level is not None:
            groups.setdefault(None if level.control else level.value, []).append(p)
    order = sorted(groups, key=lambda value: (value is not None, value or 0.0))
    return [LevelTally(value, tally(groups[value])) for value in order]
