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

"""Two sets of runs compared case by case: success flips per probe and level, tested for chance."""

from __future__ import annotations

from collections import Counter
from collections.abc import Iterable
from dataclasses import dataclass
import json
from pathlib import Path
from statistics import median
import time
from typing import Literal, TypeAlias

from pydantic import TypeAdapter
from scipy.stats import binomtest

from dimos.navigation.bench.report import Run
from dimos.navigation.bench.runner import RECORDING_FILE, RESULTS_FILE, RUN_FILE, SUITE_FILE
from dimos.navigation.bench.suite import Case, Manifest

SIGNIFICANCE = 0.05
MEASURES = ("first_contact_speed", "final_error_xy", "arrived_s", "spl")

Verdict = Literal["better", "worse", "no clear change"]
Reason = Literal["only on this side", "invalid", "scene differs"]
Json: TypeAlias = "dict[str, Json] | list[Json] | str | int | float | bool | None"

_RUN = TypeAdapter(Run)


@dataclass(frozen=True)
class Episode:
    """One case's scored episode on one side. No outcome means the episode is invalid."""

    case: Case
    outcome: str | None
    measures: dict[str, float]
    path: Path

    @property
    def success(self) -> bool:
        return self.outcome == "success"

    @property
    def recorded(self) -> bool:
        return (self.path / RECORDING_FILE).exists()


@dataclass(frozen=True)
class SuiteRun:
    """One suite's run directory."""

    path: Path
    suite: str
    run: Run


@dataclass(frozen=True)
class Side:
    """The episodes of one or more runs, keyed by case id. A case run again keeps its latest episode."""

    label: str
    runs: list[SuiteRun]
    episodes: dict[str, Episode]
    replaced: int


@dataclass(frozen=True)
class Pair:
    """One case scored on both sides."""

    base: Episode
    new: Episode

    @property
    def case(self) -> Case:
        return self.new.case

    @property
    def flip(self) -> int:
        """1 when the case became a success, -1 when it stopped being one, else 0."""
        return int(self.new.success) - int(self.base.success)

    @property
    def changed(self) -> bool:
        return self.base.outcome != self.new.outcome


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


@dataclass(frozen=True)
class Measure:
    """Medians of one continuous measure over the pairs with a value on both sides."""

    name: str
    n: int
    base: float
    new: float

    @property
    def change(self) -> float:
        return self.new - self.base


@dataclass(frozen=True)
class Flips:
    """How many paired cases changed success between two runs."""

    cases: int
    flips: int

    @property
    def rate(self) -> float:
        return self.flips / self.cases if self.cases else 0.0


@dataclass(frozen=True)
class ProbeRow:
    """One probe's comparison. Noise is the base against its repeat, when one was given."""

    probe: str
    tally: Tally
    levels: list[LevelTally]
    measures: list[Measure]
    noise: Flips | None


@dataclass(frozen=True)
class Comparison:
    base: Side
    new: Side
    pairs: list[Pair]
    left_out: dict[str, Counter[Reason]]
    scene_differs: list[str]
    probes: list[ProbeRow]
    repeat: Side | None

    @property
    def changed(self) -> list[Pair]:
        return [p for p in self.pairs if p.changed]


def suite_runs(path: Path) -> list[Path]:
    """The suite run at path, or every suite run under it."""
    if (path / RESULTS_FILE).exists():
        return [path]
    runs = sorted(d for d in path.iterdir() if (d / RESULTS_FILE).exists())
    if not runs:
        raise ValueError(f"no suite run in {path}")
    return runs


def load_run(path: Path) -> tuple[SuiteRun, list[Episode]]:
    """A suite run's metadata and its episodes in the order they finished."""
    meta = json.loads((path / RUN_FILE).read_text())
    if meta["time"] is None:
        meta["time"] = (path / RESULTS_FILE).stat().st_mtime
    manifest = Manifest.load(path / SUITE_FILE)
    cases = {c.id: c for c in manifest.cases}
    episodes = []
    for line in (path / RESULTS_FILE).read_text().splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        metrics = record.get("metrics") or {}
        measures = {k: float(metrics[k]) for k in MEASURES if metrics.get(k) is not None}
        case_id = record["case_id"]
        episodes.append(
            Episode(cases[case_id], record["outcome"], measures, path.resolve() / case_id)
        )
    return SuiteRun(path.resolve(), manifest.suite, _RUN.validate_python(meta)), episodes


def load_side(label: str, paths: Iterable[Path]) -> Side:
    """Every suite run under the paths, oldest first, so a case run again keeps its latest episode."""
    runs = sorted(
        (load_run(run) for path in paths for run in suite_runs(path)),
        key=lambda loaded: loaded[0].run.time,
    )
    episodes: dict[str, Episode] = {}
    replaced = 0
    for _, run_episodes in runs:
        for episode in run_episodes:
            replaced += episode.case.id in episodes
            episodes[episode.case.id] = episode
    return Side(label, [run for run, _ in runs], episodes, replaced)


def pair(base: Side, new: Side) -> tuple[list[Pair], dict[str, Counter[Reason]], list[str]]:
    """The cases valid on both sides with the same scene, why the rest were left out, and the scene mismatches."""
    pairs, scene_differs = [], []
    left_out: dict[str, Counter[Reason]] = {"base": Counter(), "new": Counter()}
    for name, side, other in (("base", base, new), ("new", new, base)):
        left_out[name]["only on this side"] = len(side.episodes.keys() - other.episodes.keys())
    for case_id in sorted(base.episodes.keys() & new.episodes.keys()):
        b, n = base.episodes[case_id], new.episodes[case_id]
        reason: Reason | None = None
        if b.outcome is None or n.outcome is None:
            reason = "invalid"
        elif b.case.scene_digest != n.case.scene_digest:
            reason = "scene differs"
            scene_differs.append(case_id)
        if reason is None:
            pairs.append(Pair(b, n))
        else:
            left_out["base"][reason] += 1
            left_out["new"][reason] += 1
    return pairs, {name: +counts for name, counts in left_out.items()}, scene_differs


def tally(pairs: list[Pair]) -> Tally:
    return Tally(
        cases=len(pairs),
        base_successes=sum(p.base.success for p in pairs),
        new_successes=sum(p.new.success for p in pairs),
        became_success=sum(p.flip > 0 for p in pairs),
        became_failure=sum(p.flip < 0 for p in pairs),
        base_failures=Counter(str(p.base.outcome) for p in pairs if not p.base.success),
        new_failures=Counter(str(p.new.outcome) for p in pairs if not p.new.success),
    )


def by_level(pairs: list[Pair]) -> list[LevelTally]:
    """A tally per level, the control first, then by value. Empty for cases without levels."""
    groups: dict[float | None, list[Pair]] = {}
    for p in pairs:
        level = p.case.level
        if level is not None:
            groups.setdefault(None if level.control else level.value, []).append(p)
    order = sorted(groups, key=lambda value: (value is not None, value or 0.0))
    return [LevelTally(value, tally(groups[value])) for value in order]


def measures(pairs: list[Pair]) -> list[Measure]:
    """Medians of each continuous measure over the pairs that have it on both sides."""
    rows = []
    for name in MEASURES:
        both = [p for p in pairs if name in p.base.measures and name in p.new.measures]
        if both:
            base = median(p.base.measures[name] for p in both)
            new = median(p.new.measures[name] for p in both)
            rows.append(Measure(name, len(both), base, new))
    return rows


def flips(pairs: list[Pair]) -> Flips:
    return Flips(len(pairs), sum(p.flip != 0 for p in pairs))


def compare(base: Side, new: Side, repeat: Side | None = None) -> Comparison:
    """Pair the sides by case and tally every probe, with the repeat's flips as the noise reference."""
    pairs, left_out, scene_differs = pair(base, new)
    noise: dict[str, list[Pair]] = {}
    if repeat is not None:
        for p in pair(base, repeat)[0]:
            noise.setdefault(p.case.tag, []).append(p)
    groups: dict[str, list[Pair]] = {}
    for p in pairs:
        groups.setdefault(p.case.tag, []).append(p)
    probes = [
        ProbeRow(
            probe=probe,
            tally=tally(group),
            levels=by_level(group),
            measures=measures(group),
            noise=flips(noise[probe]) if probe in noise else None,
        )
        for probe, group in sorted(groups.items())
    ]
    return Comparison(base, new, pairs, left_out, scene_differs, probes, repeat)


def warnings(comparison: Comparison) -> list[str]:
    """What makes the two sides less comparable than they look."""
    found = []
    days = {name: _days(side) for name, side in _sides(comparison)}
    if days["base"] != days["new"]:
        found.append(
            f"The sides ran on different days, base {', '.join(days['base'])}"
            f" and new {', '.join(days['new'])}."
        )
    policies = {name: {r.run.policy for r in side.runs} for name, side in _sides(comparison)}
    for name in ("base", "new"):
        if None in policies[name]:
            found.append(f"A {name} run has no policy, so it ran the default body policy.")
    if policies["base"] != policies["new"]:
        found.append("The sides ran different body policies.")
    if comparison.scene_differs:
        found.append(
            f"{len(comparison.scene_differs)} cases have a different scene on each side"
            " and are left out."
        )
    return found


def _sides(comparison: Comparison) -> list[tuple[str, Side]]:
    return [("base", comparison.base), ("new", comparison.new)]


def _days(side: Side) -> list[str]:
    stamps = (r.run.started if r.run.started is not None else r.run.time for r in side.runs)
    return sorted({time.strftime("%Y-%m-%d", time.localtime(t)) for t in stamps})


def _tally_json(t: Tally) -> dict[str, Json]:
    return {
        "cases": t.cases,
        "base_successes": t.base_successes,
        "new_successes": t.new_successes,
        "change": t.change,
        "became_success": t.became_success,
        "became_failure": t.became_failure,
        "unchanged": t.unchanged,
        "p": t.p,
        "verdict": t.verdict,
        "base_failures": dict(t.base_failures),
        "new_failures": dict(t.new_failures),
    }


def _side_json(side: Side) -> dict[str, Json]:
    return {
        "label": side.label,
        "episodes": len(side.episodes),
        "replaced": side.replaced,
        "runs": [
            {
                "path": str(r.path),
                "suite": r.suite,
                "commit": r.run.commit,
                "dirty": r.run.dirty,
                "policy": r.run.policy,
                "overrides": list(r.run.overrides),
                "started": r.run.started,
                "finished": r.run.finished,
            }
            for r in side.runs
        ],
    }


def _episode_json(episode: Episode) -> dict[str, Json]:
    return {
        "outcome": episode.outcome,
        "episode": str(episode.path),
        "recorded": episode.recorded,
    }


def to_json(comparison: Comparison) -> dict[str, Json]:
    """The comparison's numbers, for agents."""
    verdicts = Counter(row.tally.verdict for row in comparison.probes)
    total = tally(comparison.pairs)
    return {
        "base": _side_json(comparison.base),
        "new": _side_json(comparison.new),
        "repeat": None if comparison.repeat is None else _side_json(comparison.repeat),
        "summary": {
            "cases": total.cases,
            "base_successes": total.base_successes,
            "new_successes": total.new_successes,
            "probes_better": verdicts["better"],
            "probes_worse": verdicts["worse"],
            "probes_no_clear_change": verdicts["no clear change"],
            "outcomes_changed": len(comparison.changed),
        },
        "left_out": {
            name: {reason: n for reason, n in counts.items()}
            for name, counts in comparison.left_out.items()
        },
        "scene_differs": list(comparison.scene_differs),
        "warnings": list(warnings(comparison)),
        "probes": [
            {
                "probe": row.probe,
                **_tally_json(row.tally),
                "levels": [
                    {
                        "control": level.value is None,
                        "value": level.value,
                        **_tally_json(level.tally),
                    }
                    for level in row.levels
                ],
                "measures": [
                    {"name": m.name, "n": m.n, "base": m.base, "new": m.new, "change": m.change}
                    for m in row.measures
                ],
                "noise": None
                if row.noise is None
                else {"cases": row.noise.cases, "flips": row.noise.flips, "rate": row.noise.rate},
            }
            for row in comparison.probes
        ],
        "changed": [
            {
                "case": p.case.id,
                "probe": p.case.tag,
                "control": p.case.level is not None and p.case.level.control,
                "level": None if p.case.level is None else p.case.level.value,
                "seed": p.case.seed,
                "base": _episode_json(p.base),
                "new": _episode_json(p.new),
            }
            for p in comparison.changed
        ],
    }
