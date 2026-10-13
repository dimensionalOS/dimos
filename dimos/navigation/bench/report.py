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

"""One self-contained HTML page per run, from its results and the suite it names."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import json
from pathlib import Path
import shlex
from statistics import median
import time
from typing import get_args

from jinja2 import Environment, FileSystemLoader
from pydantic import TypeAdapter

from dimos.navigation.bench.runner import RESULTS_FILE, RUN_FILE, SUITE_FILE
from dimos.navigation.bench.scorer import Outcome
from dimos.navigation.bench.suite import Case, Manifest

REPORT_FILE = "report.html"
TEMPLATE_FILE = "report_template.html"
OUTCOMES: tuple[str, ...] = get_args(Outcome)
STRIP_M = 1.0
STRIP_TICK_M = 0.25
STRIP_PX = 240
THRESHOLDS = (0.5, 0.9)
CURVE_PX = (480, 180)
CURVE_PAD_PX = 44


@dataclass(frozen=True)
class Row:
    """One episode as the report sees it. The outcome is the error when scoring produced none."""

    case: Case
    episode: Path
    outcome: str
    signature: str | None
    terminal: str | None
    spl: float | None
    arrived_s: float | None
    final_xy: tuple[float, float] | None
    final_error_xy: float | None
    collisions: int | None


@dataclass(frozen=True)
class Run:
    """What run.json records that the report shows."""

    split: str | None
    blueprint: str
    policy: str | None
    overrides: list[str]
    episodes: int
    finished: bool
    time: float
    started: float | None = None


_RUN = TypeAdapter(Run)


@dataclass(frozen=True)
class Report:
    suite: str
    run: Run
    rows: list[Row]

    @property
    def outcomes(self) -> list[str]:
        """Outcome columns, known ones in rank order, then errors."""
        seen = {r.outcome for r in self.rows}
        return [o for o in OUTCOMES if o in seen] + sorted(seen - set(OUTCOMES))

    @property
    def successes(self) -> list[Row]:
        return [r for r in self.rows if r.outcome == "success"]


@dataclass(frozen=True)
class DifficultyRow:
    """Medians of the difficulty measures over one outcome's episodes, and a dot per bottleneck."""

    outcome: str
    n: int
    clearance: float
    doors: float
    detour: float
    route: float
    dots: list[int]


@dataclass(frozen=True)
class LevelRow:
    """One level of a swept variable: its outcomes and failure signatures."""

    value: float
    outcomes: Counter[str]
    signatures: Counter[str]

    @property
    def n(self) -> int:
        return sum(self.outcomes.values())

    @property
    def successes(self) -> int:
        return self.outcomes["success"]

    @property
    def rate(self) -> float:
        return self.successes / self.n


@dataclass(frozen=True)
class CurvePoint:
    """A level on the success curve, in pixels."""

    x: int
    y: int
    label: str
    count: str


def load(run_dir: Path) -> Report:
    """The run's results joined with the suite's cases."""
    meta = json.loads((run_dir / RUN_FILE).read_text())
    if meta["time"] is None:
        meta["time"] = (run_dir / RESULTS_FILE).stat().st_mtime
    run = _RUN.validate_python(meta)
    manifest = Manifest.load(run_dir / SUITE_FILE)
    cases = {c.id: c for c in manifest.cases}
    rows = [
        _row(cases[record["case_id"]], record, run_dir.resolve() / record["case_id"])
        for record in map(json.loads, (run_dir / RESULTS_FILE).read_text().splitlines())
    ]
    return Report(manifest.suite, run, sorted(rows, key=lambda r: r.case.id))


def write(run_dir: Path) -> Path:
    path = run_dir / REPORT_FILE
    path.write_text(render(load(run_dir)))
    return path


def _row(case: Case, record: dict[str, object], episode: Path) -> Row:
    metrics = record.get("metrics") or {}
    assert isinstance(metrics, dict)
    final = metrics.get("final_xy")
    return Row(
        case=case,
        episode=episode,
        outcome=str(record["outcome"] or record["error"]),
        signature=_text(metrics.get("signature")),
        terminal=_text(record.get("terminal")),
        spl=_float(metrics.get("spl")),
        arrived_s=_float(metrics.get("arrived_s")),
        final_xy=(float(final[0]), float(final[1])) if final else None,
        final_error_xy=_float(metrics.get("final_error_xy")),
        collisions=None if metrics.get("collisions") is None else int(metrics["collisions"]),
    )


def _text(value: object) -> str | None:
    return None if value is None else str(value)


def _float(value: object) -> float | None:
    return None if value is None else float(value)  # type: ignore[arg-type]


def by_stressor(report: Report) -> dict[str, Counter[str]]:
    """Outcome counts per stressor tag."""
    table: dict[str, Counter[str]] = {}
    for row in report.rows:
        table.setdefault(row.case.tag, Counter())[row.outcome] += 1
    return dict(sorted(table.items()))


def by_signature(report: Report) -> dict[str, Counter[str]]:
    """Stressor counts per failure signature."""
    table: dict[str, Counter[str]] = {}
    for row in report.rows:
        if row.signature is not None:
            table.setdefault(row.signature, Counter())[row.case.tag] += 1
    return dict(sorted(table.items(), key=lambda item: -sum(item[1].values())))


def by_difficulty(report: Report) -> list[DifficultyRow]:
    """Difficulty medians per outcome, in column order."""
    rows = []
    for outcome in report.outcomes:
        cases = [r.case for r in report.rows if r.outcome == outcome]
        clearances = [c.difficulty.min_clearance for c in cases]
        rows.append(
            DifficultyRow(
                outcome=outcome,
                n=len(cases),
                clearance=median(clearances),
                doors=median(c.difficulty.doors for c in cases),
                detour=median(c.difficulty.detour for c in cases),
                route=median(c.route_length for c in cases),
                dots=[_strip_x(v) for v in clearances],
            )
        )
    return rows


def by_level(report: Report) -> list[LevelRow]:
    """Outcomes and failure signatures per level of the swept variable, lowest first."""
    groups: dict[float, list[Row]] = {}
    for row in report.rows:
        if row.case.level is not None:
            groups.setdefault(row.case.level.value, []).append(row)
    return [
        LevelRow(
            value=value,
            outcomes=Counter(r.outcome for r in rows),
            signatures=Counter(r.signature for r in rows if r.signature is not None),
        )
        for value, rows in sorted(groups.items())
    ]


def crossing(levels: list[LevelRow], rate: float) -> float | None:
    """The level from which the success rate stays at or above the rate, interpolated between levels.

    The lowest level when every level is there, and None when the highest is not.
    """
    short = [i for i, row in enumerate(levels) if row.rate < rate]
    if not short:
        return levels[0].value
    if short[-1] == len(levels) - 1:
        return None
    below, above = levels[short[-1]], levels[short[-1] + 1]
    return below.value + (above.value - below.value) * (rate - below.rate) / (
        above.rate - below.rate
    )


def _reached(levels: list[LevelRow], rate: float) -> str:
    at = crossing(levels, rate)
    if at is None:
        return f"never reaches {rate:.0%}"
    return f"{rate:.0%} at {at:.2f} m" + (" or lower" if at == levels[0].value else "")


def _curve_y(rate: float) -> int:
    return round(CURVE_PAD_PX / 2 + (1 - rate) * (CURVE_PX[1] - CURVE_PAD_PX))


def _curve(levels: list[LevelRow]) -> list[CurvePoint]:
    inner = CURVE_PX[0] - 2 * CURVE_PAD_PX
    span = (levels[-1].value - levels[0].value) or 1.0
    return [
        CurvePoint(
            x=round(CURVE_PAD_PX + (row.value - levels[0].value) / span * inner),
            y=_curve_y(row.rate),
            label=f"{row.value:.2f}",
            count=f"{row.successes}/{row.n}",
        )
        for row in levels
    ]


def _strip_x(meters: float) -> int:
    return round(min(meters / STRIP_M, 1.0) * STRIP_PX)


def render(report: Report) -> str:
    """The whole page from the template beside this module."""
    run = report.run
    spls = [r.spl for r in report.successes if r.spl is not None]
    spl = f", median SPL of successes {median(spls):.2f}" if spls else ""
    levels = by_level(report)
    edge, curve = "", []
    if levels:
        edge = ". Success rate " + ", ".join(_reached(levels, rate) for rate in THRESHOLDS)
        curve = _curve(levels)
    env = Environment(loader=FileSystemLoader(Path(__file__).parent), autoescape=True)
    env.filters["quote"] = shlex.quote
    return env.get_template(TEMPLATE_FILE).render(
        title=f"nav-bench {report.suite}",
        verdict=f"{len(report.successes)} of {len(report.rows)} episodes succeeded{spl}{edge}",
        wall=f"{(run.time - run.started) / 60:.1f} min" if run.started is not None else "unknown",
        finished_at=time.strftime("%Y-%m-%d %H:%M", time.localtime(run.time)),
        report=report,
        run=run,
        stressors=by_stressor(report),
        totals=Counter(r.outcome for r in report.rows),
        signatures=by_signature(report),
        difficulty=by_difficulty(report),
        variable=next((r.case.level.variable for r in report.rows if r.case.level), ""),
        levels=levels,
        curve=curve,
        curve_px=CURVE_PX,
        curve_grid=[(f"{rate:.0%}", _curve_y(rate)) for rate in (0.0, *THRESHOLDS, 1.0)],
        strip_m=STRIP_M,
        strip_px=STRIP_PX,
        strip_ticks=[_strip_x(k * STRIP_TICK_M) for k in range(int(STRIP_M / STRIP_TICK_M) + 1)],
    )
