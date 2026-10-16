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

"""One self-contained HTML page per run, from its results and the suite it names, optionally against a baseline."""

from __future__ import annotations

import base64
from collections import Counter
from collections.abc import Iterable
from dataclasses import dataclass, field, replace
import json
import math
from pathlib import Path
import shlex
from statistics import median
import time
from typing import Literal, TypeAlias, get_args

from jinja2 import Environment, FileSystemLoader
from pydantic import TypeAdapter

from dimos.navigation.bench.ground_truth import GO2
from dimos.navigation.bench.paired import (
    Attempt,
    LevelTally,
    Pair,
    Reason,
    Tally,
    level_tallies,
    pair,
    tally,
)
from dimos.navigation.bench.runner import RECORDING_FILE, RESULTS_FILE, RUN_FILE, SUITE_FILE
from dimos.navigation.bench.scorer import Outcome, contact_name, definitions
from dimos.navigation.bench.snapshot import bounds, sample, snapshot_path
from dimos.navigation.bench.suite import PROBES, Case, Manifest, Pose, Rules
from dimos.simulation.scenes.procedural import Rect

REPORT_FILE = "report.html"
TEMPLATE_FILE = "report_template.html"
OUTCOMES: tuple[str, ...] = get_args(Outcome)
ARRIVED = ("success", "reached_with_collision")
STRIP_M = 1.0
STRIP_TICK_M = 0.25
STRIP_PX = 240
CURVE_PX = (480, 180)
CURVE_PAD_PX = 56
CURVE_LABEL_PX = 34
CURVE_ROW_PX = 28
THUMBNAIL_PX = 240
SNAPSHOT_PX = 480
BAR_PX = 100
FAILURE_NAMES = {"reached_with_collision": "collision"}
MEASURES = ("first_contact_speed", "final_error_xy", "arrived_s", "spl")

Json: TypeAlias = "dict[str, Json] | list[Json] | str | int | float | bool | None"


@dataclass(frozen=True)
class Row:
    """One scored episode as the report sees it."""

    case: Case
    episode: Path
    outcome: str
    signature: str | None
    signatures: tuple[str, ...]
    terminal: str | None
    spl: float | None
    arrived_s: float | None
    final_xy: tuple[float, float] | None
    final_error_xy: float | None
    collisions: int | None
    feature_m: float | None
    first_contact_speed: float | None
    near_fall: bool
    mismatch: bool
    contacts: tuple[str, ...]

    @property
    def unexplained(self) -> bool:
        """A failure that no signature check applies to."""
        return self.outcome not in ARRIVED and not self.signatures


@dataclass(frozen=True)
class Invalid:
    """An episode that produced no outcome. It is listed and left out of every count."""

    case: Case
    episode: Path
    error: str
    terminal: str | None


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
    commit: str | None = None
    dirty: bool | None = None
    procs: int | None = None
    machine: str | None = None


_RUN = TypeAdapter(Run)


@dataclass(frozen=True)
class Report:
    run_dir: Path
    suite: str
    rules: Rules
    run: Run
    rows: list[Row]
    invalid: list[Invalid]

    @property
    def outcomes(self) -> list[str]:
        """Outcome columns in rank order."""
        seen = {r.outcome for r in self.rows}
        return [o for o in OUTCOMES if o in seen]

    @property
    def successes(self) -> list[Row]:
        return [r for r in self.rows if r.outcome == "success"]

    @property
    def unexplained(self) -> list[Row]:
        return [r for r in self.rows if r.unexplained]

    @property
    def mismatches(self) -> list[Row]:
        """The episodes where the driver's verdict and the recording disagree."""
        return [r for r in self.rows if r.mismatch]


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
    """One level of a swept variable: the outcomes and failure signatures of the cases that encountered the feature.

    The cases that did not are left out of those and counted on their own.
    """

    value: float
    outcomes: Counter[str]
    signatures: Counter[str]
    left_out: Counter[str]
    left_out_collisions: int
    near_falls: int = 0
    contacts: Counter[str] = field(default_factory=Counter)
    unexplained: int = 0

    @property
    def not_encountered(self) -> int:
        return sum(self.left_out.values())

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
    drop: int = 0


def load(run_dir: Path) -> Report:
    """The run's results joined with the suite's cases."""
    meta = json.loads((run_dir / RUN_FILE).read_text())
    if meta["time"] is None:
        meta["time"] = (run_dir / RESULTS_FILE).stat().st_mtime
    run = _RUN.validate_python(meta)
    manifest = Manifest.load(run_dir / SUITE_FILE)
    cases = {c.id: c for c in manifest.cases}
    rows, invalid = [], []
    for record in map(json.loads, (run_dir / RESULTS_FILE).read_text().splitlines()):
        case, episode = cases[record["case_id"]], run_dir.resolve() / record["case_id"]
        if record["outcome"] is None:
            terminal = _text(record.get("terminal"))
            invalid.append(Invalid(case, episode, str(record["error"]), terminal))
        else:
            rows.append(_row(case, record, episode))
    return Report(
        run_dir,
        manifest.suite,
        manifest.rules,
        run,
        sorted(rows, key=lambda r: r.case.id),
        sorted(invalid, key=lambda r: r.case.id),
    )


@dataclass(frozen=True)
class Side:
    """One side's suite runs oldest first, and one report per suite holding each case's latest attempt."""

    label: str
    runs: list[Report]
    reports: list[Report]

    @property
    def cases(self) -> int:
        return sum(len(r.rows) + len(r.invalid) for r in self.reports)

    @property
    def replaced(self) -> int:
        """The episodes that a later attempt at the same case replaced."""
        return sum(len(r.rows) + len(r.invalid) for r in self.runs) - self.cases


def suite_runs(path: Path) -> list[Path]:
    """The suite run at path, or every suite run under it."""
    if (path / RESULTS_FILE).exists():
        return [path]
    runs = sorted(d for d in path.iterdir() if (d / RESULTS_FILE).exists())
    if not runs:
        raise ValueError(f"no suite run in {path}")
    return runs


def started(report: Report) -> float:
    run = report.run
    return run.started if run.started is not None else run.time


def load_side(label: str, paths: Iterable[Path]) -> Side:
    """Every suite run under the paths, where a case run again keeps the attempt that started last."""
    runs = sorted((load(d) for path in paths for d in suite_runs(path)), key=started)
    first: dict[str, Report] = {}
    latest: dict[str, dict[str, Row | Invalid]] = {}
    for run in runs:
        first.setdefault(run.suite, run)
        episodes: list[Row | Invalid] = [*run.rows, *run.invalid]
        latest.setdefault(run.suite, {}).update((e.case.id, e) for e in episodes)
    reports = [
        replace(
            first[suite],
            rows=[e for _, e in sorted(cases.items()) if isinstance(e, Row)],
            invalid=[e for _, e in sorted(cases.items()) if isinstance(e, Invalid)],
        )
        for suite, cases in latest.items()
    ]
    return Side(label, runs, sorted(reports, key=lambda r: r.run_dir))


def write(new: Side, out: Path, baseline: Side | None = None) -> Path:
    """The report of the side's runs at out, with a baseline comparison on top and its numbers in out.json."""
    comparison = None if baseline is None else compare(baseline, new)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(render(new.reports, comparison))
    if comparison is not None:
        numbers = out.with_name(out.name + ".json")
        numbers.write_text(json.dumps(to_json(comparison), indent=2) + "\n")
    return out


def _row(case: Case, record: dict[str, object], episode: Path) -> Row:
    metrics = record.get("metrics") or {}
    assert isinstance(metrics, dict)
    final = metrics.get("final_xy")
    return Row(
        case=case,
        episode=episode,
        outcome=str(record["outcome"]),
        signature=_text(metrics.get("signature")),
        signatures=tuple(metrics.get("signatures") or filter(None, [metrics.get("signature")])),
        terminal=_text(record.get("terminal")),
        spl=_float(metrics.get("spl")),
        arrived_s=_float(metrics.get("arrived_s")),
        final_xy=(float(final[0]), float(final[1])) if final else None,
        final_error_xy=_float(metrics.get("final_error_xy")),
        collisions=None if metrics.get("collisions") is None else int(metrics["collisions"]),
        feature_m=_float(metrics.get("feature_m")),
        first_contact_speed=_float(metrics.get("first_contact_speed")),
        near_fall=record["outcome"] != "fall" and bool(metrics.get("trunk_floor_s")),
        mismatch=bool(metrics.get("mismatch")),
        contacts=tuple(_contact(name) for name in metrics.get("contacts") or ()),
    )


def _contact(name: str) -> str:
    """A contact name in today's wording, whichever wording the score was written in."""
    words = name.split()
    return contact_name(*words) if len(words) == 2 else name


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
        for signature in row.signatures:
            table.setdefault(signature, Counter())[row.case.tag] += 1
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


def controls(report: Report) -> list[Row]:
    """The episodes run with the feature removed."""
    return [r for r in report.rows if r.case.level is not None and r.case.level.control]


def encountered(report: Report) -> set[int] | None:
    """The seeds whose control episode walked over the feature, or None for a run with no controls."""
    rows = controls(report)
    if not rows:
        return None
    return {r.case.seed for r in rows if r.feature_m is not None and r.feature_m < GO2.radius}


def by_level(report: Report) -> list[LevelRow]:
    """Outcomes and failure signatures per level of the swept variable, lowest first.

    A case whose seed's control never walked over the feature did not encounter it and is left out.
    """
    seeds = encountered(report)
    groups: dict[float, list[Row]] = {}
    for row in report.rows:
        if row.case.level is not None and not row.case.level.control:
            groups.setdefault(row.case.level.value, []).append(row)
    levels = []
    for value, rows in sorted(groups.items()):
        faced = [r for r in rows if seeds is None or r.case.seed in seeds]
        rest = [r for r in rows if seeds is not None and r.case.seed not in seeds]
        if faced:
            levels.append(
                LevelRow(
                    value=value,
                    outcomes=Counter(r.outcome for r in faced),
                    signatures=Counter(name for r in faced for name in r.signatures),
                    unexplained=sum(r.unexplained for r in faced),
                    left_out=Counter(r.outcome for r in rest),
                    left_out_collisions=sum(bool(r.collisions) for r in rest),
                    near_falls=sum(r.near_fall for r in faced),
                    contacts=Counter(name for r in faced for name in r.contacts),
                )
            )
    return levels


def _curve_y(rate: float) -> int:
    return round(CURVE_PAD_PX / 2 + (1 - rate) * (CURVE_PX[1] - CURVE_PAD_PX))


def level_text(value: float, unit: str) -> str:
    """A level as the page shows it: two decimals with a unit, a whole number for a count."""
    return f"{value:.2f}" if unit else f"{value:.0f}"


def _curve(levels: list[LevelRow], unit: str) -> list[CurvePoint]:
    inner = CURVE_PX[0] - 2 * CURVE_PAD_PX
    span = (levels[-1].value - levels[0].value) or 1.0
    points, last = [], [-CURVE_LABEL_PX, -CURVE_LABEL_PX]
    for row in levels:
        x = round(CURVE_PAD_PX + (row.value - levels[0].value) / span * inner)
        line = 0 if x - last[0] >= CURVE_LABEL_PX else int(last[1] < last[0])
        last[line] = x
        points.append(
            CurvePoint(
                x=x,
                y=_curve_y(row.rate),
                label=level_text(row.value, unit),
                count=f"{row.successes}/{row.n}",
                drop=line * CURVE_ROW_PX,
            )
        )
    return points


@dataclass(frozen=True)
class Shape:
    """A box seen from above. A raised one stands clear of the floor."""

    kind: str
    rect: Rect
    turn_deg: float
    raised: bool


@dataclass(frozen=True)
class Thumbnail:
    """One room of a probe seen from above, in meters."""

    bounds: Rect
    shapes: list[Shape]
    start: Pose
    goal: tuple[float, float]
    feature: tuple[float, float] | None
    image: str | None


def thumbnail(report: Report) -> Thumbnail | None:
    """The sample case's room, or None when its scene is not the one that ran.

    The image is the run's saved rendering as base64. Without one the shapes are drawn.
    """
    case = sample([r.case for r in report.rows] + [r.case for r in report.invalid])
    if case is None:
        return None
    try:
        scene = case.scene()
    except (TypeError, ValueError):
        return None
    if scene.digest() != case.scene_digest:
        return None
    saved = snapshot_path(report.run_dir, case)
    shapes = [
        Shape(
            kind=box.kind,
            rect=(
                box.center[0] - box.half[0],
                box.center[1] - box.half[1],
                box.center[0] + box.half[0],
                box.center[1] + box.half[1],
            ),
            turn_deg=math.degrees(box.yaw),
            raised=box.center[2] - box.half[2] > scene.start[2] + GO2.step,
        )
        for box in scene.boxes
        if box.kind != "floor"
    ]
    return Thumbnail(
        bounds=bounds(scene),
        shapes=sorted(shapes, key=lambda shape: shape.raised),
        start=case.start,
        goal=case.goal[:2],
        feature=case.feature,
        image=base64.b64encode(saved.read_bytes()).decode() if saved.exists() else None,
    )


def _strip_x(meters: float) -> int:
    return round(min(meters / STRIP_M, 1.0) * STRIP_PX)


@dataclass(frozen=True)
class Section:
    """One suite's part of the page."""

    report: Report
    verdict: str
    wall: str
    finished_at: str
    stressors: dict[str, Counter[str]]
    totals: Counter[str]
    signatures: dict[str, Counter[str]]
    difficulty: list[DifficultyRow]
    variable: str
    unit: str
    levels: list[LevelRow]
    controlled: bool
    encountered: str
    curve: list[CurvePoint]
    thumbnail: Thumbnail | None


def _section(report: Report) -> Section:
    run = report.run
    spls = [r.spl for r in report.successes if r.spl is not None]
    spl = f", median SPL of successes {median(spls):.2f}" if spls else ""
    levels = by_level(report)
    seeds = encountered(report)
    met = "" if seeds is None else f"{len(seeds)} of {len(controls(report))}"
    faced = f". Feature encountered in {met} control runs" if met else ""
    probe = PROBES.get(report.suite)
    unit = probe.unit if probe is not None else "m"
    return Section(
        report=report,
        verdict=f"{len(report.successes)} of {len(report.rows)} episodes succeeded{spl}{faced}",
        wall=f"{(run.time - run.started) / 60:.1f} min" if run.started is not None else "unknown",
        finished_at=time.strftime("%Y-%m-%d %H:%M", time.localtime(run.time)),
        stressors=by_stressor(report),
        totals=Counter(r.outcome for r in report.rows),
        signatures=by_signature(report),
        difficulty=by_difficulty(report),
        variable=next((r.case.level.variable for r in report.rows if r.case.level), ""),
        unit=unit,
        levels=levels,
        controlled=seeds is not None,
        encountered=met,
        curve=_curve(levels, unit) if levels else [],
        thumbnail=thumbnail(report) if levels else None,
    )


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
class ProbeRow:
    """One probe's comparison."""

    probe: str
    tally: Tally
    levels: list[LevelTally]
    measures: list[Measure]


@dataclass(frozen=True)
class Comparison:
    """The runs against a baseline, case by case, with the latest episode of every case on each side."""

    base: Side
    new: Side
    base_rows: dict[str, Row]
    new_rows: dict[str, Row]
    pairs: list[Pair]
    left_out: dict[str, Counter[Reason]]
    scene_differs: list[str]
    probes: list[ProbeRow]

    @property
    def changed(self) -> list[Pair]:
        return [p for p in self.pairs if p.changed]


@dataclass(frozen=True)
class Bar:
    """A signed change drawn from the middle, as a share of the cases."""

    x: int
    width: int
    direction: Literal["up", "down", "flat"]
    significant: bool


@dataclass(frozen=True)
class Detail:
    """One line of the run details, for each side."""

    name: str
    base: str
    new: str


def _rows(side: Side) -> dict[str, Row]:
    return {r.case.id: r for report in side.reports for r in report.rows}


def _attempts(side: Side) -> dict[str, Attempt]:
    attempts = {r.case.id: Attempt(r.case, None) for report in side.reports for r in report.invalid}
    return attempts | {case_id: Attempt(r.case, r.outcome) for case_id, r in _rows(side).items()}


def _measures(row: Row) -> dict[str, float | None]:
    return {
        "first_contact_speed": row.first_contact_speed,
        "final_error_xy": row.final_error_xy,
        "arrived_s": row.arrived_s,
        "spl": row.spl,
    }


def measures(pairs: list[Pair], base: dict[str, Row], new: dict[str, Row]) -> list[Measure]:
    """Medians of each continuous measure over the pairs that have it on both sides."""
    found = []
    for name in MEASURES:
        values = [
            (b, n)
            for p in pairs
            if (b := _measures(base[p.case.id])[name]) is not None
            and (n := _measures(new[p.case.id])[name]) is not None
        ]
        if values:
            found.append(
                Measure(
                    name, len(values), median(b for b, _ in values), median(n for _, n in values)
                )
            )
    return found


def compare(base: Side, new: Side) -> Comparison:
    """Pair the sides by case and tally every probe."""
    pairing = pair(_attempts(base), _attempts(new))
    base_rows, new_rows = _rows(base), _rows(new)
    groups: dict[str, list[Pair]] = {}
    for p in pairing.pairs:
        groups.setdefault(p.case.tag, []).append(p)
    probes = [
        ProbeRow(probe, tally(group), level_tallies(group), measures(group, base_rows, new_rows))
        for probe, group in sorted(groups.items())
    ]
    return Comparison(
        base,
        new,
        base_rows,
        new_rows,
        pairing.pairs,
        pairing.left_out,
        pairing.scene_differs,
        probes,
    )


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
    for field_name, ran in (
        ("machine", "on different machines"),
        ("procs", "a different number of episodes at once"),
    ):
        values = {name: _recorded(side, field_name) for name, side in _sides(comparison)}
        if values["base"] and values["new"] and values["base"] != values["new"]:
            found.append(
                f"The sides ran {ran}, base {', '.join(values['base'])}"
                f" and new {', '.join(values['new'])}."
            )
    if comparison.scene_differs:
        found.append(
            f"{len(comparison.scene_differs)} cases have a different scene on each side"
            " and are left out."
        )
    return found


def _sides(comparison: Comparison) -> list[tuple[str, Side]]:
    return [("base", comparison.base), ("new", comparison.new)]


def _recorded(side: Side, field_name: str) -> list[str]:
    """The values the side's runs recorded for the field. Older runs recorded none."""
    values = (getattr(r.run, field_name) for r in side.runs)
    return sorted({str(v) for v in values if v is not None})


def _days(side: Side) -> list[str]:
    return sorted({time.strftime("%Y-%m-%d", time.localtime(started(r))) for r in side.runs})


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
        "episodes": side.cases,
        "replaced": side.replaced,
        "runs": [
            {
                "path": str(r.run_dir.resolve()),
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


def recorded(row: Row) -> bool:
    return (row.episode / RECORDING_FILE).exists()


def _episode_json(row: Row) -> dict[str, Json]:
    return {"outcome": row.outcome, "episode": str(row.episode), "recorded": recorded(row)}


def to_json(comparison: Comparison) -> dict[str, Json]:
    """The comparison's numbers, for agents."""
    verdicts = Counter(row.tally.verdict for row in comparison.probes)
    total = tally(comparison.pairs)
    return {
        "base": _side_json(comparison.base),
        "new": _side_json(comparison.new),
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
                "base": _episode_json(comparison.base_rows[p.case.id]),
                "new": _episode_json(comparison.new_rows[p.case.id]),
            }
            for p in comparison.changed
        ],
    }


def bar(t: Tally) -> Bar:
    half = BAR_PX // 2
    width = round(abs(t.change) / t.cases * half) if t.cases else 0
    direction: Literal["up", "down", "flat"] = (
        "up" if t.change > 0 else "down" if t.change < 0 else "flat"
    )
    return Bar(
        half - width if t.change < 0 else half, width, direction, t.verdict != "no clear change"
    )


def failures(counts: Counter[str]) -> str:
    return ", ".join(f"{FAILURE_NAMES.get(o, o)} {counts[o]}" for o in OUTCOMES if counts[o])


def level_label(probe: str, value: float | None) -> str:
    if value is None:
        return "control"
    spec = PROBES.get(probe)
    unit = spec.unit if spec is not None else "m"
    return level_text(value, unit) + (f" {unit}" if unit else "")


def pair_level(p: Pair) -> str:
    level = p.case.level
    if level is None:
        return ""
    return level_label(p.case.tag, None if level.control else level.value)


def direction(p: Pair) -> str:
    return {1: "became success", -1: "became failure"}.get(p.flip, "other change")


def signed(value: float, digits: int = 0) -> str:
    text = f"{abs(value):.{digits}f}"
    if float(text) == 0:
        return text
    return ("+" if value > 0 else "\u2212") + text


def p_text(p: float) -> str:
    return f"{p:.3f}" if p >= 0.001 else f"{p:.0e}"


def details(base: Side, new: Side) -> list[Detail]:
    """What each side ran, one line per field."""
    sides = [_details(base), _details(new)]
    return [Detail(name, sides[0][name], sides[1][name]) for name in sides[0]]


def _details(side: Side) -> dict[str, str]:
    runs = [r.run for r in side.runs]
    commits = [(r.commit or "unknown")[:10] + (", dirty" if r.dirty else "") for r in runs]
    return {
        "runs": f"{len(side.runs)} suite runs, {side.cases} cases",
        "suites": _join(sorted(r.suite for r in side.runs)),
        "commit": _join(commits),
        "policy": _join([r.policy or "none given" for r in runs]),
        "overrides": _join([" ".join(r.overrides) for r in runs if r.overrides]),
        "date": _join(_days(side)),
        "procs": _join(_recorded(side, "procs"), "not recorded"),
        "machine": _join(_recorded(side, "machine"), "not recorded"),
        "run again": f"{side.replaced} cases, latest kept" if side.replaced else "none",
    }


def _join(values: list[str], empty: str = "none") -> str:
    return ", ".join(dict.fromkeys(values)) or empty


def _comparison_context(comparison: Comparison) -> dict[str, object]:
    changed = sorted(
        comparison.changed, key=lambda p: (p.case.tag, -p.flip, p.case.seed, p.case.id)
    )
    return {
        "c": comparison,
        "summary": to_json(comparison)["summary"],
        "warnings": warnings(comparison),
        "details": details(comparison.base, comparison.new),
        "changed": changed,
        "changed_probes": sorted({p.case.tag for p in changed}),
        "baseline": {
            case_id: a.outcome or "invalid" for case_id, a in _attempts(comparison.base).items()
        },
        "bar": bar,
        "bar_px": BAR_PX,
        "failures": failures,
        "level_label": level_label,
        "pair_level": pair_level,
        "direction": direction,
        "recorded": recorded,
        "written": time.strftime("%Y-%m-%d %H:%M"),
    }


def render(reports: list[Report], comparison: Comparison | None = None) -> str:
    """The whole page from the template beside this module, one section per suite, a comparison first when given."""
    env = Environment(loader=FileSystemLoader(Path(__file__).parent), autoescape=True)
    env.filters["quote"] = shlex.quote
    env.filters["degrees"] = math.degrees
    env.filters["level"] = level_text
    env.filters["signed"] = signed
    env.filters["p"] = p_text
    return env.get_template(TEMPLATE_FILE).render(
        sections=[_section(report) for report in reports],
        glossary=definitions(reports[0].rules),
        curve_px=CURVE_PX,
        thumbnail_px=THUMBNAIL_PX,
        snapshot_px=SNAPSHOT_PX,
        curve_grid=[(f"{rate:.0%}", _curve_y(rate)) for rate in (0.0, 0.5, 1.0)],
        strip_m=STRIP_M,
        strip_px=STRIP_PX,
        strip_ticks=[_strip_x(k * STRIP_TICK_M) for k in range(int(STRIP_M / STRIP_TICK_M) + 1)],
        **({} if comparison is None else _comparison_context(comparison)),
    )
