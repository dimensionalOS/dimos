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

"""Nav-3d evaluation CLI.

Run every suite:      python -m dimos.navigation.nav_3d.evaluator run
One dataset:          python -m dimos.navigation.nav_3d.evaluator run --dataset mid360_athens_stairs
Only some cases:      python -m dimos.navigation.nav_3d.evaluator run --tag stairs --tag up
Machine output:       python -m dimos.navigation.nav_3d.evaluator run --json report.json
Override a gate:      python -m dimos.navigation.nav_3d.evaluator run --set goal_tolerance=0.4
Compare two runs:     python -m dimos.navigation.nav_3d.evaluator diff old.json new.json
Determinism check:    run twice with --json, then diff a.json b.json --exact
New dataset:          python -m dimos.navigation.nav_3d.evaluator ingest recordings/.../mem2.db --name office_a
Pick cases by click:  python -m dimos.navigation.nav_3d.evaluator pick-case office_a
Curate by coords:     python -m dimos.navigation.nav_3d.evaluator add-case office_a --start x y z --goal x y z
Flag dynamic route:   python -m dimos.navigation.nav_3d.evaluator tag office_a auto_03 --final-fail
Recompute tags:       python -m dimos.navigation.nav_3d.evaluator retag office_a
"""

from __future__ import annotations

import contextlib
import dataclasses
import json
import os
from pathlib import Path
import sqlite3
from typing import TYPE_CHECKING

import numpy as np
import typer

from dimos.navigation.nav_3d.evaluator import tripwire
from dimos.navigation.nav_3d.evaluator.cases import (
    CASES_DIR,
    Suite,
    load_suite,
    load_suites,
    save_suite,
)
from dimos.navigation.nav_3d.evaluator.config import EvalConfig
from dimos.navigation.nav_3d.evaluator.curation import CurationError, load_store
from dimos.navigation.nav_3d.evaluator.final_map import load_or_build_final_map
from dimos.navigation.nav_3d.evaluator.generate import GenerationParams, generate_cases
from dimos.navigation.nav_3d.evaluator.metrics import ground_truth_route
from dimos.navigation.nav_3d.evaluator.recording import load_trajectory
from dimos.navigation.nav_3d.evaluator.runner import Report, evaluate
from dimos.navigation.nav_3d.evaluator.tagging import GEOMETRIC_TAGS, route_tags
from dimos.utils.data import get_data_dir

if TYPE_CHECKING:
    from dimos.navigation.nav_3d.evaluator.curation import CaseStore
    from dimos.navigation.nav_3d.evaluator.final_map import FinalMap
    from dimos.navigation.nav_3d.evaluator.runner import PlanOutcome

app = typer.Typer(no_args_is_help=True, add_completion=False)


def _apply_overrides(cfg: EvalConfig, overrides: list[str]) -> EvalConfig:
    fields = {f.name for f in dataclasses.fields(EvalConfig)}
    for spec in overrides:
        if "=" not in spec:
            raise typer.BadParameter(f"--set expects name=value, got {spec!r}")
        name, value = spec.split("=", 1)
        if name.startswith("planner."):
            # Planner constructor arguments are validated by the planner, which
            # owns their names and defaults.
            cfg.planner[name.removeprefix("planner.")] = float(value)
            continue
        if name not in fields:
            raise typer.BadParameter(f"unknown config field {name!r}")
        current = getattr(cfg, name)
        setattr(cfg, name, type(current)(value))
    return cfg


def _score_cell(outcome: PlanOutcome) -> str:
    """No path at all shows x. A planned path shows its SPL, which is 0.00 when
    the path is invalid and higher when it is valid."""
    return "x" if not outcome.planned and not outcome.success else f"{outcome.spl:.2f}"


def _print_report(report: Report) -> None:
    header = (
        f"{'case':<28} {'dataset':<22} {'inc':>5} {'fin':>5} "
        f"{'len':>6} {'ref':>6} {'clr':>6} {'vox':>8} {'ms':>7}"
    )
    print(header)
    print("-" * len(header))
    for d in report.datasets:
        for c in d.cases:
            inc = "-" if c.final_only else _score_cell(c.online)
            no_path = not c.online.planned and not c.online.success
            length = "x" if no_path else f"{c.online.length:.1f}"
            clr = (
                f"{c.online.min_clearance:>6.2f}" if c.online.min_clearance is not None else " " * 6
            )
            print(
                f"{c.id:<28} {c.dataset:<22} "
                f"{inc:>5} {_score_cell(c.final):>5} "
                f"{length:>6} {c.l_ref:>6.1f} "
                f"{clr} "
                f"{c.online_voxels:>8d} {c.online.plan_ms:>7.1f}"
            )
    print("-" * len(header))
    for d in report.datasets:
        print(
            f"{d.dataset}: {d.frames} frames, "
            f"final {d.final_voxels} voxels, "
            f"map build {d.map_build_ms / 1000:.1f}s"
        )
    print(f"\n{'by tag':<12} {'inc':>5} {'fin':>5} {'n':>4}")
    for tag, s in report.by_tag.items():
        inc = f"{s.inc_score:.2f}" if s.n_online else "-"
        print(f"{tag:<12} {inc:>5} {s.fin_score:>5.2f} {s.n:>4}")
    print(
        f"\nscore {report.score:.3f} | soft {report.score_soft:.3f} | "
        f"final {report.final_score:.3f} | "
        f"success inc {report.n_success}/{report.n_online} "
        f"fin {report.n_success_final}/{report.n_cases} | "
        f"outcomes {report.outcome_counts} | "
        f"plan p95 {report.plan_ms['p95']:.1f}ms | "
        f"map update p95 {report.map_update_ms['p95']:.0f}ms"
    )
    inc_only = [
        f"{c.dataset}/{c.id}"
        for d in report.datasets
        for c in d.cases
        if c.online.success and not c.final.success
    ]
    if inc_only:
        candidates = set(report.dynamic_candidates)
        others = [x for x in inc_only if x not in candidates]
        print(f"\nincremental-only ({len(inc_only)}) — passed online, failed final:")
        if report.dynamic_candidates:
            print(f"  dynamic-obstacle candidates: {', '.join(report.dynamic_candidates)}")
            print("    review with --rrd, confirm: evaluator tag <dataset> <id> --final-fail")
        if others:
            print(f"  not explained by a new obstacle, inspect final map: {', '.join(others)}")


@app.command("diff")
def diff_reports(
    old: Path = typer.Argument(..., help="Baseline report JSON from `run --json`"),
    new: Path = typer.Argument(..., help="Candidate report JSON from `run --json`"),
    exact: bool = typer.Option(
        False,
        "--exact",
        help="Require bit-identical results (ignoring timings); "
        "two runs of the same code must pass",
    ),
) -> None:
    """Name every case whose pass/fail flipped between two runs.

    Exits 1 when any case regressed, so a CI check or before/after comparison
    can gate on it. Perf budget breaches always print but only exit 1 with
    --exact, which also exits 1 on any non-timing difference. Running the suite
    twice and exact-diffing the reports is the determinism check.
    """
    old_report = json.loads(old.read_text())
    new_report = json.loads(new.read_text())
    print(
        f"score {old_report['score']:.3f} -> {new_report['score']:.3f} | "
        f"final {old_report['final_score']:.3f} -> {new_report['final_score']:.3f}"
    )
    d = tripwire.diff(old_report, new_report)
    print(f"{len(d.fixed)} fixed, {len(d.broke)} broke")
    for flip in d.fixed:
        print(f"  fixed: {flip.key} ({flip.test})")
    for flip in d.broke:
        print(f"  BROKE: {flip.key} ({flip.test}: pass -> fail)")
    for key in d.added:
        print(f"  new case: {key}")
    for key in d.removed:
        print(f"  case gone: {key}")
    violations = tripwire.perf_violations(new_report)
    for violation in violations:
        print(f"PERF BUDGET EXCEEDED: {violation}")
    if violations and not exact:
        print("  advisory here; the --exact confirmation run is the binding perf gate")
    if exact:
        differences = tripwire.exact_differences(old_report, new_report)
        if differences:
            shown = 20
            print(f"{len(differences)} exact difference(s):")
            for line in differences[:shown]:
                print(f"  {line}")
            if len(differences) > shown:
                print(f"  ... and {len(differences) - shown} more")
            raise typer.Exit(code=1)
        print("exact: reports identical")
    if d.broke or (exact and violations):
        raise typer.Exit(code=1)


@app.command()
def run(
    manifests: list[Path] = typer.Argument(
        None, help="Suite YAMLs; defaults to every manifest under cases/"
    ),
    dataset: str = typer.Option(None, "--dataset", help="Only run suites for this dataset"),
    tag: list[str] = typer.Option(
        None, "--tag", help="Only run cases carrying every given tag, e.g. --tag stairs --tag up"
    ),
    json_out: Path = typer.Option(None, "--json", help="Write the full report as JSON"),
    rrd_out: Path = typer.Option(None, "--rrd", help="Write a rerun recording of every case"),
    workers: int = typer.Option(
        os.cpu_count() or 1,
        "--workers",
        help="Total parallelism: dataset processes x checkpoint threads",
    ),
    set_: list[str] = typer.Option(
        None, "--set", help="Repeatable EvalConfig override, e.g. goal_tolerance=0.4"
    ),
) -> None:
    """Evaluate every case suite and print scores. The headline is incremental-map SPL."""
    suites = load_suites(manifests or None)
    if dataset is not None:
        wanted = Path(dataset).stem
        suites = [
            s
            for s in suites
            if s.dataset == dataset or (s.path is not None and s.path.stem == wanted)
        ]
        if not suites:
            raise typer.BadParameter(f"no suite for dataset {dataset!r}")
    if tag:
        wanted_tags = set(tag)
        for s in suites:
            s.cases = [c for c in s.cases if wanted_tags <= set(c.tags)]
        suites = [s for s in suites if s.cases]
        if not suites:
            raise typer.BadParameter(f"no cases carry all tags {tag}")
    cfg = _apply_overrides(EvalConfig(), set_ or [])
    report = evaluate(suites, cfg, workers=workers, keep_artifacts=rrd_out is not None)
    _print_report(report)
    for violation in tripwire.perf_violations(report.to_dict()):
        print(f"PERF BUDGET EXCEEDED: {violation}")
    if json_out is not None:
        json_out.parent.mkdir(parents=True, exist_ok=True)
        json_out.write_text(json.dumps(report.to_dict(), indent=2))
        print(f"wrote {json_out}")
    if rrd_out is not None:
        # Lazy: viz pulls in rerun, only needed with --rrd.
        from dimos.navigation.nav_3d.evaluator.viz import write_rrd

        write_rrd(report, suites, cfg, rrd_out)


def _copy_recording(src: Path, dest: Path) -> None:
    """Copy via the sqlite backup API so WAL sidecar content is never lost."""
    with (
        contextlib.closing(sqlite3.connect(src)) as source,
        contextlib.closing(sqlite3.connect(dest)) as target,
    ):
        source.backup(target)


@app.command()
def ingest(
    source: Path = typer.Argument(
        ..., help="Recording to ingest: a mem2.db file or the directory holding one"
    ),
    name: str = typer.Option(..., "--name", help="Dataset name; becomes data/<name>.db"),
    lidar_stream: str = typer.Option("pointlio_lidar", "--lidar-stream"),
    odom_stream: str = typer.Option("pointlio_odometry", "--odom-stream"),
    cases: int = typer.Option(
        0, "--cases", help="Exact auto-generated case count; 0 scales with recording length"
    ),
    external: bool = typer.Option(
        False,
        "--external",
        help="Reference the recording in place instead of copying it into data/; "
        "keeps it out of the LFS flow",
    ),
    force: bool = typer.Option(False, "--force", help="Overwrite dataset and manifest"),
) -> None:
    """Register a recording as a dataset: copy, map, generate cases."""
    src = source / "mem2.db" if source.is_dir() else source
    if not src.exists():
        raise typer.BadParameter(f"{src} does not exist")
    manifest = CASES_DIR / f"{name}.yaml"
    if manifest.exists() and not force:
        raise typer.BadParameter(f"{manifest} already exists; pass --force to regenerate")
    if external:
        dest = src.resolve()
    else:
        dest = get_data_dir() / f"{name}.db"
        if src.resolve() != dest.resolve():
            if dest.exists() and not force:
                raise typer.BadParameter(f"{dest} already exists; pass --force to overwrite")
            print(f"copying {src} -> {dest}")
            _copy_recording(src, dest)

    suite = Suite(
        dataset=name,
        cases=[],
        lidar_stream=lidar_stream,
        odom_stream=odom_stream,
        db=str(dest) if external else None,
    )
    trajectory = load_trajectory(dest, odom_stream)
    arcs = trajectory.arc_lengths()
    print(
        f"trajectory: {len(trajectory.positions)} poses, "
        f"{trajectory.ts[-1] - trajectory.ts[0]:.0f}s, {arcs[-1]:.1f}m walked, "
        f"z [{trajectory.positions[:, 2].min():.2f}, {trajectory.positions[:, 2].max():.2f}]"
    )
    cfg = EvalConfig()
    final = load_or_build_final_map(dest, suite, cfg)
    planner = cfg.make_planner()
    planner.update_global_map(final.occupied)
    gen = GenerationParams(max_cases=cases or None)
    if cases:
        gen.min_cases = cases
    suite.cases = generate_cases(trajectory, final, planner.surface_map(), cfg, gen)
    if not suite.cases:
        raise typer.Exit(code=1)
    floor = min(gen.min_cases, gen.resolve_max_cases(float(arcs[-1])))
    if len(suite.cases) < floor:
        print(
            f"WARNING: only {len(suite.cases)} cases generated; the recording "
            "may be too short or too uniform for more"
        )
    path = save_suite(suite, manifest)
    print(f"\n{len(suite.cases)} cases -> {path}")
    for case in suite.cases:
        print(f"  {case.id}: [{', '.join(case.tags)}]")
    print(f"\nrun with: python -m dimos.navigation.nav_3d.evaluator run --dataset {name}")


def _open(dataset: str) -> tuple[CaseStore, FinalMap]:
    try:
        return load_store(dataset)
    except CurationError as err:
        raise typer.BadParameter(str(err)) from err


@app.command("add-case")
def add_case(
    dataset: str = typer.Argument(..., help="Dataset whose manifest gets the case"),
    start: tuple[float, float, float] = typer.Option(..., "--start", help="Foot-level xyz"),
    goal: tuple[float, float, float] = typer.Option(..., "--goal", help="Foot-level xyz"),
    case_id: str = typer.Option(None, "--id", help="Case id; default manual_<n> or neg_<n>"),
    tags: str = typer.Option(None, "--tags", help="Comma-separated tags"),
    expect_fail: bool = typer.Option(
        False, "--expect-fail", help="Certified-infeasible pair; the planner must refuse"
    ),
) -> None:
    """Append a curated case, with endpoints snapped to the final surface."""
    store, _ = _open(dataset)
    try:
        store.add(
            start,
            goal,
            [t.strip() for t in (tags or "").split(",") if t.strip()],
            case_id=case_id,
            expect_fail=expect_fail,
        )
    except CurationError as err:
        raise typer.BadParameter(str(err)) from err


@app.command("tag")
def tag_case(
    dataset: str = typer.Argument(..., help="Dataset whose manifest holds the case"),
    case_id: str = typer.Argument(..., help="Case id to edit"),
    final_fail: bool = typer.Option(
        True,
        "--final-fail/--no-final-fail",
        help="Mark a dynamic-obstacle route: online path expected, final path expected refused",
    ),
) -> None:
    """Flag an auto case as a dynamic-obstacle route, e.g. a door that closed.

    The online plan is still scored normally, the final plan is scored 1.0
    for refusing. Use it on a case that shows up as incremental-only because a
    real obstacle blocked the route by the final map, not a planner bug.
    """
    manifest = CASES_DIR / f"{dataset}.yaml"
    if not manifest.exists():
        raise typer.BadParameter(f"no manifest {manifest}")
    suite = load_suite(manifest)
    case = next((c for c in suite.cases if c.id == case_id), None)
    if case is None:
        raise typer.BadParameter(f"case {case_id!r} not found in {manifest}")
    if final_fail and case.expect_fail:
        raise typer.BadParameter(
            f"case {case_id!r} is expect_fail; a case cannot be both infeasible and dynamic"
        )
    case.expect_final_fail = final_fail
    tags = [t for t in case.tags if t != "dynamic"]
    case.tags = [*tags, "dynamic"] if final_fail else tags
    save_suite(suite, manifest)
    flag = "expect_final_fail" if final_fail else "cleared expect_final_fail"
    print(f"{case.id}: {flag} [{', '.join(case.tags)}]")


@app.command("retag")
def retag(
    dataset: str = typer.Argument(..., help="Dataset whose manifest gets retagged"),
) -> None:
    """Recompute geometric tags for auto-generated cases from the final map.

    Only the geometric tags (flat, stairs, narrow, switchback, and the rest)
    are replaced, so improving the tagger never churns start/goal pairs. The
    auto provenance tag survives. Manually curated cases are left untouched:
    their tags are human intent, not something to recompute.
    """
    manifest = CASES_DIR / f"{dataset}.yaml"
    if not manifest.exists():
        raise typer.BadParameter(f"no manifest {manifest}; run ingest first")
    suite = load_suite(manifest)
    cfg = EvalConfig()
    final = load_or_build_final_map(suite.db_path(), suite, cfg)
    trajectory = load_trajectory(suite.db_path(), suite.odom_stream)
    changed = 0
    for case in suite.cases:
        if "auto" not in case.tags:
            print(f"  {case.id}: curated, tags kept [{', '.join(case.tags)}]")
            continue
        route = ground_truth_route(trajectory, case.start, case.goal, cfg)
        if route is None:
            print(f"  {case.id}: off-trajectory, tags kept [{', '.join(case.tags)}]")
            continue
        provenance = [t for t in case.tags if t not in GEOMETRIC_TAGS]
        geo = route_tags(case.start, case.goal, route, final.occupied_keys, cfg)
        new_tags = provenance + [t for t in geo if t not in provenance]
        if new_tags != case.tags:
            changed += 1
            print(f"  {case.id}: [{', '.join(case.tags)}] -> [{', '.join(new_tags)}]")
        case.tags = new_tags
    save_suite(suite, manifest)
    print(f"\nretagged {changed} case(s) in {manifest.name}")


@app.command("pick-case")
def pick_case(
    dataset: str = typer.Argument(..., help="Dataset whose manifest gets the cases"),
) -> None:
    """Pick and edit cases by shift+clicking the map in a browser viewer.

    Serves the final map, the walked path, and every case already in the
    manifest as an editable panel entry. Shift+click picks new start/goal
    pairs. Any case can be renamed, retagged, flipped negative, or deleted.
    New pairs save to the manifest snapped like add-case.
    """
    # Lazy: picker/viz pull in viser and matplotlib, only needed for pick-case.
    from dimos.navigation.nav_3d.evaluator.picker import pick_cases
    from dimos.navigation.nav_3d.evaluator.viz import turbo_by_height

    store, final = _open(dataset)
    trajectory = load_trajectory(store.suite.db_path(), store.suite.odom_stream)
    foot = trajectory.positions - np.array([0.0, 0.0, store.cfg.robot_height], dtype=np.float32)
    pick_cases(
        dataset,
        final.occupied,
        turbo_by_height(final.occupied),
        final.voxel_size,
        foot,
        store,
    )
    print(f"\nrun with: python -m dimos.navigation.nav_3d.evaluator run --dataset {dataset}")


@app.command("list")
def list_cases() -> None:
    """Print every dataset's cases with endpoints and tags."""
    for suite in load_suites():
        print(f"{suite.dataset} ({suite.path.name if suite.path else '?'})")
        for case in suite.cases:
            tags = f" [{', '.join(case.tags)}]" if case.tags else ""
            print(
                f"  {case.id}: {tuple(round(v, 2) for v in case.start)} -> "
                f"{tuple(round(v, 2) for v in case.goal)}{tags}"
            )


if __name__ == "__main__":
    app()
