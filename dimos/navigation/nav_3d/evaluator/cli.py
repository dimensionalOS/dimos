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
Override a knob:      python -m dimos.navigation.nav_3d.evaluator run --set wall_clearance_m=0.05
Compare two runs:     python -m dimos.navigation.nav_3d.evaluator diff old.json new.json
New dataset:          python -m dimos.navigation.nav_3d.evaluator ingest recordings/.../mem2.db --name office_a
Pick cases by click:  python -m dimos.navigation.nav_3d.evaluator pick-case office_a
Curate by coords:     python -m dimos.navigation.nav_3d.evaluator add-case office_a --start x y z --goal x y z
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
    Case,
    Suite,
    load_suite,
    load_suites,
    save_suite,
)
from dimos.navigation.nav_3d.evaluator.config import EvalConfig
from dimos.navigation.nav_3d.evaluator.final_map import load_or_build_final_map
from dimos.navigation.nav_3d.evaluator.generate import (
    GenerationParams,
    drift_stats,
    generate_cases,
    snap_to_surface,
)
from dimos.navigation.nav_3d.evaluator.recording import load_trajectory
from dimos.navigation.nav_3d.evaluator.runner import Report, evaluate
from dimos.utils.data import get_data_dir, resolve_named_path

if TYPE_CHECKING:
    from numpy.typing import NDArray

app = typer.Typer(no_args_is_help=True, add_completion=False)


def _apply_overrides(cfg: EvalConfig, overrides: list[str]) -> EvalConfig:
    fields = {f.name: f.type for f in dataclasses.fields(EvalConfig)}
    for spec in overrides:
        if "=" not in spec:
            raise typer.BadParameter(f"--set expects name=value, got {spec!r}")
        name, value = spec.split("=", 1)
        if name not in fields:
            raise typer.BadParameter(f"unknown config field {name!r}")
        current = getattr(cfg, name)
        setattr(cfg, name, type(current)(value))
    return cfg


def _print_report(report: Report) -> None:
    header = (
        f"{'case':<28} {'dataset':<22} {'inc':>5} {'fin':>5} "
        f"{'len':>6} {'ref':>6} {'miss':>6} {'clr':>6} {'vox':>8} {'ms':>7}"
    )
    print(header)
    print("-" * len(header))
    for d in report.datasets:
        for c in d.cases:
            clr = (
                f"{c.online.min_clearance:>6.2f}" if c.online.min_clearance is not None else " " * 6
            )
            print(
                f"{c.id:<28} {c.dataset:<22} "
                f"{c.online.spl:>5.2f} {c.final.spl:>5.2f} "
                f"{c.online.length:>6.1f} {c.l_ref:>6.1f} "
                f"{c.online.goal_miss:>6.1f} {clr} "
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
        print(f"{tag:<12} {s.inc_score:>5.2f} {s.fin_score:>5.2f} {s.n:>4}")
    print(
        f"\nscore {report.score:.3f} | soft {report.score_soft:.3f} | "
        f"final {report.final_score:.3f} | "
        f"success inc {report.n_success}/{report.n_cases} "
        f"fin {report.n_success_final}/{report.n_cases} | "
        f"outcomes {report.outcome_counts} | "
        f"plan p95 {report.plan_ms['p95']:.1f}ms | "
        f"map update p95 {report.map_update_ms['p95']:.0f}ms"
    )


@app.command("diff")
def diff_reports(
    old: Path = typer.Argument(..., help="Baseline report JSON from `run --json`"),
    new: Path = typer.Argument(..., help="Candidate report JSON from `run --json`"),
) -> None:
    """Name every case whose pass/fail flipped between two runs.

    Exits 1 when any case regressed, so a keep/discard loop can gate on it.
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
    if d.broke:
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
        None, "--set", help="Repeatable EvalConfig override, e.g. wall_clearance_m=0.05"
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
    report = evaluate(suites, cfg, workers=workers)
    _print_report(report)
    if json_out is not None:
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


def _snap_or_fail(
    label: str,
    point: tuple[float, float, float],
    surface: NDArray[np.float32],
    snap_max_m: float,
) -> tuple[float, float, float]:
    snapped = snap_to_surface(np.asarray(point, dtype=np.float32), surface, snap_max_m)
    if snapped is None:
        raise typer.BadParameter(
            f"{label} {point} is more than {snap_max_m}m from any standable surface"
        )
    return (float(snapped[0]), float(snapped[1]), float(snapped[2]))


@app.command()
def ingest(
    source: Path = typer.Argument(
        ..., help="Recording to ingest: a mem2.db file or the directory holding one"
    ),
    name: str = typer.Option(..., "--name", help="Dataset name; becomes data/<name>.db"),
    lidar_stream: str = typer.Option("pointlio_lidar", "--lidar-stream"),
    odom_stream: str = typer.Option("pointlio_odometry", "--odom-stream"),
    max_cases: int = typer.Option(
        0, "--max-cases", help="Auto-generated case cap; 0 scales with recording length"
    ),
    force: bool = typer.Option(False, "--force", help="Overwrite dataset and manifest"),
) -> None:
    """Register a recording as a dataset: copy, drift-check, map, generate cases."""
    src = source / "mem2.db" if source.is_dir() else source
    if not src.exists():
        raise typer.BadParameter(f"{src} does not exist")
    manifest = CASES_DIR / f"{name}.yaml"
    if manifest.exists() and not force:
        raise typer.BadParameter(f"{manifest} already exists; pass --force to regenerate")
    dest = get_data_dir() / f"{name}.db"
    if src.resolve() != dest.resolve():
        if dest.exists() and not force:
            raise typer.BadParameter(f"{dest} already exists; pass --force to overwrite")
        print(f"copying {src} -> {dest}")
        _copy_recording(src, dest)

    suite = Suite(dataset=name, cases=[], lidar_stream=lidar_stream, odom_stream=odom_stream)
    trajectory = load_trajectory(dest, odom_stream)
    arcs = trajectory.arc_lengths()
    print(
        f"trajectory: {len(trajectory.positions)} poses, "
        f"{trajectory.ts[-1] - trajectory.ts[0]:.0f}s, {arcs[-1]:.1f}m walked, "
        f"z [{trajectory.positions[:, 2].min():.2f}, {trajectory.positions[:, 2].max():.2f}]"
    )
    drift = drift_stats(trajectory)
    closure = f"{drift.closure_m:.2f}m" if drift.closure_m is not None else "n/a"
    print(
        f"drift: {drift.revisit_count} same-floor revisits, "
        f"z mismatch p95 {drift.revisit_dz_p95:.2f}m, loop closure {closure}"
    )
    for warning in drift.warnings:
        print(f"WARNING: {warning}")

    cfg = EvalConfig()
    final = load_or_build_final_map(dest, suite, cfg)
    planner = cfg.make_planner()
    planner.update_global_map(final.occupied)
    gen = GenerationParams(max_cases=max_cases or None)
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
        print(f"  {case.id}: w={case.weight:g} [{', '.join(case.tags)}]")
    print(f"\nrun with: python -m dimos.navigation.nav_3d.evaluator run --dataset {name}")


def _append_case(
    suite: Suite,
    manifest: Path,
    surface: NDArray[np.float32],
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    case_id: str | None,
    tags: list[str],
    weight: float,
    snap_max: float,
    expect_fail: bool,
) -> Case:
    prefix = "neg" if expect_fail else "manual"
    snapped_goal = snap_to_surface(np.asarray(goal, dtype=np.float32), surface, snap_max)
    if snapped_goal is not None:
        goal = (float(snapped_goal[0]), float(snapped_goal[1]), float(snapped_goal[2]))
    elif expect_fail:
        # An infeasible goal may sit on geometry with no standable surface.
        print(f"note: goal {goal} is off any standable surface; keeping it as picked")
    else:
        raise typer.BadParameter(f"goal {goal} is more than {snap_max}m from standable surface")
    case = Case(
        id=case_id or f"{prefix}_{sum(c.id.startswith(f'{prefix}_') for c in suite.cases):02d}",
        start=_snap_or_fail("start", start, surface, snap_max),
        goal=goal,
        weight=weight,
        tags=tags,
        expect_fail=expect_fail,
    )
    if any(c.id == case.id for c in suite.cases):
        raise typer.BadParameter(f"case id {case.id!r} already exists in {manifest}")
    suite.cases.append(case)
    save_suite(suite, manifest)
    kind = "negative (must refuse)" if expect_fail else "positive"
    print(f"added {kind} {case.id}: {case.start} -> {case.goal} to {manifest}")
    return case


def _load_for_curation(dataset: str) -> tuple[Suite, Path, NDArray[np.float32], EvalConfig]:
    manifest = CASES_DIR / f"{dataset}.yaml"
    if not manifest.exists():
        raise typer.BadParameter(f"no manifest {manifest}; run ingest first")
    suite = load_suite(manifest)
    cfg = EvalConfig()
    final = load_or_build_final_map(resolve_named_path(dataset, ".db"), suite, cfg)
    planner = cfg.make_planner()
    planner.update_global_map(final.occupied)
    return suite, manifest, planner.surface_map(), cfg


@app.command("add-case")
def add_case(
    dataset: str = typer.Argument(..., help="Dataset whose manifest gets the case"),
    start: tuple[float, float, float] = typer.Option(..., "--start", help="Foot-level xyz"),
    goal: tuple[float, float, float] = typer.Option(..., "--goal", help="Foot-level xyz"),
    case_id: str = typer.Option(None, "--id", help="Case id; default manual_<n> or neg_<n>"),
    tags: str = typer.Option(None, "--tags", help="Comma-separated tags"),
    weight: float = typer.Option(1.0, "--weight"),
    snap_max: float = typer.Option(1.0, "--snap-max", help="Max snap distance to surface (m)"),
    expect_fail: bool = typer.Option(
        False, "--expect-fail", help="Certified-infeasible pair; the planner must refuse"
    ),
) -> None:
    """Append a curated case, with endpoints snapped to the final surface."""
    suite, manifest, surface, _ = _load_for_curation(dataset)
    default_tags = "manual,negative" if expect_fail else "manual"
    _append_case(
        suite,
        manifest,
        surface,
        start,
        goal,
        case_id,
        [t.strip() for t in (tags or default_tags).split(",") if t.strip()],
        weight,
        snap_max,
        expect_fail,
    )


@app.command("pick-case")
def pick_case(
    dataset: str = typer.Argument(..., help="Dataset whose manifest gets the cases"),
    weight: float = typer.Option(1.0, "--weight"),
    snap_max: float = typer.Option(1.0, "--snap-max", help="Max snap distance to surface (m)"),
) -> None:
    """Pick cases by shift+clicking the map in a browser viewer.

    Serves the final map and walked path with viser. Shift+click picks
    start/goal pairs. The side panel tags negatives, undoes picks, and saves
    pairs to the manifest, snapped like add-case.
    """
    # Lazy: picker/viz pull in viser and matplotlib, only needed for pick-case.
    from dimos.navigation.nav_3d.evaluator.picker import pick_cases
    from dimos.navigation.nav_3d.evaluator.viz import turbo_by_height

    suite, manifest, surface, cfg = _load_for_curation(dataset)
    final = load_or_build_final_map(resolve_named_path(dataset, ".db"), suite, cfg)
    trajectory = load_trajectory(resolve_named_path(dataset, ".db"), suite.odom_stream)
    foot = trajectory.positions - np.array([0.0, 0.0, cfg.robot_height], dtype=np.float32)

    def full_tags(negative: bool, extra: list[str]) -> list[str]:
        tags = ["manual"] + (["negative"] if negative else [])
        return tags + [t for t in extra if t not in tags]

    def save_pair(
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
        negative: bool,
        extra_tags: list[str],
        case_id: str | None,
    ) -> tuple[bool, str, str | None]:
        try:
            case = _append_case(
                suite,
                manifest,
                surface,
                start,
                goal,
                case_id,
                full_tags(negative, extra_tags),
                weight,
                snap_max,
                negative,
            )
        except typer.BadParameter as err:
            return False, str(err), None
        return True, f"saved {case.id} [{', '.join(case.tags)}]", case.id

    def update_case(
        saved_id: str, new_id: str, negative: bool, extra_tags: list[str]
    ) -> tuple[bool, str, str | None]:
        case = next((c for c in suite.cases if c.id == saved_id), None)
        if case is None:
            return False, f"case {saved_id!r} not found in manifest", None
        if new_id != saved_id and any(c.id == new_id for c in suite.cases):
            return False, f"case id {new_id!r} already exists", None
        case.id = new_id
        case.tags = full_tags(negative, extra_tags)
        case.expect_fail = negative
        save_suite(suite, manifest)
        return True, f"updated {case.id} [{', '.join(case.tags)}]", case.id

    pick_cases(
        dataset, final.occupied, turbo_by_height(final.occupied), foot, save_pair, update_case
    )
    print(f"\nrun with: python -m dimos.navigation.nav_3d.evaluator run --dataset {dataset}")


@app.command("list")
def list_cases() -> None:
    """Print every dataset's cases with endpoints, weights, and tags."""
    for suite in load_suites():
        print(f"{suite.dataset} ({suite.path.name if suite.path else '?'})")
        for case in suite.cases:
            tags = f" [{', '.join(case.tags)}]" if case.tags else ""
            print(
                f"  {case.id}: {tuple(round(v, 2) for v in case.start)} -> "
                f"{tuple(round(v, 2) for v in case.goal)} w={case.weight:g}{tags}"
            )


if __name__ == "__main__":
    app()
