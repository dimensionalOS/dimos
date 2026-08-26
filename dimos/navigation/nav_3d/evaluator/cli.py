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

"""Nav-3d evaluation CLI, mounted as the dimos nav-eval sub-command."""

from __future__ import annotations

import contextlib
import dataclasses
import json
import os
from pathlib import Path
import sqlite3
from time import perf_counter
from typing import TYPE_CHECKING

import typer

from dimos.navigation.nav_3d.evaluator.cases import (
    Suite,
    load_suites,
    manifest_path,
    save_suite,
)
from dimos.navigation.nav_3d.evaluator.config import EvalConfig
from dimos.navigation.nav_3d.evaluator.curation import CurationError, load_store
from dimos.navigation.nav_3d.evaluator.generate import (
    MIN_CASES,
    generate_cases,
    resolve_max_cases,
)
from dimos.navigation.nav_3d.evaluator.progress import RunProgress
from dimos.navigation.nav_3d.evaluator.runner import Report, evaluate
from dimos.utils.data import get_data_dir

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.curation import CaseStore
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
            # owns their names and defaults. Integer params reject floats, so
            # keep int-looking values int.
            try:
                parsed: float | int = int(value)
            except ValueError:
                parsed = float(value)
            cfg.planner[name.removeprefix("planner.")] = parsed
            continue
        if name not in fields:
            raise typer.BadParameter(f"unknown config field {name!r}")
        current = getattr(cfg, name)
        # bool is left out on purpose: bool("false") is True, so a boolean
        # override would silently mean the opposite of what was asked.
        if not isinstance(current, (int, float, str)) or isinstance(current, bool):
            raise typer.BadParameter(f"{name!r} cannot be set from the command line")
        setattr(cfg, name, type(current)(value))
    try:
        cfg.validate()
    except ValueError as err:
        raise typer.BadParameter(str(err)) from err
    return cfg


def _no_plan(outcome: PlanOutcome) -> bool:
    """No path came back and refusing was not the right answer."""
    return not outcome.planned and not outcome.success


def _clearance_cell(outcome: PlanOutcome) -> str:
    return "-" if outcome.min_clearance is None else f"{outcome.min_clearance:.2f}"


def _print_report(report: Report) -> None:
    header = (
        f"{'case':<28} {'dataset':<22} {'inc':^3} {'fin':^3} "
        f"{'inc_spl':>7} {'fin_spl':>7} "
        f"{'len':>6} {'ref':>6} {'clr':>6} {'ms':>7}"
    )
    print(header)
    print("-" * len(header))
    for d in report.datasets:
        for c in d.cases:
            no_path = _no_plan(c.online)
            inc_fail = "x" if no_path and not c.final_only else ""
            fin_fail = "x" if _no_plan(c.final) else ""
            inc_spl = "-" if c.final_only else f"{c.online.spl:.2f}"
            length = "x" if no_path else f"{c.online.length:.1f}"
            print(
                f"{c.id:<28} {c.dataset:<22} "
                f"{inc_fail:^3} {fin_fail:^3} "
                f"{inc_spl:>7} {c.final.spl:>7.2f} "
                f"{length:>6} {c.l_ref:>6.1f} "
                f"{_clearance_cell(c.online):>6} {c.online.plan_ms:>7.1f}"
            )
    print("-" * len(header))
    for d in report.datasets:
        print(f"{d.dataset}: {d.frames} frames")
    print(f"\n{'by tag':<12} {'inc_spl':>7} {'fin_spl':>7} {'n':>4}")
    for tag, s in report.by_tag.items():
        inc = f"{s.inc_score:.2f}" if s.n_online else "-"
        print(f"{tag:<12} {inc:>7} {s.fin_score:>7.2f} {s.n:>4}")
    print(
        f"\nscore {report.score:.3f} | "
        f"final {report.final_score:.3f} | "
        f"success inc {report.n_success}/{report.n_online} "
        f"fin {report.n_success_final}/{report.n_cases} | "
        f"outcomes {report.outcome_counts} | "
        f"plan p95 {report.plan_ms['p95']:.1f}ms | "
        f"map sync p95 {report.map_sync_ms['p95']:.1f}ms | "
        f"ingest p95 {report.map_update_ms['p95']:.1f}ms/frame"
    )
    inc_only = [
        f"{c.dataset}/{c.id}"
        for d in report.datasets
        for c in d.cases
        if c.online.success and not c.final.success
    ]
    if inc_only:
        print(f"\nincremental-only ({len(inc_only)}) — passed online, failed final:")
        print(f"  {', '.join(inc_only)}")
        print("  review with --rrd, confirm with: dimos nav-eval pick-case <dataset>")


@app.command()
def run(
    manifests: list[Path] | None = typer.Argument(
        None, help="Suite YAMLs; defaults to every manifest under case_manifests/"
    ),
    dataset: str | None = typer.Option(None, "--dataset", help="Only run suites for this dataset"),
    tag: list[str] | None = typer.Option(
        None, "--tag", help="Only run cases carrying every given tag, e.g. --tag stairs --tag up"
    ),
    json_out: Path | None = typer.Option(None, "--json", help="Write the full report as JSON"),
    rrd_out: Path | None = typer.Option(
        None, "--rrd", help="Write a rerun recording of every case"
    ),
    workers: int = typer.Option(
        os.cpu_count() or 1,
        "--workers",
        min=1,
        help="Datasets evaluated in parallel processes",
    ),
    set_: list[str] | None = typer.Option(
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
    # A case the planner cannot solve is the measurement here, and the report
    # names every one of them. Its per-case warning would only scroll away the
    # progress bar. Read once, when the planner starts its tracing subscriber.
    os.environ.setdefault("RUST_LOG", "warn,dimos_mls_planner=error")
    started = perf_counter()
    with RunProgress() as bars:
        report = evaluate(
            suites,
            cfg,
            workers=workers,
            keep_artifacts=rrd_out is not None,
            progress=bars.factory(),
            on_dataset=lambda name: print(f"  {name} scored ({perf_counter() - started:.0f}s)"),
        )
    _print_report(report)
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
        ..., help="Recording to ingest: a memory.db file or the directory holding one"
    ),
    name: str = typer.Option(..., "--name", help="Dataset name; becomes data/<name>.db"),
    lidar_stream: str = typer.Option("pointlio_lidar", "--lidar-stream"),
    odom_stream: str = typer.Option("pointlio_odometry", "--odom-stream"),
    cases: int = typer.Option(
        0, "--cases", help="Exact auto-generated case count; 0 scales with recording length"
    ),
    force: bool = typer.Option(False, "--force", help="Overwrite dataset and manifest"),
) -> None:
    """Register a recording as a dataset: copy, map, generate cases."""
    src = source / "memory.db" if source.is_dir() else source
    if not src.exists():
        raise typer.BadParameter(f"{src} does not exist")
    manifest = manifest_path(name)
    if manifest.exists() and not force:
        raise typer.BadParameter(f"{manifest} already exists; pass --force to regenerate")
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
    )
    trajectory = suite.trajectory()
    arcs = trajectory.arc_lengths()
    print(
        f"trajectory: {len(trajectory.positions)} poses, "
        f"{trajectory.ts[-1] - trajectory.ts[0]:.0f}s, {arcs[-1]:.1f}m walked, "
        f"z [{trajectory.positions[:, 2].min():.2f}, {trajectory.positions[:, 2].max():.2f}]"
    )
    max_cases = cases or None
    min_cases = cases or MIN_CASES
    suite.cases = generate_cases(trajectory, EvalConfig(), max_cases, min_cases)
    if not suite.cases:
        raise typer.Exit(code=1)
    floor = min(min_cases, resolve_max_cases(max_cases, float(arcs[-1])))
    if len(suite.cases) < floor:
        print(
            f"WARNING: only {len(suite.cases)} cases generated; the recording "
            "may be too short or too uniform for more"
        )
    path = save_suite(suite, manifest)
    print(f"\n{len(suite.cases)} cases -> {path}")
    for case in suite.cases:
        print(f"  {case.id}: [{', '.join(case.tags)}]")
    print(f"\nrun with: dimos nav-eval run --dataset {name}")


def _replay_occupancy(suite: Suite, cfg: EvalConfig) -> NDArray[np.float32]:
    """Feed the whole recording to the pipeline and take the map it built."""
    from dimos.navigation.nav_3d.evaluator.pipeline import PipelineIntrospection, make_pipeline
    from dimos.navigation.nav_3d.evaluator.progress import RunProgress, frame_progress

    pipeline = make_pipeline(cfg.pipeline, cfg)
    with RunProgress() as bars, frame_progress(bars.factory(), suite, "replay") as tick:
        for frame in suite.world_frames(cfg.align_tol):
            tick()
            pipeline.add_frame(frame.points, frame.origin, frame.ts)
    if not isinstance(pipeline, PipelineIntrospection):
        raise typer.BadParameter(f"pipeline {cfg.pipeline!r} does not expose its map")
    return pipeline.occupied()


def _open_store(dataset: str) -> CaseStore:
    try:
        return load_store(dataset)
    except CurationError as err:
        raise typer.BadParameter(str(err)) from err


@app.command("pick-case")
def pick_case(
    dataset: str = typer.Argument(..., help="Dataset whose manifest gets the cases"),
) -> None:
    """Pick and edit cases by shift+clicking the pipeline's map in a browser."""
    # Lazy: picker/viz pull in viser and matplotlib, only needed for pick-case.
    from dimos.navigation.nav_3d.evaluator.picker import pick_cases

    store = _open_store(dataset)
    cfg = EvalConfig()
    occupied = _replay_occupancy(store.suite, cfg)
    foot = store.suite.trajectory().foot(cfg.robot_height)
    pick_cases(store, foot, occupied, cfg.voxel_size)
    print(f"\nrun with: dimos nav-eval run --dataset {dataset}")


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
