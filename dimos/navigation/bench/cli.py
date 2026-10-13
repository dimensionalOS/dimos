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

"""dimos nav-bench: freeze a suite, run it against a blueprint, score, report and play episodes back."""

from __future__ import annotations

from collections import Counter
from dataclasses import replace
import json
from pathlib import Path
import signal
from typing import TYPE_CHECKING, cast

from rich.progress import (
    BarColumn,
    MofNCompleteColumn,
    Progress as Bar,
    TextColumn,
    TimeElapsedColumn,
    TimeRemainingColumn,
)
import typer

if TYPE_CHECKING:
    from dimos.navigation.bench.runner import Keep, Progress, Report
    from dimos.navigation.bench.suite import Split

DEFAULT_SUITE = "v1"

app = typer.Typer(help="Closed-loop navigation benchmark in the simulated world.")


def seeds_of(text: str) -> tuple[int, ...]:
    """Seeds from a list like 1,3,5-8."""
    seeds: list[int] = []
    for part in text.split(","):
        lo, _, hi = part.strip().partition("-")
        seeds.extend(range(int(lo), int(hi or lo) + 1))
    return tuple(dict.fromkeys(seeds))


def override_flag(setting: str) -> str:
    """Module.field=value as the --module.field=value flag dimos run takes."""
    key, sep, value = setting.partition("=")
    if not sep or not key:
        raise typer.BadParameter(f"expected Module.field=value, got {setting!r}")
    return f"--{key.lower().replace('_', '-')}={value}"


@app.command("freeze")
def freeze_command(
    out: Path = typer.Option(..., "--out", help="Where to write the manifest."),
    suite: str = typer.Option(DEFAULT_SUITE, help="Suite name recorded in the manifest."),
    probe: str | None = typer.Option(None, help="Freeze this probe suite and not the mixed one."),
    seeds: str | None = typer.Option(None, help="Scene seeds, e.g. 1,3,5-8. Default: the suite's."),
    sampling_seed: int = typer.Option(0, help="Seed for the case sampling."),
    stressor_samples: int = typer.Option(300, help="Candidate goals per stressor per scene."),
    samples_per_scene: int = typer.Option(60, help="Random start-goal pairs mined per scene."),
    cases_per_bin: int = typer.Option(3, help="Mined cases kept per difficulty bin."),
) -> None:
    """Generate, validate and bin the suite's cases into a manifest."""
    from dimos.navigation.bench.suite import PROBES, FreezeConfig, freeze, freeze_probe

    if probe is not None:
        if probe not in PROBES:
            raise typer.BadParameter(f"unknown probe {probe!r}, known: {sorted(PROBES)}")
        spec = PROBES[probe]
        if seeds is not None:
            spec = replace(spec, seeds=seeds_of(seeds))
        manifest = freeze_probe(spec, sampling_seed)
    else:
        config = FreezeConfig(
            suite=suite,
            sampling_seed=sampling_seed,
            stressor_samples=stressor_samples,
            samples_per_scene=samples_per_scene,
            cases_per_bin=cases_per_bin,
        )
        if seeds is not None:
            config = replace(config, seeds=seeds_of(seeds))
        with _Bar(suite) as bar:
            manifest = freeze(config, bar.tick)
    out.parent.mkdir(parents=True, exist_ok=True)
    manifest.save(out)
    tags = Counter(c.tag for c in manifest.cases)
    typer.echo(f"{len(manifest.cases)} cases -> {out}")
    for tag, count in sorted(tags.items()):
        typer.echo(f"  {tag}: {count}")


@app.command("run")
def run_command(
    suite: list[Path] = typer.Option(
        [],
        "--suite",
        exists=True,
        help=f"A frozen manifest. Repeatable. With none and no --probes, {DEFAULT_SUITE} is frozen once.",
    ),
    probes: bool = typer.Option(False, "--probes", help="Freeze and run every probe as well."),
    blueprint: str = typer.Option("go2-sim-nav-episode", help="Blueprint with an EpisodeDriver."),
    split: str = typer.Option("dev", help="dev, held_out or all."),
    case: list[str] = typer.Option(
        [],
        "--case",
        help="Only these cases: a tag, a tag and scene like narrow_door-s3, or a full id.",
    ),
    procs: int = typer.Option(4, min=1, help="Episodes running at once."),
    out: Path | None = typer.Option(None, "--out", help="Run directory, new."),
    policy: Path | None = typer.Option(None, help="Body policy file for the sim world."),
    viewer: bool = typer.Option(
        False,
        "--viewer/--no-viewer",
        help="Open a Rerun viewer for every episode. One window each, so use --procs 1.",
    ),
    setting: list[str] = typer.Option(
        [], "--set", help="Module config override, Module.field=value. Repeatable."
    ),
    keep: str = typer.Option(
        "all", help="all, or failures to delete the recording of a success once it is scored."
    ),
) -> None:
    """Run every case of the split, several at a time, and score each recording.

    Several suites run one after another, each into its own directory under the run directory,
    with one report for all of them.
    """
    from dimos.navigation.bench.report import write
    from dimos.navigation.bench.runner import RUNS_DIR, RunConfig, new_run_dir, run as run_suite
    from dimos.navigation.bench.suite import PROBES, FreezeConfig, Manifest, freeze, freeze_probe

    if split not in ("dev", "held_out", "all"):
        raise typer.BadParameter("split must be dev, held_out or all")
    if keep not in ("all", "failures"):
        raise typer.BadParameter("keep must be all or failures")
    suites = list(suite)
    if not suites and not probes:
        default = RUNS_DIR / f"{DEFAULT_SUITE}.json"
        if not default.exists():
            with _Bar(DEFAULT_SUITE) as bar:
                manifest = freeze(FreezeConfig(suite=DEFAULT_SUITE), bar.tick)
            default.parent.mkdir(parents=True, exist_ok=True)
            manifest.save(default)
        suites.append(default)
    several = probes or len(suites) > 1
    if out is None:
        out = new_run_dir("suites" if several else Manifest.load(suites[0]).suite)
    if probes:
        out.mkdir(parents=True)
        for name, probe in PROBES.items():
            typer.echo(f"freezing {name}")
            freeze_probe(probe).save(out / f"{name}.json")
            suites.append(out / f"{name}.json")
    config = RunConfig(
        suite=suites[0],
        blueprint=blueprint,
        split=None if split == "all" else cast("Split", split),
        cases=tuple(case),
        procs=procs,
        out_dir=out,
        overrides=tuple(override_flag(s) for s in setting),
        policy=policy,
        viewer=viewer,
        keep=cast("Keep", keep),
    )
    signal.signal(signal.SIGTERM, _interrupt)
    run_dir = out
    try:
        for path in suites:
            run_dir = out / Manifest.load(path).suite if several else out
            with _Bar(path.stem) as bar:
                run_suite(replace(config, suite=path, out_dir=run_dir), _report(bar))
            typer.echo(f"run finished -> {run_dir}")
            _finish(run_dir)
    except KeyboardInterrupt:
        typer.echo(f"run interrupted -> {run_dir}")
        _finish(run_dir)
        if several:
            typer.echo(f"report -> {write(out)}")
        raise
    if several:
        typer.echo(f"report -> {write(out)}")


@app.command("score")
def score_command(
    run_dir: Path = typer.Argument(..., exists=True, help="A run directory."),
) -> None:
    """Score every episode of a run again from its recording."""
    from dimos.navigation.bench.runner import rescore

    _summary(rescore(run_dir))


@app.command("report")
def report_command(
    run_dir: Path = typer.Argument(..., exists=True, help="A run directory."),
) -> None:
    """Write report.html for a suite's run, or for every suite run under the directory."""
    from dimos.navigation.bench.report import write

    typer.echo(write(run_dir))


@app.command("replay")
def replay_command(
    episode: str = typer.Argument(
        ..., help="An episode directory, or a case id to find in the newest run."
    ),
    speed: float = typer.Option(1.0, min=0.01, help="Playback pace relative to the recording."),
    loop: bool = typer.Option(True, help="Start over at the end."),
    rerun: bool = typer.Option(True, help="Also open the episode's replay file in Rerun."),
) -> None:
    """Play the episode back: body motion in a MuJoCo viewer, the replay file in Rerun."""
    from dimos.navigation.bench.playback import find_episode, play

    play(find_episode(episode), speed=speed, loop=loop, rerun=rerun)


def _interrupt(*_: object) -> None:
    raise KeyboardInterrupt


class _Bar:
    """A bar over a long command's steps, with a status beside it and lines printed above it."""

    def __init__(self, name: str) -> None:
        self._bar = Bar(
            TextColumn("{task.description}"),
            BarColumn(),
            MofNCompleteColumn(),
            TimeElapsedColumn(),
            TimeRemainingColumn(),
            TextColumn("{task.fields[status]}"),
        )
        self._task = self._bar.add_task(name, total=None, status="")

    def __enter__(self) -> _Bar:
        self._bar.start()
        return self

    def __exit__(self, *_: object) -> None:
        self._bar.stop()

    def tick(self, done: int, total: int, status: str) -> None:
        self._bar.update(self._task, total=total, completed=done, status=status)

    def say(self, line: str) -> None:
        self._bar.console.print(line, highlight=False, markup=False)


def _report(bar: _Bar) -> Report:
    def report(progress: Progress) -> None:
        if progress.finished is not None:
            episode, verdict = progress.finished
            bar.say(f"{episode:44} {verdict}")
        tally = "  ".join(f"{k} {n}" for k, n in progress.outcomes.most_common())
        bar.tick(progress.done, progress.total, f"{tally}  running {len(progress.running)}")

    return report


def _finish(run_dir: Path) -> None:
    """Print the run's summary and write its report, if any episode finished."""
    from dimos.navigation.bench.report import write
    from dimos.navigation.bench.runner import RESULTS_FILE

    results = run_dir / RESULTS_FILE
    if not results.exists():
        return
    _summary(results)
    typer.echo(f"report -> {write(run_dir)}")


def _summary(results: Path) -> None:
    rows = [json.loads(line) for line in results.read_text().splitlines() if line.strip()]
    outcomes = Counter(str(row["outcome"] or row["error"]) for row in rows)
    typer.echo(f"{len(rows)} episodes")
    for outcome, count in outcomes.most_common():
        typer.echo(f"  {outcome}: {count}")
