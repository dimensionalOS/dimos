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

"""``dimos nav-bench``: freeze a suite, run it against a blueprint, score runs again, play episodes back."""

from __future__ import annotations

from collections import Counter
import json
from pathlib import Path
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
    from dimos.navigation.bench.runner import Progress, Report
    from dimos.navigation.bench.suite import Split

app = typer.Typer(help="Closed-loop navigation benchmark in the simulated world.")


def seeds_of(text: str) -> tuple[int, ...]:
    """Seeds from a list like ``1,3,5-8``."""
    seeds: list[int] = []
    for part in text.split(","):
        lo, _, hi = part.strip().partition("-")
        seeds.extend(range(int(lo), int(hi or lo) + 1))
    return tuple(seeds)


def override_flag(setting: str) -> str:
    """``Module.field=value`` as the ``--module.field=value`` flag ``dimos run`` takes."""
    key, sep, value = setting.partition("=")
    if not sep or not key:
        raise typer.BadParameter(f"expected Module.field=value, got {setting!r}")
    return f"--{key.lower().replace('_', '-')}={value}"


@app.command("freeze")
def freeze_command(
    out: Path = typer.Option(..., "--out", help="Where to write the manifest."),
    suite: str = typer.Option("v1", help="Suite name recorded in the manifest."),
    seeds: str = typer.Option("1-20", help="Scene seeds, e.g. 1,3,5-8."),
    sampling_seed: int = typer.Option(0, help="Seed for the case sampling."),
    stressor_samples: int = typer.Option(300, help="Candidate goals per stressor per scene."),
    samples_per_scene: int = typer.Option(60, help="Random start-goal pairs mined per scene."),
    cases_per_bin: int = typer.Option(3, help="Mined cases kept per difficulty bin."),
) -> None:
    """Generate, validate and bin the suite's cases into a manifest."""
    from dimos.navigation.bench.suite import FreezeConfig, freeze

    config = FreezeConfig(
        suite=suite,
        seeds=seeds_of(seeds),
        sampling_seed=sampling_seed,
        stressor_samples=stressor_samples,
        samples_per_scene=samples_per_scene,
        cases_per_bin=cases_per_bin,
    )
    with _Bar(suite) as bar:
        manifest = freeze(config, bar.tick)
    out.parent.mkdir(parents=True, exist_ok=True)
    manifest.save(out)
    tags = Counter(c.tag for c in manifest.cases)
    typer.echo(f"{len(manifest.cases)} cases, {len(manifest.rejections)} rejections -> {out}")
    for tag, count in sorted(tags.items()):
        typer.echo(f"  {tag}: {count}")


@app.command("run")
def run_command(
    suite: Path | None = typer.Option(
        None, "--suite", exists=True, help="A frozen manifest. Without one, v1 is frozen once."
    ),
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
    viewer: str = typer.Option(
        "none",
        help="Viewer for every episode, as dimos --viewer. One window each, so use --procs 1.",
    ),
    setting: list[str] = typer.Option(
        [], "--set", help="Module config override, Module.field=value. Repeatable."
    ),
) -> None:
    """Run every case of the split, several at a time, and score each recording."""
    from dimos.navigation.bench.runner import RESULTS_FILE, RUNS_DIR, RunConfig, run as run_suite

    if split not in ("dev", "held_out", "all"):
        raise typer.BadParameter("split must be dev, held_out or all")
    if suite is None:
        suite = RUNS_DIR / "v1.json"
        if not suite.exists():
            from dimos.navigation.bench.suite import FreezeConfig, freeze

            with _Bar("v1") as bar:
                manifest = freeze(FreezeConfig(), bar.tick)
            suite.parent.mkdir(parents=True, exist_ok=True)
            manifest.save(suite)
    config = RunConfig(
        suite=suite,
        blueprint=blueprint,
        split=None if split == "all" else cast("Split", split),
        cases=tuple(case),
        procs=procs,
        out_dir=out,
        overrides=tuple(override_flag(s) for s in setting),
        policy=policy,
        viewer=viewer,
    )
    with _Bar(suite.stem) as bar:
        run_dir = run_suite(config, _report(bar))
    typer.echo(f"run finished -> {run_dir}")
    _summary(run_dir / RESULTS_FILE)


@app.command("score")
def score_command(
    run_dir: Path = typer.Argument(..., exists=True, help="A run directory."),
) -> None:
    """Score every episode of a run again from its recording."""
    from dimos.navigation.bench.runner import rescore

    _summary(rescore(run_dir))


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


def _summary(results: Path) -> None:
    rows = [json.loads(line) for line in results.read_text().splitlines() if line.strip()]
    outcomes = Counter(str(row["outcome"] or row["error"]) for row in rows)
    typer.echo(f"{len(rows)} episodes")
    for outcome, count in outcomes.most_common():
        typer.echo(f"  {outcome}: {count}")
