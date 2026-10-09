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

"""``dimos sim-eval``: freeze a suite, run it against a blueprint, and score runs again."""

from __future__ import annotations

from collections import Counter
import json
from pathlib import Path

import typer

from dimos.navigation.sim_eval.runner import RESULTS_FILE, RunConfig, rescore, run as run_suite
from dimos.navigation.sim_eval.suite import FreezeConfig, freeze

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
    manifest = freeze(
        FreezeConfig(
            suite=suite,
            seeds=seeds_of(seeds),
            sampling_seed=sampling_seed,
            stressor_samples=stressor_samples,
            samples_per_scene=samples_per_scene,
            cases_per_bin=cases_per_bin,
        )
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    manifest.save(out)
    tags = Counter(c.tag for c in manifest.cases)
    typer.echo(f"{len(manifest.cases)} cases, {len(manifest.rejections)} rejections -> {out}")
    for tag, count in sorted(tags.items()):
        typer.echo(f"  {tag}: {count}")


@app.command("run")
def run_command(
    suite: Path = typer.Option(..., "--suite", exists=True, help="A frozen manifest."),
    blueprint: str = typer.Option("go2-sim-nav-episode", help="Blueprint with an EpisodeDriver."),
    split: str = typer.Option("dev", help="dev, held_out or all."),
    repeats: int = typer.Option(1, min=1),
    procs: int = typer.Option(4, min=1, help="Episodes running at once."),
    out: Path | None = typer.Option(None, "--out", help="Run directory, new."),
    policy: Path | None = typer.Option(None, help="Body policy file for the sim world."),
    setting: list[str] = typer.Option(
        [], "--set", help="Module config override, Module.field=value. Repeatable."
    ),
) -> None:
    """Run every case of the split, several at a time, and score each recording."""
    if split not in ("dev", "held_out", "all"):
        raise typer.BadParameter("split must be dev, held_out or all")
    run_dir = run_suite(
        RunConfig(
            suite=suite,
            blueprint=blueprint,
            split=None if split == "all" else split,  # type: ignore[arg-type]
            repeats=repeats,
            procs=procs,
            out_dir=out,
            overrides=tuple(override_flag(s) for s in setting),
            policy=policy,
        )
    )
    typer.echo(f"run finished -> {run_dir}")
    _summary(run_dir / RESULTS_FILE)


@app.command("score")
def score_command(
    run_dir: Path = typer.Argument(..., exists=True, help="A run directory."),
) -> None:
    """Score every episode of a run again from its recording."""
    _summary(rescore(run_dir))


def _summary(results: Path) -> None:
    rows = [json.loads(line) for line in results.read_text().splitlines() if line.strip()]
    outcomes = Counter(str(row["outcome"] or row["error"]) for row in rows)
    typer.echo(f"{len(rows)} episodes")
    for outcome, count in outcomes.most_common():
        typer.echo(f"  {outcome}: {count}")
