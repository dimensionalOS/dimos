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

"""CLI commands for generating and evaluating standalone VQA datasets."""

from __future__ import annotations

from pathlib import Path

import typer

app = typer.Typer(help="Generate and evaluate standalone visual question-answering datasets.")


@app.command("generate")
def generate(
    dataset: str = typer.Argument(
        help="Memory dataset with color_image, camera_info, tf, and pointlio_lidar/lidar streams"
    ),
    image_index: int | None = typer.Option(None, min=0, help="Process one color_image index"),
    start: int | None = typer.Option(None, min=0, help="First color_image index in range mode"),
    stop: int | None = typer.Option(None, min=1, help="Exclusive color_image stop index"),
    stride: int | None = typer.Option(None, min=1, help="Frame stride in range mode"),
    quality_window: float | None = typer.Option(
        None,
        min=0,
        help=(
            "In range mode, keep the sharpest color_image per this many seconds "
            "via mem QualityWindow (cannot combine with --stride)"
        ),
    ),
    min_brightness: float | None = typer.Option(
        None,
        min=0,
        max=1,
        help="Drop frames at or below this brightness before QualityWindow (requires --quality-window)",
    ),
    output: Path | None = typer.Option(
        None,
        help="Destination directory; must be absent or empty",
    ),
    with_localize: bool = typer.Option(
        False,
        "--with-localize/--no-with-localize",
        help=(
            "Run Dan localize once over a padded window: ego memory clips + "
            "cross_view_same_object pairs from the same hits / first-sighting "
            "timestamps / color frames (off by default)"
        ),
    ),
    localize_query: list[str] = typer.Option(
        [],
        "--localize-query",
        help=(
            "Optional override object queries for localize (repeatable). "
            "When omitted with --with-localize, names are discovered from the recording"
        ),
    ),
    localize_pad: float = typer.Option(
        10.0,
        "--localize-pad",
        # Match GenerationRequest.localize_pad_s (gt=0); zero collapses single-frame windows.
        min=1e-9,
        help="Seconds to pad before/after selected frame timestamps for the localize window",
    ),
    localize_coverage_k: int | None = typer.Option(
        None,
        "--localize-coverage-k",
        min=0,
        help=(
            "With --with-localize, keep at most K published localize cases per frame "
            "(default 5), round-robining ego question types across frames. "
            "Use 0 to publish every answered localize case"
        ),
    ),
    min_views: int = typer.Option(
        3,
        min=1,
        help="With --with-localize: minimum distinct camera poses per cross-view identity instance",
    ),
    localize_evidence: Path | None = typer.Option(
        None,
        "--localize-evidence",
        help=(
            "Reuse a prior audit/localize_evidence.json (lightweight instance ids, "
            "names, coords, first_seen, sighting timestamps+bbox). Skips Dan when "
            "dataset/window match; still reads color frames from the recording"
        ),
    ),
) -> None:
    """Generate a standalone VQA dataset from selected color_image frames.

    ``--with-localize`` runs Dan localize once and adds ego + cross-view cases from
    that shared evidence (no separate cross-view-only pass). Pass
    ``--localize-evidence`` to skip Dan on a later run.
    """
    # Keep generation's optional model stack out of global CLI startup.
    from dimos.evals.vqa.generate import (
        GenerationRequest,
        generate_dataset,
    )

    try:
        request = GenerationRequest(
            dataset=dataset,
            output=output,
            image_index=image_index,
            start=start,
            stop=stop,
            stride=stride,
            quality_window=quality_window,
            min_brightness=min_brightness,
            with_localize=with_localize,
            localize_queries=tuple(localize_query),
            localize_pad_s=localize_pad,
            localize_coverage_k=localize_coverage_k,
            min_views=min_views,
            localize_evidence=localize_evidence,
        )
        result = generate_dataset(request)
    except (ValueError, IndexError, OSError) as exc:
        raise typer.BadParameter(str(exc)) from exc
    typer.echo(f"Generated {len(result.cases)} VQA case(s) in {result.output}")


@app.command("run")
def run(
    dataset: Path = typer.Argument(help="Generated standalone VQA dataset"),
    agent: str = typer.Option("dimos.evals.agents.question_answer", help="Dotted agent module"),
    model: str = typer.Option("", help="Override the agent's model"),
) -> None:
    """Evaluate a generated standalone VQA dataset."""
    # Keep evaluation implementation imports out of global CLI startup.
    from dimos.evals.cli import agent_kwargs, load_agent, run_provenance
    from dimos.evals.runner import run_suite
    from dimos.evals.vqa.suite import load_suite

    suite = load_suite(dataset)
    results = run_suite(
        suite,
        load_agent(agent, **agent_kwargs(model=model or None)),
        provenance=run_provenance(
            command=("evals", "vqa", "run"),
            agent=agent,
            model=model or None,
            suite=str(dataset),
        ),
    )
    typer.echo(f"Wrote {len(results)} evaluation result(s)")
