# Copyright 2026 Dimensional Inc.
"""Point-cloud-grounded VQA generation commands."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal, cast

import typer

from dimos.benchmark.vqa.generation.config import (
    GenerationConfig,
    GroundingConfig,
)

app = typer.Typer(help="Generate point-cloud-grounded VQA benchmark examples")


def execute_generation(*args: Any, **kwargs: Any) -> Any:
    """Import and dispatch the generation runtime only when generation executes."""
    from dimos.benchmark.vqa.generation.runner import execute_generation as execute

    return execute(*args, **kwargs)


@app.callback()
def vqa() -> None:
    """Generate point-cloud-grounded VQA benchmark examples."""


@app.command("generate")
def generate(
    recording: str | None = typer.Option(None, "--recording"),
    start_index: int | None = typer.Option(None, "--start-index"),
    stop_index: int | None = typer.Option(None, "--stop-index"),
    stride: int | None = typer.Option(None, "--stride"),
    question_mode: str | None = typer.Option(None, "--question-mode"),
    min_mask_area_px: int | None = typer.Option(None, "--min-mask-area-px"),
    min_foreground_points: int | None = typer.Option(None, "--min-foreground-points"),
    output: Path | None = typer.Option(None, "--output"),
    spec: Path | None = typer.Option(None, "--spec", exists=True, dir_okay=False, readable=True),
) -> None:
    """Generate a resumable VQA dataset from sampled Go2 recording frames."""
    generation = _resolve_generation_spec(
        spec,
        recording,
        start_index,
        stop_index,
        stride,
        question_mode,
        min_mask_area_px,
        min_foreground_points,
        output,
    )
    try:
        result = execute_generation(generation, progress=typer.echo)
    except ValueError as exc:
        raise typer.BadParameter(str(exc)) from exc
    typer.echo(f"Dataset manifest: {result.summary}")


def _resolve_generation_spec(
    spec: Path | None,
    recording: str | None,
    start_index: int | None,
    stop_index: int | None,
    stride: int | None,
    question_mode: str | None,
    min_mask_area_px: int | None,
    min_foreground_points: int | None,
    output: Path | None,
) -> GenerationConfig:
    """Load a JSON generation specification or resolve the explicit CLI alternatives."""
    values = (
        recording,
        start_index,
        stop_index,
        stride,
        question_mode,
        min_mask_area_px,
        min_foreground_points,
        output,
    )
    if spec is not None:
        if any(value is not None for value in values):
            raise typer.BadParameter("--spec cannot be combined with generation options")
        try:
            return GenerationConfig.model_validate_json(spec.read_bytes())
        except ValueError as exc:
            raise typer.BadParameter(f"invalid generation specification: {exc}") from exc
    if recording is None or stop_index is None:
        raise typer.BadParameter("--recording and --stop-index are required without --spec")
    if question_mode is not None and question_mode not in ("constrained", "agentic"):
        raise typer.BadParameter("question mode must be constrained or agentic")
    resolved_question_mode = cast(
        "Literal['constrained', 'agentic']",
        "constrained" if question_mode is None else question_mode,
    )
    try:
        return GenerationConfig(
            recording=recording,
            start_index=0 if start_index is None else start_index,
            stop_index=stop_index,
            stride=1 if stride is None else stride,
            question_mode=resolved_question_mode,
            grounding=GroundingConfig(
                min_mask_area_px=128 if min_mask_area_px is None else min_mask_area_px,
                min_foreground_points=3 if min_foreground_points is None else min_foreground_points,
            ),
            output=str(output) if output is not None else None,
        )
    except ValueError as exc:
        raise typer.BadParameter(f"invalid generation options: {exc}") from exc
