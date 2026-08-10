# Copyright 2026 Dimensional Inc.
"""Single-frame point-cloud-grounded VQA commands."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, cast

import typer

from dimos.benchmark.vqa.generation.adapters import (
    EdgeTamObjectSegmenter,
    MoondreamObjectDetector,
)
from dimos.benchmark.vqa.generation.dataset import write_dataset_manifest, write_frame_record
from dimos.benchmark.vqa.generation.ground_truth_generator import VqaGroundTruthGenerator
from dimos.benchmark.vqa.generation.oracle import create_openai_oracle
from dimos.benchmark.vqa.generation.oracle_tools import LocalOracleToolRegistry
from dimos.benchmark.vqa.generation.question_agent import (
    OpenAIFreeformQuestionAuthor,
    OpenAIQuestionAgent,
)
from dimos.benchmark.vqa.generation.recording import load_go2_frame
from dimos.benchmark.vqa.models import (
    AcceptedOracleResult,
    CalibratedFrame,
    GroundingConfig,
    GroundTruthResult,
    QuestionIntent,
    QuestionProposal,
    RejectedOracleResult,
)
from dimos.constants import STATE_DIR
from dimos.models.base import default_local_model_device
from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.models.vl.openai import OpenAIVlModel
from dimos.utils.data import resolve_named_path

app = typer.Typer(help="Generate point-cloud-grounded VQA benchmark examples")
QUESTION_MODEL = "gpt-4o-mini"
ORACLE_MODEL = "gpt-4o-mini"


@app.command("single-frame")
def single_frame(
    recording: str = typer.Option(..., "--recording"),
    frame_index: int = typer.Option(0, "--frame-index"),
    query: list[str] = typer.Option([], "--query"),
    question_mode: str = typer.Option("constrained", "--question-mode"),
    min_mask_area_px: int = typer.Option(128, "--min-mask-area-px"),
    min_foreground_points: int = typer.Option(3, "--min-foreground-points"),
    output: Path | None = typer.Option(None, "--output"),
) -> None:
    """Generate private-grounded questions for one Go2 recording frame."""
    output = output or (
        STATE_DIR / "datasets" / "vqa" / f"{Path(recording).stem}-frame-{frame_index:06d}"
    )
    if output.exists():
        raise typer.BadParameter("output must not already exist")
    _validate_question_mode(question_mode)
    uses_image_author = _uses_image_question_author(question_mode, query)
    _require_openai_for_question_author(uses_image_author)
    _require_edgetam_cuda()
    typer.echo(f"Loading frame {frame_index} from {recording}")
    frame = load_go2_frame(str(resolve_named_path(recording, ".db")), frame_index)
    model = MoondreamVlModel()
    typer.echo("Loading private MoonDream model")
    model.start()
    question_agent = OpenAIQuestionAgent(OpenAIVlModel(model_name=QUESTION_MODEL))
    try:
        if uses_image_author:
            typer.echo(f"Proposing questions with {QUESTION_MODEL}")
        intents: list[QuestionIntent] | list[QuestionProposal] = (
            OpenAIFreeformQuestionAuthor(OpenAIVlModel(model_name=QUESTION_MODEL)).propose(
                frame.image
            )
            if question_mode == "agentic"
            else question_agent.propose(frame.image)
            if uses_image_author
            else [
                QuestionIntent(
                    kind=cast("Any", kind),
                    object_query=item,
                    threshold_m=3.0 if kind == "within_distance" else None,
                )
                for item in query
                for kind in (
                    "presence",
                    "horizontal_direction",
                    "within_distance",
                    "compare_nearest_by_side",
                )
            ]
        )
        typer.echo(f"Grounding {len(intents)} questions for frame {frame_index}")
        ground_truth = VqaGroundTruthGenerator(
            detector := MoondreamObjectDetector(model),
            segmenter := EdgeTamObjectSegmenter(EdgeTAMImageSegmenter()),
            localizer=detector,
            point_segmenter=segmenter,
            config=GroundingConfig(
                min_mask_area_px=min_mask_area_px,
                min_foreground_points=min_foreground_points,
            ),
        )
        results: list[GroundTruthResult] | list[AcceptedOracleResult | RejectedOracleResult] = (
            _answer_agentic(ground_truth, frame, cast("list[QuestionProposal]", intents))
            if question_mode == "agentic"
            else _answer_intents(
                ground_truth, frame, cast("list[QuestionIntent]", intents), f"Frame {frame_index}"
            )
        )
        examples = [
            result
            for result in results
            if isinstance(result, AcceptedOracleResult)
            or (isinstance(result, GroundTruthResult) and result.status == "answered")
        ]
    finally:
        model.stop()
    write_frame_record(
        output,
        frame,
        recording,
        frame_index,
        cast("list[QuestionIntent | QuestionProposal]", intents),
        cast("list[GroundTruthResult | AcceptedOracleResult | RejectedOracleResult]", results),
        ground_truth,
        {
            "question_source": "agentic_image_author"
            if question_mode == "agentic"
            else "openai_image_agent"
            if uses_image_author
            else "explicit_queries",
            "question_model": QUESTION_MODEL if uses_image_author else None,
            "oracle_model": ORACLE_MODEL if question_mode == "agentic" else None,
            "grounding": {
                "min_mask_area_px": min_mask_area_px,
                "min_foreground_points": min_foreground_points,
            },
        },
    )
    typer.echo(f"Wrote {len(examples)} examples to {output}")


@app.command("generate")
def generate(
    recording: str = typer.Option(..., "--recording"),
    start_index: int = typer.Option(0, "--start-index"),
    stop_index: int = typer.Option(..., "--stop-index"),
    stride: int = typer.Option(1, "--stride"),
    query: list[str] = typer.Option([], "--query"),
    question_mode: str = typer.Option("constrained", "--question-mode"),
    min_mask_area_px: int = typer.Option(128, "--min-mask-area-px"),
    min_foreground_points: int = typer.Option(3, "--min-foreground-points"),
    output: Path | None = typer.Option(None, "--output"),
) -> None:
    """Generate a resumable VQA dataset from sampled Go2 recording frames."""
    if start_index < 0 or stop_index <= start_index or stride < 1:
        raise typer.BadParameter("provide valid frame bounds")
    output = output or (STATE_DIR / "datasets" / "vqa" / f"{Path(recording).stem}-frames")
    _validate_question_mode(question_mode)
    uses_image_author = _uses_image_question_author(question_mode, query)
    _require_openai_for_question_author(uses_image_author)
    output.mkdir(parents=True, exist_ok=True)
    _require_edgetam_cuda()
    frame_indices = range(start_index, stop_index, stride)
    typer.echo(f"Generating {len(frame_indices)} sampled frames from {recording} into {output}")
    model = MoondreamVlModel()
    typer.echo("Loading private MoonDream model")
    model.start()
    try:
        detector = MoondreamObjectDetector(model)
        segmenter = EdgeTamObjectSegmenter(EdgeTAMImageSegmenter())
        question_agent = OpenAIQuestionAgent(OpenAIVlModel(model_name=QUESTION_MODEL))
        for frame_number, frame_index in enumerate(frame_indices, start=1):
            frame_output = output / f"frame-{frame_index:06d}"
            if (frame_output / "frame.json").is_file():
                typer.echo(
                    f"Skipping completed frame {frame_number}/{len(frame_indices)}: {frame_index}"
                )
                continue
            typer.echo(
                f"Frame {frame_number}/{len(frame_indices)}: loading recording index {frame_index}"
            )
            frame = load_go2_frame(str(resolve_named_path(recording, ".db")), frame_index)
            if uses_image_author:
                typer.echo(f"Frame {frame_index}: proposing questions with {QUESTION_MODEL}")
            intents: list[QuestionIntent] | list[QuestionProposal] = (
                OpenAIFreeformQuestionAuthor(OpenAIVlModel(model_name=QUESTION_MODEL)).propose(
                    frame.image
                )
                if question_mode == "agentic"
                else question_agent.propose(frame.image)
                if uses_image_author
                else [
                    QuestionIntent(
                        kind=cast("Any", kind),
                        object_query=item,
                        threshold_m=3.0 if kind == "within_distance" else None,
                    )
                    for item in query
                    for kind in (
                        "presence",
                        "horizontal_direction",
                        "within_distance",
                        "compare_nearest_by_side",
                    )
                ]
            )
            typer.echo(f"Frame {frame_index}: grounding {len(intents)} questions")
            ground_truth = VqaGroundTruthGenerator(
                detector,
                segmenter,
                localizer=detector,
                point_segmenter=segmenter,
                config=GroundingConfig(
                    min_mask_area_px=min_mask_area_px, min_foreground_points=min_foreground_points
                ),
            )
            results: list[GroundTruthResult] | list[AcceptedOracleResult | RejectedOracleResult] = (
                _answer_agentic(ground_truth, frame, cast("list[QuestionProposal]", intents))
                if question_mode == "agentic"
                else _answer_intents(
                    ground_truth,
                    frame,
                    cast("list[QuestionIntent]", intents),
                    f"Frame {frame_index}",
                )
            )
            write_frame_record(
                frame_output,
                frame,
                recording,
                frame_index,
                cast("list[QuestionIntent | QuestionProposal]", intents),
                cast(
                    "list[GroundTruthResult | AcceptedOracleResult | RejectedOracleResult]", results
                ),
                ground_truth,
                {
                    "question_source": "agentic_image_author"
                    if question_mode == "agentic"
                    else "openai_image_agent"
                    if uses_image_author
                    else "explicit_queries",
                    "question_model": QUESTION_MODEL if uses_image_author else None,
                    "oracle_model": ORACLE_MODEL if question_mode == "agentic" else None,
                    "grounding": {
                        "min_mask_area_px": min_mask_area_px,
                        "min_foreground_points": min_foreground_points,
                    },
                },
            )
            typer.echo(f"Generated {frame_output}")
    finally:
        model.stop()
    summary = write_dataset_manifest(output)
    typer.echo(f"Dataset manifest: {summary}")


def _answer_intents(
    ground_truth: VqaGroundTruthGenerator,
    frame: CalibratedFrame,
    intents: list[QuestionIntent],
    label: str,
) -> list[GroundTruthResult]:
    results: list[GroundTruthResult] = []
    for number, intent in enumerate(intents, start=1):
        typer.echo(
            f"{label}: grounding question {number}/{len(intents)}: "
            f"{intent.kind} ({intent.object_query})"
        )
        result = ground_truth.answer(frame, intent)
        results.append(result)
        typer.echo(f"{label}: question {number}/{len(intents)} {result.status}")
    return results


def _answer_agentic(
    ground_truth: VqaGroundTruthGenerator,
    frame: CalibratedFrame,
    proposals: list[QuestionProposal],
) -> list[AcceptedOracleResult | RejectedOracleResult]:
    oracle = create_openai_oracle(ORACLE_MODEL)
    results: list[AcceptedOracleResult | RejectedOracleResult] = []
    for number, proposal in enumerate(proposals, start=1):
        typer.echo(f"Agentic question {number}/{len(proposals)}: {proposal.question}")
        result = oracle.answer(proposal, LocalOracleToolRegistry(frame, ground_truth))
        results.append(result)
        if isinstance(result, AcceptedOracleResult):
            typer.echo(f"Agentic question {number}/{len(proposals)} accepted")
        else:
            typer.echo(f"Agentic question {number}/{len(proposals)} rejected: {result.reason}")
    return results


def _validate_question_mode(question_mode: str) -> None:
    if question_mode not in ("constrained", "agentic"):
        raise typer.BadParameter("question mode must be constrained or agentic")


def _require_openai_for_question_author(uses_image_author: bool) -> None:
    if uses_image_author and not os.environ.get("OPENAI_API_KEY"):
        raise typer.BadParameter("OPENAI_API_KEY must be set for image-authored question modes")


def _uses_image_question_author(question_mode: str, query: list[str]) -> bool:
    """Choose authored questions unless constrained mode has explicit queries."""
    return question_mode == "agentic" or not query


def _require_edgetam_cuda() -> None:
    if default_local_model_device() != "cuda":
        raise typer.BadParameter(
            "VQA generation requires an installed PyTorch CUDA build that supports this GPU for EdgeTAM"
        )
