# Copyright 2026 Dimensional Inc.
"""Single-frame point-cloud-grounded VQA commands."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import cast

import typer

from dimos.benchmark.vqa.generation.adapters import (
    EdgeTamObjectSegmenter,
    MoondreamObjectDetector,
)
from dimos.benchmark.vqa.generation.dataset import write_dataset_manifest, write_frame_record
from dimos.benchmark.vqa.generation.ground_truth_generator import VqaGroundTruthGenerator
from dimos.benchmark.vqa.generation.oracle import create_openai_oracle
from dimos.benchmark.vqa.generation.oracle_tools import LocalOracleToolRegistry
from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
from dimos.benchmark.vqa.generation.question_agent import (
    OpenAIFreeformQuestionAuthor,
    OpenAIQuestionAgent,
)
from dimos.benchmark.vqa.generation.recording import load_go2_frame
from dimos.benchmark.vqa.generation.specification import VqaGenerationSpecification
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
    _require_openai_for_question_author()
    _require_edgetam_cuda()
    typer.echo(f"Loading frame {frame_index} from {recording}")
    frame = load_go2_frame(str(resolve_named_path(recording, ".db")), frame_index)
    model = MoondreamVlModel()
    typer.echo("Loading private MoonDream model")
    model.start()
    question_agent = OpenAIQuestionAgent(OpenAIVlModel(model_name=QUESTION_MODEL))
    try:
        typer.echo(f"Proposing questions with {QUESTION_MODEL}")
        intents: list[QuestionIntent] | list[QuestionProposal] = (
            OpenAIFreeformQuestionAuthor(OpenAIVlModel(model_name=QUESTION_MODEL)).propose(
                frame.image
            )
            if question_mode == "agentic"
            else question_agent.propose(frame.image)
        )
        typer.echo(f"Grounding {len(intents)} questions for frame {frame_index}")
        primitives = FramePerceptionPrimitives(
            frame,
            detector := MoondreamObjectDetector(model),
            segmenter := EdgeTamObjectSegmenter(EdgeTAMImageSegmenter()),
            localizer=detector,
            point_segmenter=segmenter,
            config=GroundingConfig(
                min_mask_area_px=min_mask_area_px,
                min_foreground_points=min_foreground_points,
            ),
        )
        ground_truth = VqaGroundTruthGenerator(primitives)
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
        {
            "question_source": "agentic_image_author"
            if question_mode == "agentic"
            else "openai_image_agent",
            "question_model": QUESTION_MODEL,
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
    if generation.stop_index <= generation.start_index:
        raise typer.BadParameter("provide valid frame bounds")
    recording = generation.recording
    start_index = generation.start_index
    stop_index = generation.stop_index
    stride = generation.stride
    question_mode = generation.question_mode
    min_mask_area_px = generation.grounding.min_mask_area_px
    min_foreground_points = generation.grounding.min_foreground_points
    output = (
        Path(generation.output).expanduser()
        if generation.output is not None
        else STATE_DIR / "datasets" / "vqa" / f"{Path(recording).stem}-frames"
    )
    _require_openai_for_question_author()
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
            typer.echo(f"Frame {frame_index}: proposing questions with {QUESTION_MODEL}")
            intents: list[QuestionIntent] | list[QuestionProposal] = (
                OpenAIFreeformQuestionAuthor(OpenAIVlModel(model_name=QUESTION_MODEL)).propose(
                    frame.image
                )
                if question_mode == "agentic"
                else question_agent.propose(frame.image)
            )
            typer.echo(f"Frame {frame_index}: grounding {len(intents)} questions")
            primitives = FramePerceptionPrimitives(
                frame,
                detector,
                segmenter,
                localizer=detector,
                point_segmenter=segmenter,
                config=GroundingConfig(
                    min_mask_area_px=min_mask_area_px, min_foreground_points=min_foreground_points
                ),
            )
            ground_truth = VqaGroundTruthGenerator(primitives)
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
                {
                    "question_source": "agentic_image_author"
                    if question_mode == "agentic"
                    else "openai_image_agent",
                    "question_model": QUESTION_MODEL,
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
    _write_generation_run(output, generation, summary)
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
        result = oracle.answer(proposal, LocalOracleToolRegistry(ground_truth.primitives))
        results.append(result)
        if isinstance(result, AcceptedOracleResult):
            typer.echo(f"Agentic question {number}/{len(proposals)} accepted")
        else:
            typer.echo(f"Agentic question {number}/{len(proposals)} rejected: {result.reason}")
    return results


def _validate_question_mode(question_mode: str) -> None:
    if question_mode not in ("constrained", "agentic"):
        raise typer.BadParameter("question mode must be constrained or agentic")


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
) -> VqaGenerationSpecification:
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
            return VqaGenerationSpecification.model_validate_json(spec.read_bytes())
        except ValueError as exc:
            raise typer.BadParameter(f"invalid generation specification: {exc}") from exc
    if recording is None or stop_index is None:
        raise typer.BadParameter("--recording and --stop-index are required without --spec")
    try:
        return VqaGenerationSpecification(
            recording=recording,
            start_index=0 if start_index is None else start_index,
            stop_index=stop_index,
            stride=1 if stride is None else stride,
            question_mode="constrained" if question_mode is None else question_mode,
            grounding={
                "min_mask_area_px": 128 if min_mask_area_px is None else min_mask_area_px,
                "min_foreground_points": 3
                if min_foreground_points is None
                else min_foreground_points,
            },
            output=str(output) if output is not None else None,
        )
    except ValueError as exc:
        raise typer.BadParameter(f"invalid generation options: {exc}") from exc


def _write_generation_run(
    output: Path,
    generation: VqaGenerationSpecification,
    summary: dict[str, int],
) -> None:
    """Record the resolved request that produced one generated dataset."""
    payload = {
        "schema_version": "1.0",
        "generation": {
            **generation.model_dump(mode="json"),
            "output": str(output),
        },
        "models": {
            "question_author": QUESTION_MODEL,
            "oracle": ORACLE_MODEL if generation.question_mode == "agentic" else None,
        },
        "summary": summary,
    }
    (output / "run.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _require_openai_for_question_author() -> None:
    if not os.environ.get("OPENAI_API_KEY"):
        raise typer.BadParameter("OPENAI_API_KEY must be set for image-authored question modes")


def _require_edgetam_cuda() -> None:
    if default_local_model_device() != "cuda":
        raise typer.BadParameter(
            "VQA generation requires an installed PyTorch CUDA build that supports this GPU for EdgeTAM"
        )
