# Copyright 2026 Dimensional Inc.
"""Single-frame point-cloud-grounded VQA commands."""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path

import cv2
import typer

from dimos.constants import STATE_DIR
from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.models.vl.openai import OpenAIVlModel
from dimos.perception.vqa.adapters import (
    EdgeTamObjectSegmenter,
    MoondreamObjectDetector,
    MoondreamQuestionAnswerer,
)
from dimos.perception.vqa.dataset import write_dataset_manifest, write_frame_record
from dimos.perception.vqa.ground_truth_agent import GroundTruthPerceptionAgent
from dimos.perception.vqa.models import GroundingConfig, QuestionIntent
from dimos.perception.vqa.pipeline import evaluate_ground_truth
from dimos.perception.vqa.question_agent import OpenAIQuestionAgent
from dimos.perception.vqa.recording import load_go2_frame
from dimos.utils.data import resolve_named_path

app = typer.Typer(help="Generate and evaluate point-cloud-grounded VQA examples")


@app.command("single-frame")
def single_frame(
    recording: str = typer.Option(..., "--recording"),
    frame_index: int = typer.Option(0, "--frame-index"),
    query: list[str] = typer.Option([], "--query"),
    propose_questions: bool = typer.Option(False, "--propose-questions"),
    question_model: str = typer.Option("gpt-4o-mini", "--question-model"),
    min_mask_area_px: int = typer.Option(128, "--min-mask-area-px"),
    min_foreground_points: int = typer.Option(3, "--min-foreground-points"),
    output: Path | None = typer.Option(None, "--output"),
) -> None:
    """Generate and evaluate questions for one Go2 recording frame."""
    output = output or (
        STATE_DIR / "datasets" / "vqa" / f"{Path(recording).stem}-frame-{frame_index:06d}"
    )
    if output.exists() or (not query and not propose_questions):
        raise typer.BadParameter("output must not already exist")
    frame = load_go2_frame(str(resolve_named_path(recording, ".db")), frame_index)
    model = MoondreamVlModel()
    model.start()
    question_agent = OpenAIQuestionAgent(OpenAIVlModel(model_name=question_model))
    try:
        intents = (
            question_agent.propose(frame.image)
            if propose_questions
            else [
                QuestionIntent(
                    kind=kind,
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
        ground_truth = GroundTruthPerceptionAgent(
            detector := MoondreamObjectDetector(model),
            segmenter := EdgeTamObjectSegmenter(EdgeTAMImageSegmenter()),
            localizer=detector,
            point_segmenter=segmenter,
            config=GroundingConfig(
                min_mask_area_px=min_mask_area_px,
                min_foreground_points=min_foreground_points,
            ),
        )
        results = [ground_truth.answer(frame, intent) for intent in intents]
        examples = [result.question for result in results if result.status == "answered"]
        evaluations = evaluate_ground_truth(frame, examples, MoondreamQuestionAnswerer(model))
    finally:
        model.stop()
    output.mkdir(parents=True)
    image_path = output / "image.jpg"
    if not cv2.imwrite(str(image_path), frame.image.data):
        raise RuntimeError(f"failed to write {image_path}")
    original_image_path = output / "original_image.jpg"
    if frame.original_image is not None and not cv2.imwrite(
        str(original_image_path), frame.original_image.data
    ):
        raise RuntimeError(f"failed to write {original_image_path}")
    overlay_path = output / "grounding_overlay.jpg"
    ground_truth.write_overlay(frame, str(overlay_path))
    (output / "frame.json").write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "frame_id": frame.id,
                "recording": recording,
                "frame_index": frame_index,
                "image": image_path.name,
                "original_image": original_image_path.name
                if frame.original_image is not None
                else None,
                "grounding_overlay": overlay_path.name,
                "question_count": len(intents),
                "accepted_question_count": len(examples),
                "rejected_question_count": len(results) - len(examples),
                "question_source": "image_agent" if propose_questions else "explicit_queries",
                "question_model": question_model if propose_questions else None,
                "grounding": {
                    "min_mask_area_px": min_mask_area_px,
                    "min_foreground_points": min_foreground_points,
                },
            },
            indent=2,
        )
        + "\n"
    )
    (output / "intents.json").write_text(
        json.dumps([asdict(item) for item in intents], indent=2) + "\n"
    )
    (output / "examples.json").write_text(
        json.dumps([asdict(item) for item in examples], indent=2) + "\n"
    )
    (output / "ground_truth.json").write_text(
        json.dumps([asdict(item) for item in results], indent=2) + "\n"
    )
    (output / "evaluations.json").write_text(
        json.dumps([asdict(item) for item in evaluations], indent=2) + "\n"
    )
    typer.echo(f"Wrote {len(examples)} examples and {len(evaluations)} evaluations to {output}")


@app.command("generate")
def generate(
    recording: str = typer.Option(..., "--recording"),
    start_index: int = typer.Option(0, "--start-index"),
    stop_index: int = typer.Option(..., "--stop-index"),
    stride: int = typer.Option(1, "--stride"),
    query: list[str] = typer.Option([], "--query"),
    propose_questions: bool = typer.Option(False, "--propose-questions"),
    question_model: str = typer.Option("gpt-4o-mini", "--question-model"),
    min_mask_area_px: int = typer.Option(128, "--min-mask-area-px"),
    min_foreground_points: int = typer.Option(3, "--min-foreground-points"),
    output: Path | None = typer.Option(None, "--output"),
) -> None:
    """Generate a resumable VQA dataset from sampled Go2 recording frames."""
    if (
        start_index < 0
        or stop_index <= start_index
        or stride < 1
        or (not query and not propose_questions)
    ):
        raise typer.BadParameter(
            "provide valid frame bounds and either --query or --propose-questions"
        )
    output = output or (STATE_DIR / "datasets" / "vqa" / f"{Path(recording).stem}-frames")
    output.mkdir(parents=True, exist_ok=True)
    model = MoondreamVlModel()
    model.start()
    try:
        detector = MoondreamObjectDetector(model)
        segmenter = EdgeTamObjectSegmenter(EdgeTAMImageSegmenter())
        question_agent = OpenAIQuestionAgent(OpenAIVlModel(model_name=question_model))
        for frame_index in range(start_index, stop_index, stride):
            frame_output = output / f"frame-{frame_index:06d}"
            if (frame_output / "frame.json").is_file():
                continue
            frame = load_go2_frame(str(resolve_named_path(recording, ".db")), frame_index)
            intents = (
                question_agent.propose(frame.image)
                if propose_questions
                else [
                    QuestionIntent(
                        kind=kind,
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
            ground_truth = GroundTruthPerceptionAgent(
                detector,
                segmenter,
                localizer=detector,
                point_segmenter=segmenter,
                config=GroundingConfig(
                    min_mask_area_px=min_mask_area_px, min_foreground_points=min_foreground_points
                ),
            )
            results = [ground_truth.answer(frame, intent) for intent in intents]
            examples = [result.question for result in results if result.status == "answered"]
            evaluations = evaluate_ground_truth(frame, examples, MoondreamQuestionAnswerer(model))
            write_frame_record(
                frame_output,
                frame,
                recording,
                frame_index,
                intents,
                results,
                evaluations,
                ground_truth,
                {
                    "question_source": "openai_image_agent"
                    if propose_questions
                    else "explicit_queries",
                    "question_model": question_model if propose_questions else None,
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
