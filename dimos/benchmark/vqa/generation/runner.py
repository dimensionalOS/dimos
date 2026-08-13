# Copyright 2026 Dimensional Inc.
"""Execute resumable point-cloud-grounded VQA generation requests."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import json
import os
from pathlib import Path
from typing import TYPE_CHECKING, cast

from dimos.benchmark.vqa.generation.specification import VqaGenerationSpecification
from dimos.benchmark.vqa.models import (
    AcceptedOracleResult,
    GroundingConfig,
    GroundTruthResult,
    QuestionIntent,
    QuestionProposal,
    RejectedOracleResult,
)
from dimos.constants import STATE_DIR
from dimos.utils.data import resolve_named_path

if TYPE_CHECKING:
    from dimos.benchmark.vqa.generation.deterministic_question_answerer import (
        DeterministicQuestionAnswerer,
    )

QUESTION_MODEL = "gpt-4o-mini"
ORACLE_MODEL = "gpt-4o-mini"
ProgressSink = Callable[[str], None]


@dataclass(frozen=True)
class VqaGenerationResult:
    """Published dataset location and aggregate generation counts."""

    output: Path
    summary: dict[str, int]


def execute_generation(
    generation: VqaGenerationSpecification,
    *,
    progress: ProgressSink | None = None,
) -> VqaGenerationResult:
    """Execute one validated, resumable VQA generation request."""
    if generation.stop_index <= generation.start_index:
        raise ValueError("provide valid frame bounds")
    if not os.environ.get("OPENAI_API_KEY"):
        raise ValueError("OPENAI_API_KEY must be set for image-authored question modes")
    _require_edgetam_cuda()

    from dimos.benchmark.vqa.generation.adapters import (
        EdgeTamObjectSegmenter,
        MoondreamObjectDetector,
    )
    from dimos.benchmark.vqa.generation.dataset import (
        frame_audit_path,
        write_dataset_manifest,
        write_frame_record,
    )
    from dimos.benchmark.vqa.generation.deterministic_question_answerer import (
        DeterministicQuestionAnswerer,
    )
    from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
    from dimos.benchmark.vqa.generation.question_agent import (
        OpenAIFreeformQuestionAuthor,
        OpenAIQuestionAgent,
    )
    from dimos.benchmark.vqa.generation.recording import load_go2_frame
    from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
    from dimos.models.vl.moondream import MoondreamVlModel
    from dimos.models.vl.openai import OpenAIVlModel

    recording = generation.recording
    question_mode = generation.question_mode
    grounding = generation.grounding
    output = (
        Path(generation.output).expanduser()
        if generation.output is not None
        else STATE_DIR / "datasets" / "vqa" / f"{Path(recording).stem}-frames"
    )
    output.mkdir(parents=True, exist_ok=True)
    frame_indices = range(generation.start_index, generation.stop_index, generation.stride)
    _emit(
        progress,
        f"Generating {len(frame_indices)} sampled frames from {recording} into {output}",
    )
    model = MoondreamVlModel()
    _emit(progress, "Loading private MoonDream model")
    model.start()
    try:
        detector = MoondreamObjectDetector(model)
        segmenter = EdgeTamObjectSegmenter(EdgeTAMImageSegmenter())
        question_agent = OpenAIQuestionAgent(OpenAIVlModel(model_name=QUESTION_MODEL))
        for frame_number, frame_index in enumerate(frame_indices, start=1):
            frame_output = frame_audit_path(output, frame_index)
            if (frame_output / "frame.json").is_file():
                _validate_completed_frame(frame_output, generation, frame_index)
                _emit(
                    progress,
                    f"Skipping completed frame {frame_number}/{len(frame_indices)}: {frame_index}",
                )
                continue
            _emit(
                progress,
                f"Frame {frame_number}/{len(frame_indices)}: loading recording index {frame_index}",
            )
            frame = load_go2_frame(str(resolve_named_path(recording, ".db")), frame_index)
            _emit(progress, f"Frame {frame_index}: proposing questions with {QUESTION_MODEL}")
            intents: list[QuestionIntent] | list[QuestionProposal] = (
                OpenAIFreeformQuestionAuthor(
                    OpenAIVlModel(model_name=QUESTION_MODEL),
                    answerability_model=OpenAIVlModel(model_name=QUESTION_MODEL),
                ).propose(frame.image)
                if question_mode == "agentic"
                else question_agent.propose(frame.image)
            )
            _emit(progress, f"Frame {frame_index}: grounding {len(intents)} questions")
            primitives = FramePerceptionPrimitives(
                frame,
                detector,
                segmenter,
                localizer=detector,
                point_segmenter=segmenter,
                config=GroundingConfig(
                    min_mask_area_px=grounding.min_mask_area_px,
                    min_foreground_points=grounding.min_foreground_points,
                ),
            )
            answerer = DeterministicQuestionAnswerer(primitives)
            results: list[GroundTruthResult] | list[AcceptedOracleResult | RejectedOracleResult] = (
                _answer_agentic(
                    answerer,
                    cast("list[QuestionProposal]", intents),
                    progress,
                )
                if question_mode == "agentic"
                else _answer_intents(
                    answerer,
                    cast("list[QuestionIntent]", intents),
                    f"Frame {frame_index}",
                    progress,
                )
            )
            write_frame_record(
                output,
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
                    "grounding": grounding.model_dump(mode="json"),
                },
            )
            _emit(progress, f"Generated frame {frame_index}")
    finally:
        model.stop()
    summary = write_dataset_manifest(output)
    _write_generation_run(output, generation, summary)
    return VqaGenerationResult(output, summary)


def _answer_intents(
    answerer: DeterministicQuestionAnswerer,
    intents: list[QuestionIntent],
    label: str,
    progress: ProgressSink | None,
) -> list[GroundTruthResult]:
    results: list[GroundTruthResult] = []
    for number, intent in enumerate(intents, start=1):
        _emit(
            progress,
            f"{label}: grounding question {number}/{len(intents)}: "
            f"{intent.kind} ({intent.object_query})",
        )
        result = answerer.answer(intent)
        results.append(result)
        _emit(progress, f"{label}: question {number}/{len(intents)} {result.status}")
    return results


def _answer_agentic(
    answerer: DeterministicQuestionAnswerer,
    proposals: list[QuestionProposal],
    progress: ProgressSink | None,
) -> list[AcceptedOracleResult | RejectedOracleResult]:
    from dimos.benchmark.vqa.generation.oracle import create_openai_oracle
    from dimos.benchmark.vqa.generation.oracle_tools import LocalOracleToolRegistry

    oracle = create_openai_oracle(ORACLE_MODEL)
    results: list[AcceptedOracleResult | RejectedOracleResult] = []
    for number, proposal in enumerate(proposals, start=1):
        _emit(progress, f"Agentic question {number}/{len(proposals)}: {proposal.question}")
        result = oracle.answer(proposal, LocalOracleToolRegistry(answerer.primitives))
        results.append(result)
        status = (
            "accepted" if isinstance(result, AcceptedOracleResult) else f"rejected: {result.reason}"
        )
        _emit(progress, f"Agentic question {number}/{len(proposals)} {status}")
    return results


def _write_generation_run(
    output: Path,
    generation: VqaGenerationSpecification,
    summary: dict[str, int],
) -> None:
    """Record the resolved request that produced one generated dataset."""
    payload = {
        "schema_version": "1.0",
        "generation": {**generation.model_dump(mode="json"), "output": str(output)},
        "models": {
            "question_author": QUESTION_MODEL,
            "oracle": ORACLE_MODEL if generation.question_mode == "agentic" else None,
        },
        "summary": summary,
    }
    audit = output / "audit"
    audit.mkdir(parents=True, exist_ok=True)
    (audit / "run.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _validate_completed_frame(
    output: Path,
    generation: VqaGenerationSpecification,
    frame_index: int,
) -> None:
    """Reject completed frame records created by a different generation request."""
    try:
        payload = json.loads((output / "frame.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid completed frame marker: {output}") from exc
    expected_source = (
        "agentic_image_author" if generation.question_mode == "agentic" else "openai_image_agent"
    )
    grounding = payload.get("grounding")
    if (
        payload.get("recording") != generation.recording
        or payload.get("frame_index") != frame_index
        or payload.get("question_source") != expected_source
        or payload.get("question_model") != QUESTION_MODEL
        or payload.get("oracle_model")
        != (ORACLE_MODEL if generation.question_mode == "agentic" else None)
        or grounding != generation.grounding.model_dump(mode="json")
    ):
        raise ValueError(f"completed frame {output.name} was generated with different settings")


def _require_edgetam_cuda() -> None:
    from dimos.models.base import default_local_model_device

    if default_local_model_device() != "cuda":
        raise ValueError(
            "VQA generation requires an installed PyTorch CUDA build that supports this GPU for EdgeTAM"
        )


def _emit(progress: ProgressSink | None, message: str) -> None:
    if progress is not None:
        progress(message)
