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

# Copyright 2026 Dimensional Inc.
"""Execute resumable point-cloud-grounded VQA generation requests."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import os
from pathlib import Path
from typing import TYPE_CHECKING, cast

from dimos.benchmark.vqa.contracts import (
    AcceptedOracleResult,
    GroundTruthResult,
    PrimitiveGroundingConfig,
    QuestionIntent,
    QuestionProposal,
    RejectedOracleResult,
)
from dimos.benchmark.vqa.generation.config import ORACLE_MODEL, QUESTION_MODEL, GenerationConfig
from dimos.constants import STATE_DIR
from dimos.utils.data import resolve_named_path

if TYPE_CHECKING:
    from dimos.benchmark.vqa.generation.agentic_answerer import AgenticAnswerer
    from dimos.benchmark.vqa.generation.deterministic_answerer import DeterministicAnswerer
    from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives

ProgressSink = Callable[[str], None]


@dataclass(frozen=True)
class GenerationResult:
    """Published dataset location and aggregate generation counts."""

    output: Path
    summary: dict[str, int]


def execute_generation(
    generation: GenerationConfig,
    *,
    progress: ProgressSink | None = None,
) -> GenerationResult:
    """Execute one validated, resumable VQA generation request."""
    if generation.stop_index <= generation.start_index:
        raise ValueError("provide valid frame bounds")
    if not os.environ.get("OPENAI_API_KEY"):
        raise ValueError("OPENAI_API_KEY must be set for image-authored question modes")
    _require_edgetam_cuda()

    from dimos.benchmark.vqa.generation.agentic_answerer import create_openai_oracle
    from dimos.benchmark.vqa.generation.dataset import GenerationDataset
    from dimos.benchmark.vqa.generation.deterministic_answerer import DeterministicAnswerer
    from dimos.benchmark.vqa.generation.model_adapters import (
        EdgeTamObjectSegmenter,
        MoondreamObjectDetector,
    )
    from dimos.benchmark.vqa.generation.preprocessing import Go2FramePreprocessor
    from dimos.benchmark.vqa.generation.primitives.frame import FramePerceptionPrimitives
    from dimos.benchmark.vqa.generation.question_authors import (
        AgenticQuestionAuthor,
        ConstrainedQuestionAuthor,
    )
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
    dataset = GenerationDataset(output)
    frame_indices = range(generation.start_index, generation.stop_index, generation.stride)
    _emit(
        progress,
        f"Generating {len(frame_indices)} sampled frames from {recording} into {output}",
    )
    model = MoondreamVlModel()
    frames = Go2FramePreprocessor(str(resolve_named_path(recording, ".db")))
    _emit(progress, "Loading private MoonDream model")
    model.start()
    try:
        frames.start()
        detector = MoondreamObjectDetector(model)
        segmenter = EdgeTamObjectSegmenter(EdgeTAMImageSegmenter())
        question_author = ConstrainedQuestionAuthor(OpenAIVlModel(model_name=QUESTION_MODEL))
        agentic_author = (
            AgenticQuestionAuthor(OpenAIVlModel(model_name=QUESTION_MODEL))
            if question_mode == "agentic"
            else None
        )
        oracle = create_openai_oracle(ORACLE_MODEL) if question_mode == "agentic" else None
        for frame_number, frame_index in enumerate(frame_indices, start=1):
            if dataset.completed_frame(frame_index, generation):
                _emit(
                    progress,
                    f"Skipping completed frame {frame_number}/{len(frame_indices)}: {frame_index}",
                )
                continue
            _emit(
                progress,
                f"Frame {frame_number}/{len(frame_indices)}: loading recording index {frame_index}",
            )
            frame = frames.load(frame_index)
            _emit(progress, f"Frame {frame_index}: proposing questions with {QUESTION_MODEL}")
            if question_mode == "agentic":
                if agentic_author is None:
                    raise RuntimeError("agentic question author was not initialized")
                intents: list[QuestionIntent] | list[QuestionProposal] = agentic_author.propose(
                    frame.image
                )
            else:
                intents = question_author.propose(frame.image)
            _emit(progress, f"Frame {frame_index}: grounding {len(intents)} questions")
            primitives = FramePerceptionPrimitives(
                frame,
                detector,
                segmenter,
                localizer=detector,
                point_segmenter=segmenter,
                config=PrimitiveGroundingConfig(
                    min_mask_area_px=grounding.min_mask_area_px,
                    min_foreground_points=grounding.min_foreground_points,
                ),
            )
            results: list[GroundTruthResult] | list[AcceptedOracleResult | RejectedOracleResult] = (
                _answer_agentic(
                    primitives,
                    cast("AgenticAnswerer", oracle),
                    cast("list[QuestionProposal]", intents),
                    progress,
                )
                if question_mode == "agentic"
                else _answer_intents(
                    DeterministicAnswerer(primitives),
                    cast("list[QuestionIntent]", intents),
                    f"Frame {frame_index}",
                    progress,
                )
            )
            dataset.write_frame(
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
                    "question_author_rejections": (
                        agentic_author.rejections
                        if question_mode == "agentic" and agentic_author is not None
                        else question_author.rejections
                    ),
                },
            )
            _emit(progress, f"Generated frame {frame_index}")
    finally:
        frames.stop()
        model.stop()
    summary = dataset.finalize(generation, frame_indices)
    return GenerationResult(output, summary)


def _answer_intents(
    answerer: DeterministicAnswerer,
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
        try:
            result = answerer.answer(intent)
        except Exception as exc:
            from dimos.benchmark.vqa.generation.families import rejected_result

            result = rejected_result(
                answerer.context,
                intent,
                (),
                (),
                f"question_error:{type(exc).__name__}:{exc}",
            )
        results.append(result)
        _emit(progress, f"{label}: question {number}/{len(intents)} {result.status}")
    return results


def _answer_agentic(
    primitives: FramePerceptionPrimitives,
    oracle: AgenticAnswerer,
    proposals: list[QuestionProposal],
    progress: ProgressSink | None,
) -> list[AcceptedOracleResult | RejectedOracleResult]:
    from dimos.benchmark.vqa.generation.agentic_tools import VqaPrimitiveToolRegistry

    results: list[AcceptedOracleResult | RejectedOracleResult] = []
    for number, proposal in enumerate(proposals, start=1):
        _emit(progress, f"Agentic question {number}/{len(proposals)}: {proposal.question}")
        try:
            result = oracle.answer(proposal, VqaPrimitiveToolRegistry(primitives))
        except Exception as exc:
            result = RejectedOracleResult(
                proposal,
                f"question_error:{type(exc).__name__}:{exc}",
                (),
                (),
            )
        results.append(result)
        status = (
            "accepted" if isinstance(result, AcceptedOracleResult) else f"rejected: {result.reason}"
        )
        _emit(progress, f"Agentic question {number}/{len(proposals)} {status}")
    return results


def _require_edgetam_cuda() -> None:
    import torch

    compatible = False
    if torch.cuda.is_available():
        try:
            major, minor = torch.cuda.get_device_capability()
            compatible = f"sm_{major}{minor}" in torch.cuda.get_arch_list()
        except RuntimeError:
            pass
    if not compatible:
        raise ValueError(
            "VQA generation requires an installed PyTorch CUDA build that supports this GPU for EdgeTAM"
        )


def _emit(progress: ProgressSink | None, message: str) -> None:
    if progress is not None:
        progress(message)
