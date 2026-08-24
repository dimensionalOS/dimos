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

"""High-level contracts for standalone VQA dataset generation."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
import json
from pathlib import Path
import re
import tempfile
from typing import TYPE_CHECKING

from pydantic import BaseModel, ConfigDict, Field, model_validator

from dimos.constants import STATE_DIR
from dimos.evals.vqa.author import OpenAIQuestionAuthor, QuestionAuthor
from dimos.evals.vqa.contracts import (
    FamilyAnswer,
    FamilySpec,
    InsufficientEvidenceError,
    InvalidQuestionProposalError,
    NonEmptyString,
    ObjectMaskEstimator,
    ObjectRangeEstimator,
    QuestionProposal,
)
from dimos.evals.vqa.families import (
    AVAILABLE_FAMILIES,
    answer_question,
)
from dimos.evals.vqa.pointcloud_frame import (
    PointCloudFrame,
    PointCloudFrameLoader,
    PointCloudFrameUnavailableError,
)
from dimos.evals.vqa.primitives.edge_tam import EdgeTAMObjectMaskPipeline
from dimos.evals.vqa.primitives.range import LidarRangeEstimator
from dimos.msgs.sensor_msgs.Image import Image

if TYPE_CHECKING:
    from dimos.models.vl.base import VlModel

_UNORDERED_FAMILIES = frozenset(
    family.name for family in AVAILABLE_FAMILIES if not family.object_order_matters
)


class GenerationRequest(BaseModel):
    """Single-frame or range selection from a recorded Memory dataset."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    dataset: NonEmptyString
    output: Path | None = None
    image_index: int | None = Field(default=None, ge=0)
    start: int | None = Field(default=None, ge=0)
    stop: int | None = Field(default=None, gt=0)
    stride: int | None = Field(default=None, ge=1)

    @model_validator(mode="after")
    def valid_selection(self) -> GenerationRequest:
        if self.image_index is not None:
            if self.start is not None or self.stop is not None or self.stride is not None:
                raise ValueError("image_index cannot be combined with start, stop, or stride")
            return self
        if self.start is None or self.stop is None:
            raise ValueError("provide image_index or both start and stop")
        if self.stop <= self.start:
            raise ValueError("stop must be greater than start")
        return self

    def frame_indices(self) -> range:
        if self.image_index is not None:
            return range(self.image_index, self.image_index + 1)
        assert self.start is not None and self.stop is not None
        return range(self.start, self.stop, self.stride or 1)

    def output_directory(self) -> Path:
        if self.output is not None:
            return self.output.expanduser()
        return STATE_DIR / "datasets" / "vqa" / f"{Path(self.dataset).stem}-frames"


class VqaGenerationConfig(BaseModel):
    """Processing policy shared by VQA generation runs."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    synchronization_tolerance_s: float = Field(default=0.1, gt=0)


class PublicCase(BaseModel):
    """One evaluator-facing multiple-choice VQA case."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    id: NonEmptyString
    image: NonEmptyString
    question: NonEmptyString
    choices: tuple[NonEmptyString, ...]

    @model_validator(mode="after")
    def choices_are_distinct(self) -> PublicCase:
        unique_choices = {choice.casefold() for choice in self.choices}
        if len(self.choices) < 2 or len(unique_choices) != len(self.choices):
            raise ValueError("a VQA case requires at least two unique choices")
        return self


class PrivateLabel(BaseModel):
    """The private expected choice for one public case."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    id: NonEmptyString
    answer: NonEmptyString


class GenerationResult(BaseModel):
    """Artifacts produced for selected recorded images."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    output: Path
    cases: tuple[PublicCase, ...]


@dataclass(frozen=True)
class GenerationFrame:
    """One indexed image with optional image-aligned point-cloud evidence."""

    index: int
    image: Image
    pointcloud_frame: PointCloudFrame | None = None


@dataclass(frozen=True)
class GeneratedFrame:
    """Questions and evidence generated for one source frame."""

    index: int
    image: Image
    cases: tuple[PublicCase, ...]
    labels: tuple[PrivateLabel, ...]
    audit_rows: tuple[dict[str, object], ...]
    families: tuple[FamilySpec, ...]


def generate_dataset(
    request: GenerationRequest,
    config: VqaGenerationConfig | None = None,
) -> GenerationResult:
    """Generate a standalone dataset from selected Memory images."""
    # Load optional model dependencies only when generation is requested.
    from dimos.models.vl.moondream import MoondreamVlModel
    from dimos.models.vl.openai import OpenAIVlModel

    config = config or VqaGenerationConfig()
    indices = request.frame_indices()
    with PointCloudFrameLoader(
        request.dataset,
        tolerance_s=config.synchronization_tolerance_s,
    ) as loader:
        image_count = loader.image_count
        if indices[-1] >= image_count:
            raise IndexError(
                f"color_image index {indices[-1]} is out of range for {image_count} images"
            )

        author_model = OpenAIVlModel()
        detector_model = MoondreamVlModel()
        try:
            model_names = {
                "author": author_model.config.model_name,
                "detector": detector_model.config.model_name,
            }
            mask_estimator = EdgeTAMObjectMaskPipeline(detector_model)

            def pointcloud_frames() -> Iterable[GenerationFrame]:
                for index in indices:
                    try:
                        pointcloud_frame = loader.load(index)
                    except PointCloudFrameUnavailableError:
                        yield GenerationFrame(index, loader.load_image(index))
                    else:
                        yield GenerationFrame(index, pointcloud_frame.image, pointcloud_frame)

            return generate_frames_dataset(
                request,
                pointcloud_frames(),
                OpenAIQuestionAuthor(author_model),
                detector_model,
                LidarRangeEstimator(mask_estimator),
                mask_estimator,
                config=config,
                model_names=model_names,
            )
        finally:
            author_model.stop()
            detector_model.stop()


def generate_frames_dataset(
    request: GenerationRequest,
    frames: Iterable[GenerationFrame],
    author: QuestionAuthor,
    detector: VlModel,
    range_estimator: ObjectRangeEstimator | None = None,
    mask_estimator: ObjectMaskEstimator | None = None,
    *,
    config: VqaGenerationConfig | None = None,
    model_names: dict[str, str] | None = None,
) -> GenerationResult:
    """Generate and write one dataset from already loaded indexed images."""
    config = config or VqaGenerationConfig()
    output = request.output_directory()
    _prepare_output(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f".{output.name}-", dir=output.parent) as temporary:
        staging = Path(temporary)
        all_cases: list[PublicCase] = []
        all_labels: list[PrivateLabel] = []
        frame_count = 0
        rejected_count = 0
        used_family_names: set[str] = set()
        for source in frames:
            frame = generate_frame(source, author, detector, range_estimator, mask_estimator)
            _write_frame(staging, frame)
            all_cases.extend(frame.cases)
            all_labels.extend(frame.labels)
            frame_count += 1
            used_family_names.update(family.name for family in frame.families)
            rejected_count += sum(row.get("status") == "rejected" for row in frame.audit_rows)

        if frame_count == 0:
            raise ValueError("generation selected no frames")
        if not all_cases:
            raise ValueError("no selected frame produced an answerable question")
        _write_jsonl(
            staging / "cases.jsonl",
            [case.model_dump(mode="json") for case in all_cases],
        )
        _write_jsonl(
            staging / "labels.jsonl",
            [label.model_dump(mode="json") for label in all_labels],
        )
        _write_json(
            staging / "audit" / "run.json",
            {
                "generation": request.model_dump(mode="json", exclude={"output"}),
                "configuration": config.model_dump(mode="json"),
                "families": [
                    family.name for family in AVAILABLE_FAMILIES if family.name in used_family_names
                ],
                "models": model_names or {},
                "frame_count": frame_count,
                "question_count": len(all_cases),
                "rejected_question_count": rejected_count,
            },
        )
        if output.exists():
            output.rmdir()
        staging.replace(output)
    return GenerationResult(output=output, cases=tuple(all_cases))


def generate_frame(
    source: GenerationFrame,
    author: QuestionAuthor,
    detector: VlModel,
    range_estimator: ObjectRangeEstimator | None,
    mask_estimator: ObjectMaskEstimator | None,
) -> GeneratedFrame:
    """Generate answerable questions for one already loaded frame."""
    image_index = source.index
    image = source.image
    families = tuple(
        family
        for family in AVAILABLE_FAMILIES
        if not (family.requires_masks and mask_estimator is None)
        and not (
            family.requires_pointcloud
            and (source.pointcloud_frame is None or range_estimator is None)
        )
    )
    proposals = _deduplicate_proposals(author.propose(image, families))
    answered: list[tuple[QuestionProposal, FamilyAnswer]] = []
    audit_rows: list[dict[str, object]] = []
    for proposal in proposals:
        try:
            answer = answer_question(
                proposal,
                image,
                detector,
                source.pointcloud_frame,
                range_estimator,
                mask_estimator,
            )
        except (InsufficientEvidenceError, InvalidQuestionProposalError) as exc:
            audit_rows.append(
                {
                    "proposal": proposal.model_dump(mode="json", exclude_none=True),
                    "status": "rejected",
                    "reason": str(exc),
                }
            )
        else:
            answered.append((proposal, answer))
            audit_rows.append(
                {
                    "proposal": proposal.model_dump(mode="json", exclude_none=True),
                    "status": "answered",
                    "answer": answer.model_dump(mode="json"),
                }
            )
    case_ids = _case_ids(image_index, tuple(proposal for proposal, _ in answered))
    cases = tuple(
        PublicCase(
            id=case_id,
            image=f"assets/frame-{image_index:06d}.png",
            question=answer.question,
            choices=answer.choices,
        )
        for case_id, (_, answer) in zip(case_ids, answered, strict=True)
    )
    labels = tuple(
        PrivateLabel(id=case.id, answer=answer.answer)
        for case, (_, answer) in zip(cases, answered, strict=True)
    )
    return GeneratedFrame(
        index=image_index,
        image=image,
        cases=cases,
        labels=labels,
        audit_rows=tuple(audit_rows),
        families=families,
    )


def _deduplicate_proposals(proposals: Sequence[QuestionProposal]) -> tuple[QuestionProposal, ...]:
    unique: list[QuestionProposal] = []
    seen_object_sets: set[tuple[str, frozenset[str]]] = set()
    for proposal in proposals:
        if proposal.family in _UNORDERED_FAMILIES:
            object_set = (
                proposal.family,
                frozenset(name.casefold() for name in proposal.object_names),
            )
            if object_set in seen_object_sets:
                continue
            seen_object_sets.add(object_set)
        if proposal not in unique:
            unique.append(proposal)
    return tuple(unique)


def _case_ids(image_index: int, proposals: tuple[QuestionProposal, ...]) -> tuple[str, ...]:
    counts: dict[str, int] = {}
    identifiers: list[str] = []
    for proposal in proposals:
        object_names = proposal.object_names
        if proposal.family in _UNORDERED_FAMILIES:
            object_names = tuple(sorted(object_names, key=str.casefold))
        joined_names = "-vs-".join(object_names)
        object_id = re.sub(r"[^a-z0-9]+", "-", joined_names.casefold()).strip("-")
        base = f"frame-{image_index:06d}-{object_id or 'object'}-{proposal.family}"
        count = counts.get(base, 0) + 1
        counts[base] = count
        identifiers.append(base if count == 1 else f"{base}-{count}")
    return tuple(identifiers)


def _prepare_output(output: Path) -> None:
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"VQA output directory is not empty: {output}")


def _write_frame(output: Path, frame: GeneratedFrame) -> None:
    (output / "assets").mkdir(parents=True, exist_ok=True)
    frame_audit = output / "audit" / f"frame-{frame.index:06d}"
    frame_audit.mkdir(parents=True, exist_ok=True)

    image_name = f"assets/frame-{frame.index:06d}.png"
    image_path = output / image_name
    if not frame.image.save(str(image_path)):
        raise RuntimeError(f"failed to write VQA image: {image_path}")

    case_rows = [case.model_dump(mode="json") for case in frame.cases]
    label_rows = [label.model_dump(mode="json") for label in frame.labels]
    _write_json(frame_audit / "cases.json", case_rows)
    _write_json(frame_audit / "labels.json", label_rows)
    _write_json(frame_audit / "ground_truth.json", frame.audit_rows)
    _write_json(
        frame_audit / "frame.json",
        {
            "frame_index": frame.index,
            "image": image_name,
            "timestamp": frame.image.ts,
            "question_count": len(frame.cases),
            "rejected_question_count": sum(
                row.get("status") == "rejected" for row in frame.audit_rows
            ),
        },
    )


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
