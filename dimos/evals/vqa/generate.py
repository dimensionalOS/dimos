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

from collections.abc import Sequence
import json
from pathlib import Path
import re
from typing import TYPE_CHECKING

from pydantic import BaseModel, ConfigDict, Field

from dimos.evals.vqa.author import OpenAIQuestionAuthor, QuestionAuthor
from dimos.evals.vqa.families import (
    AVAILABLE_FAMILIES,
    FamilyAnswer,
    InsufficientEvidenceError,
    NonEmptyString,
    ObjectDetector,
    QuestionProposal,
    answer_question,
)

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.Image import Image


class GenerationRequest(BaseModel):
    """One image selected from a recorded Memory dataset."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    dataset: NonEmptyString
    image_index: int = Field(ge=0)
    output: Path


class PublicCase(BaseModel):
    """One evaluator-facing multiple-choice VQA case."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    id: NonEmptyString
    image: NonEmptyString
    question: NonEmptyString
    choices: tuple[NonEmptyString, ...]


class PrivateLabel(BaseModel):
    """The private expected choice for one public case."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    id: NonEmptyString
    answer: NonEmptyString


class GenerationResult(BaseModel):
    """Artifacts produced for one recorded image."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    output: Path
    cases: tuple[PublicCase, ...]


def generate_dataset(request: GenerationRequest) -> GenerationResult:
    """Generate a standalone dataset from one Memory image."""
    from dimos.models.vl.moondream import MoondreamVlModel
    from dimos.models.vl.openai import OpenAIVlModel

    image = load_memory_image(request.dataset, request.image_index)
    author_model = OpenAIVlModel()
    detector_model = MoondreamVlModel()
    try:
        from dimos.evals.vqa.primitives.moondream import MoondreamObjectDetector

        return generate_image_dataset(
            request,
            image,
            OpenAIQuestionAuthor(author_model),
            MoondreamObjectDetector(detector_model),
        )
    finally:
        author_model.stop()
        detector_model.stop()


def load_memory_image(dataset: str, image_index: int) -> Image:
    """Load one indexed image from a recorded Memory dataset."""
    from dimos.memory.cli.dataset import open_dataset
    from dimos.msgs.sensor_msgs.Image import Image

    store = open_dataset(dataset)
    try:
        observation = store.streams.color_image.offset(image_index).first()
        image = observation.data
        if not isinstance(image, Image):
            raise TypeError("color_image stream must contain dimos Image values")
        image = image.copy()
        image.ts = observation.ts
        return image
    finally:
        store.stop()


def generate_image_dataset(
    request: GenerationRequest,
    image: Image,
    author: QuestionAuthor,
    detector: ObjectDetector,
) -> GenerationResult:
    """Generate and write one dataset from an already loaded image."""
    proposals = _deduplicate_proposals(author.propose(image, AVAILABLE_FAMILIES))
    if not proposals:
        raise ValueError("question author returned no proposals")

    answered: list[tuple[QuestionProposal, FamilyAnswer]] = []
    audit_rows: list[dict[str, object]] = []
    rejected_count = 0
    for proposal in proposals:
        try:
            answer = answer_question(proposal, image, detector)
        except InsufficientEvidenceError as exc:
            rejected_count += 1
            audit_rows.append(
                {
                    "proposal": proposal.model_dump(mode="json"),
                    "status": "rejected",
                    "reason": str(exc),
                }
            )
        else:
            answered.append((proposal, answer))
            audit_rows.append(
                {
                    "proposal": proposal.model_dump(mode="json"),
                    "status": "answered",
                    "answer": answer.model_dump(mode="json"),
                }
            )
    if not answered:
        raise ValueError("no authored question had sufficient deterministic evidence")

    answers = tuple(answer for _, answer in answered)
    case_ids = _case_ids(request.image_index, tuple(proposal for proposal, _ in answered))
    cases = tuple(
        PublicCase(
            id=case_id,
            image=f"assets/frame-{request.image_index:06d}.jpg",
            question=answer.question,
            choices=answer.choices,
        )
        for case_id, (_, answer) in zip(case_ids, answered, strict=True)
    )
    labels = tuple(
        PrivateLabel(id=case.id, answer=answer.answer)
        for case, answer in zip(cases, answers, strict=True)
    )
    _write_dataset(
        request,
        image,
        cases,
        labels,
        audit_rows,
        rejected_count,
    )
    return GenerationResult(output=request.output, cases=cases)


def _deduplicate_proposals(proposals: Sequence[QuestionProposal]) -> tuple[QuestionProposal, ...]:
    unique: list[QuestionProposal] = []
    for proposal in proposals:
        if proposal not in unique:
            unique.append(proposal)
    return tuple(unique)


def _case_ids(image_index: int, proposals: tuple[QuestionProposal, ...]) -> tuple[str, ...]:
    counts: dict[str, int] = {}
    identifiers: list[str] = []
    for proposal in proposals:
        object_id = re.sub(r"[^a-z0-9]+", "-", proposal.object_name.casefold()).strip("-")
        base = f"frame-{image_index:06d}-{object_id or 'object'}-{proposal.family}"
        count = counts.get(base, 0) + 1
        counts[base] = count
        identifiers.append(base if count == 1 else f"{base}-{count}")
    return tuple(identifiers)


def _write_dataset(
    request: GenerationRequest,
    image: Image,
    cases: tuple[PublicCase, ...],
    labels: tuple[PrivateLabel, ...],
    audit_rows: list[dict[str, object]],
    rejected_count: int,
) -> None:
    output = request.output
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"VQA output directory is not empty: {output}")

    assets = output / "assets"
    frame_audit = output / "audit" / f"frame-{request.image_index:06d}"
    assets.mkdir(parents=True, exist_ok=True)
    frame_audit.mkdir(parents=True, exist_ok=True)

    image_path = output / cases[0].image
    if not image.save(str(image_path)):
        raise RuntimeError(f"failed to write VQA image: {image_path}")

    case_rows = [case.model_dump(mode="json") for case in cases]
    label_rows = [label.model_dump(mode="json") for label in labels]
    _write_jsonl(output / "cases.jsonl", case_rows)
    _write_jsonl(output / "labels.jsonl", label_rows)
    _write_json(frame_audit / "cases.json", case_rows)
    _write_json(frame_audit / "labels.json", label_rows)
    _write_json(frame_audit / "ground_truth.json", audit_rows)
    _write_json(
        frame_audit / "frame.json",
        {
            "frame_index": request.image_index,
            "image": cases[0].image,
            "timestamp": image.ts,
            "question_count": len(cases),
            "rejected_question_count": rejected_count,
        },
    )
    _write_json(
        output / "audit" / "run.json",
        {
            "dataset": request.dataset,
            "image_index": request.image_index,
            "families": [family.name for family in AVAILABLE_FAMILIES],
            "question_count": len(cases),
            "rejected_question_count": rejected_count,
        },
    )


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def _write_jsonl(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
