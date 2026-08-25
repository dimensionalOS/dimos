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
from dimos.evals.vqa.families import (
    AVAILABLE_FAMILIES,
    FamilyAnswer,
    InsufficientEvidenceError,
    NonEmptyString,
    QuestionProposal,
    answer_question,
)
from dimos.memory.cli.dataset import open_dataset
from dimos.msgs.sensor_msgs.Image import Image

if TYPE_CHECKING:
    from dimos.models.vl.base import VlModel


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
class _GeneratedFrame:
    index: int
    image: Image
    cases: tuple[PublicCase, ...]
    labels: tuple[PrivateLabel, ...]
    audit_rows: tuple[dict[str, object], ...]


def generate_dataset(request: GenerationRequest) -> GenerationResult:
    """Generate a standalone dataset from selected Memory images."""
    # Load optional model dependencies only when generation is requested.
    from dimos.models.vl.moondream import MoondreamVlModel
    from dimos.models.vl.openai import OpenAIVlModel

    indices = request.frame_indices()
    store = open_dataset(request.dataset)
    author_model = OpenAIVlModel()
    detector_model = MoondreamVlModel()
    try:
        images = store.streams.color_image
        image_count = images.count()
        if indices[-1] >= image_count:
            raise IndexError(
                f"color_image index {indices[-1]} is out of range for {image_count} images"
            )

        def selected_frames() -> Iterable[tuple[int, Image]]:
            for index in indices:
                observation = images.offset(index).first()
                yield index, _copy_observation_image(observation.data, observation.ts)

        return generate_frames_dataset(
            request,
            selected_frames(),
            OpenAIQuestionAuthor(author_model),
            detector_model,
            model_names={
                "author": author_model.config.model_name,
                "detector": detector_model.config.model_name,
            },
        )
    finally:
        store.stop()
        author_model.stop()
        detector_model.stop()


def _copy_observation_image(value: object, timestamp: float) -> Image:
    if not isinstance(value, Image):
        raise TypeError("color_image stream must contain dimos Image values")
    image = value.copy()
    image.ts = timestamp
    return image


def generate_frames_dataset(
    request: GenerationRequest,
    frames: Iterable[tuple[int, Image]],
    author: QuestionAuthor,
    detector: VlModel,
    *,
    model_names: dict[str, str] | None = None,
) -> GenerationResult:
    """Generate and write one dataset from already loaded indexed images."""
    output = request.output_directory()
    _prepare_output(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f".{output.name}-", dir=output.parent) as temporary:
        staging = Path(temporary)
        all_cases: list[PublicCase] = []
        all_labels: list[PrivateLabel] = []
        frame_count = 0
        rejected_count = 0
        for index, image in frames:
            frame = _generate_frame(index, image, author, detector)
            _write_frame(staging, frame)
            all_cases.extend(frame.cases)
            all_labels.extend(frame.labels)
            frame_count += 1
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
                "families": [family.name for family in AVAILABLE_FAMILIES],
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


def _generate_frame(
    image_index: int,
    image: Image,
    author: QuestionAuthor,
    detector: VlModel,
) -> _GeneratedFrame:
    proposals = _deduplicate_proposals(author.propose(image, AVAILABLE_FAMILIES))
    answered: list[tuple[QuestionProposal, FamilyAnswer]] = []
    audit_rows: list[dict[str, object]] = []
    for proposal in proposals:
        try:
            answer = answer_question(proposal, image, detector)
        except InsufficientEvidenceError as exc:
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
    answers = tuple(answer for _, answer in answered)
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
        for case, answer in zip(cases, answers, strict=True)
    )
    return _GeneratedFrame(
        index=image_index,
        image=image,
        cases=cases,
        labels=labels,
        audit_rows=tuple(audit_rows),
    )


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


def _prepare_output(output: Path) -> None:
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"VQA output directory is not empty: {output}")


def _write_frame(output: Path, frame: _GeneratedFrame) -> None:
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
