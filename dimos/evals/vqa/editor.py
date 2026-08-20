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

"""In-memory editing session for an existing standalone VQA dataset."""

from __future__ import annotations

from collections.abc import Callable, Iterable
import json
from pathlib import Path, PurePosixPath
import re
import tempfile
from threading import RLock
from typing import TYPE_CHECKING, Any

import jsonlines
from pydantic import BaseModel, ConfigDict, Field, model_validator

from dimos.evals.vqa.contracts import NonEmptyString
from dimos.evals.vqa.families import AVAILABLE_FAMILIES
from dimos.evals.vqa.generate import (
    GeneratedFrame,
    GenerationFrame,
    PrivateLabel,
    PublicCase,
    generate_frame,
)
from dimos.evals.vqa.pointcloud_frame import (
    PointCloudFrameLoader,
    PointCloudFrameUnavailableError,
)

if TYPE_CHECKING:
    from dimos.evals.vqa.author import QuestionAuthor
    from dimos.evals.vqa.contracts import ObjectDetector, ObjectMaskEstimator, ObjectRangeEstimator
    from dimos.msgs.sensor_msgs.Image import Image

_FRAME_IMAGE = re.compile(r"^assets/frame-(\d{6})\.png$")


class EditableQuestion(BaseModel):
    """One editable multiple-choice question and private answer."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    id: NonEmptyString
    question: NonEmptyString
    choices: tuple[NonEmptyString, ...] = Field(min_length=2)
    answer: NonEmptyString

    @model_validator(mode="after")
    def valid_answer(self) -> EditableQuestion:
        if len(set(self.choices)) != len(self.choices):
            raise ValueError("choices must be unique")
        if self.answer not in self.choices:
            raise ValueError("answer must be one of the choices")
        return self


class FrameDraft(BaseModel):
    """The complete editable question set for one recording frame."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    index: int = Field(ge=0)
    questions: tuple[EditableQuestion, ...] = ()

    @model_validator(mode="after")
    def unique_question_ids(self) -> FrameDraft:
        identifiers = [question.id for question in self.questions]
        if len(set(identifiers)) != len(identifiers):
            raise ValueError("question IDs must be unique within a frame")
        return self


class EditorState(BaseModel):
    """Browser bootstrap metadata for an editor session."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    recording: str
    output: Path
    image_count: int
    existing_frames: tuple[int, ...]
    dirty_frames: tuple[int, ...]


class SubmitResult(BaseModel):
    """Summary of an explicit dataset update."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    output: Path
    frame_count: int
    question_count: int


class VqaEditorSession:
    """Own recording resources, generated drafts, and explicit dataset writes."""

    def __init__(
        self,
        recording: str,
        output: Path,
        *,
        preprocessor: PointCloudFrameLoader | None = None,
        frame_generator: Callable[[GenerationFrame], GeneratedFrame] | None = None,
    ) -> None:
        self.recording = recording
        self.output = output.expanduser().resolve()
        self._preprocessor = preprocessor or PointCloudFrameLoader(recording)
        self._frame_generator = frame_generator
        self._cases: list[PublicCase] = []
        self._labels: dict[str, PrivateLabel] = {}
        self._drafts: dict[int, FrameDraft] = {}
        self._generated_frames: dict[int, GeneratedFrame] = {}
        self._dirty: set[int] = set()
        self._image_count: int | None = None
        self._models: tuple[Any, Any, Any, Any] | None = None
        self._lock = RLock()

    def __enter__(self) -> VqaEditorSession:
        return self.start()

    def __exit__(self, *_: Any) -> None:
        self.stop()

    def start(self) -> VqaEditorSession:
        """Open recording streams and validate the target dataset."""
        with self._lock:
            self._load_dataset()
            self._preprocessor.start()
            self._image_count = self._preprocessor.image_count
        return self

    def stop(self) -> None:
        """Release recording and optional model resources."""
        with self._lock:
            self._preprocessor.stop()
            self._image_count = None
            if self._models is not None:
                author_model, detector_model, _, _ = self._models
                author_model.stop()
                detector_model.stop()
                self._models = None

    def state(self) -> EditorState:
        """Return immutable metadata used to initialize the browser."""
        with self._lock:
            return EditorState(
                recording=self.recording,
                output=self.output,
                image_count=self._require_image_count(),
                existing_frames=tuple(sorted(self._existing_frame_indices())),
                dirty_frames=tuple(sorted(self._dirty)),
            )

    def raw_image(self, frame_index: int) -> Image:
        """Load one source image without applying camera rectification."""
        with self._lock:
            self._validate_index(frame_index)
            return self._preprocessor.load_raw_image(frame_index)

    def draft(self, frame_index: int) -> FrameDraft:
        """Return a generated/edited draft or derive it from existing cases."""
        with self._lock:
            self._validate_index(frame_index)
            if frame_index in self._drafts:
                return self._drafts[frame_index]
            questions = tuple(
                EditableQuestion(
                    id=case.id,
                    question=case.question,
                    choices=case.choices,
                    answer=self._labels[case.id].answer,
                )
                for case in self._cases
                if _case_frame_index(case) == frame_index
            )
            return FrameDraft(index=frame_index, questions=questions)

    def replace_draft(self, draft: FrameDraft) -> FrameDraft:
        """Store one complete frame draft in memory until submission."""
        with self._lock:
            self._validate_index(draft.index)
            self._drafts[draft.index] = draft
            self._dirty.add(draft.index)
            return draft

    def generate(self, indices: Iterable[int]) -> tuple[FrameDraft, ...]:
        """Generate and retain drafts for selected recording frame indices."""
        selected = tuple(indices)
        if not selected:
            raise ValueError("generation selected no frames")
        generated: list[FrameDraft] = []
        with self._lock:
            for index in selected:
                self._validate_index(index)
            for index in selected:
                try:
                    calibrated = self._preprocessor.load(index)
                except PointCloudFrameUnavailableError:
                    source = GenerationFrame(index, self._preprocessor.load_image(index))
                else:
                    source = GenerationFrame(index, calibrated.image, calibrated)
                frame = self._generate(source)
                labels = {label.id: label.answer for label in frame.labels}
                draft = FrameDraft(
                    index=index,
                    questions=tuple(
                        EditableQuestion(
                            id=case.id,
                            question=case.question,
                            choices=case.choices,
                            answer=labels[case.id],
                        )
                        for case in frame.cases
                    ),
                )
                self._drafts[index] = draft
                self._generated_frames[index] = frame
                self._dirty.add(index)
                generated.append(draft)
        return tuple(generated)

    def submit(self) -> SubmitResult:
        """Merge dirty frame drafts into the target dataset with atomic file writes."""
        with self._lock:
            dirty = set(self._dirty)
            if not dirty:
                return SubmitResult(output=self.output, frame_count=0, question_count=0)

            cases = [case for case in self._cases if _case_frame_index(case) not in dirty]
            labels = {case.id: self._labels[case.id] for case in cases}
            new_cases: list[PublicCase] = []
            for index in sorted(dirty):
                image_name = f"assets/frame-{index:06d}.png"
                for question in self._drafts[index].questions:
                    case = PublicCase(
                        id=question.id,
                        image=image_name,
                        question=question.question,
                        choices=question.choices,
                    )
                    if case.id in labels:
                        raise ValueError(
                            f"question ID already exists on another frame: {case.id!r}"
                        )
                    cases.append(case)
                    new_cases.append(case)
                    labels[case.id] = PrivateLabel(id=case.id, answer=question.answer)

            if len(labels) != len(cases):
                raise ValueError("question IDs must be unique across the dataset")

            assets = self.output / "assets"
            assets.mkdir(parents=True, exist_ok=True)
            for index in sorted(dirty):
                if not self._drafts[index].questions:
                    continue
                generated_frame = self._generated_frames.get(index)
                image = (
                    generated_frame.image
                    if generated_frame is not None
                    else self._preprocessor.load_image(index)
                )
                temporary_image = assets / f".frame-{index:06d}.tmp.png"
                if not image.save(str(temporary_image)):
                    raise RuntimeError(f"failed to write VQA image: {temporary_image}")
                temporary_image.replace(assets / f"frame-{index:06d}.png")

            for index in sorted(dirty):
                frame_cases = [case for case in cases if _case_frame_index(case) == index]
                _write_frame_audit(
                    self.output,
                    index,
                    frame_cases,
                    [labels[case.id] for case in frame_cases],
                    self._generated_frames.get(index),
                )
            _write_run_audit(
                self.output,
                self.recording,
                cases,
                dirty,
            )

            _write_jsonl_atomic(
                self.output / "labels.jsonl",
                (labels[case.id].model_dump(mode="json") for case in cases),
            )
            _write_jsonl_atomic(
                self.output / "cases.jsonl",
                (case.model_dump(mode="json") for case in cases),
            )
            self._cases = cases
            self._labels = labels
            for index in dirty:
                self._generated_frames.pop(index, None)
            self._dirty.clear()
            return SubmitResult(
                output=self.output,
                frame_count=len(dirty),
                question_count=len(new_cases),
            )

    def _load_dataset(self) -> None:
        cases_path = self.output / "cases.jsonl"
        labels_path = self.output / "labels.jsonl"
        if not cases_path.is_file() or not labels_path.is_file():
            raise ValueError(f"target is not an existing VQA dataset: {self.output}")
        try:
            with jsonlines.open(cases_path) as reader:
                cases = [PublicCase.model_validate(row) for row in reader.iter(skip_empty=True)]
            with jsonlines.open(labels_path) as reader:
                parsed_labels = [
                    PrivateLabel.model_validate(row) for row in reader.iter(skip_empty=True)
                ]
        except (OSError, jsonlines.Error) as exc:
            raise ValueError(f"invalid VQA dataset: {self.output}") from exc
        labels = {label.id: label for label in parsed_labels}
        if len(labels) != len(parsed_labels) or len({case.id for case in cases}) != len(cases):
            raise ValueError("VQA dataset contains duplicate question IDs")
        if set(labels) != {case.id for case in cases}:
            raise ValueError("VQA case and label IDs do not match")
        self._cases = cases
        self._labels = labels

    def _existing_frame_indices(self) -> set[int]:
        return {index for case in self._cases if (index := _case_frame_index(case)) is not None}

    def _validate_index(self, frame_index: int) -> None:
        image_count = self._require_image_count()
        if frame_index < 0 or frame_index >= image_count:
            raise IndexError(
                f"color_image index {frame_index} is out of range for {image_count} images"
            )

    def _require_image_count(self) -> int:
        if self._image_count is None:
            raise RuntimeError("VQA editor session is not started")
        return self._image_count

    def _generate(self, source: GenerationFrame) -> GeneratedFrame:
        if self._frame_generator is not None:
            return self._frame_generator(source)
        author, detector, range_estimator, mask_estimator = self._generation_models()
        return generate_frame(source, author, detector, range_estimator, mask_estimator)

    def _generation_models(
        self,
    ) -> tuple[QuestionAuthor, ObjectDetector, ObjectRangeEstimator, ObjectMaskEstimator]:
        from dimos.evals.vqa.author import OpenAIQuestionAuthor

        if self._models is None:
            from dimos.evals.vqa.primitives.edge_tam import EdgeTAMObjectMaskPipeline
            from dimos.evals.vqa.primitives.range import LidarRangeEstimator
            from dimos.models.vl.moondream import MoondreamVlModel
            from dimos.models.vl.openai import OpenAIVlModel

            author_model = OpenAIVlModel()
            detector_model = MoondreamVlModel()
            mask_estimator = EdgeTAMObjectMaskPipeline(detector_model)
            self._models = (
                author_model,
                detector_model,
                LidarRangeEstimator(mask_estimator),
                mask_estimator,
            )
        author_model, detector_model, range_estimator, mask_estimator = self._models
        return (
            OpenAIQuestionAuthor(author_model),
            detector_model,
            range_estimator,
            mask_estimator,
        )


def _case_frame_index(case: PublicCase) -> int | None:
    path = PurePosixPath(case.image)
    match = _FRAME_IMAGE.fullmatch(path.as_posix())
    return int(match.group(1)) if match else None


def _write_jsonl_atomic(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}-", delete=False
        ) as handle:
            temporary = Path(handle.name)
            for row in rows:
                handle.write(json.dumps(row) + "\n")
            handle.flush()
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _write_frame_audit(
    output: Path,
    index: int,
    cases: list[PublicCase],
    labels: list[PrivateLabel],
    generated: GeneratedFrame | None,
) -> None:
    audit = output / "audit" / f"frame-{index:06d}"
    audit.mkdir(parents=True, exist_ok=True)
    previous_frame = _read_json(audit / "frame.json", {})
    if not isinstance(previous_frame, dict):
        previous_frame = {}
    _write_json_atomic(audit / "cases.json", [case.model_dump(mode="json") for case in cases])
    _write_json_atomic(audit / "labels.json", [label.model_dump(mode="json") for label in labels])
    ground_truth = audit / "ground_truth.json"
    if generated is not None:
        _write_json_atomic(ground_truth, list(generated.audit_rows))
    elif not ground_truth.exists():
        _write_json_atomic(ground_truth, [])
    rejected_count = 0
    if ground_truth.is_file():
        rows = _read_json(ground_truth, [])
        if isinstance(rows, list):
            rejected_count = sum(
                isinstance(row, dict) and row.get("status") == "rejected" for row in rows
            )
    else:
        rows = []
    attempted_families, answered_families = _audit_families(rows)
    timestamp = generated.image.ts if generated is not None else previous_frame.get("timestamp")
    _write_json_atomic(
        audit / "frame.json",
        {
            "frame_index": index,
            "image": f"assets/frame-{index:06d}.png",
            "timestamp": timestamp,
            "question_count": len(cases),
            "rejected_question_count": rejected_count,
            "available_families": (
                [family.name for family in generated.families]
                if generated is not None
                else previous_frame.get("available_families", [])
            ),
            "attempted_families": attempted_families,
            "answered_families": answered_families,
            "reviewed": True,
        },
    )


def _write_run_audit(
    output: Path,
    recording: str,
    cases: list[PublicCase],
    submitted_frames: set[int],
) -> None:
    audit = output / "audit"
    frame_indices = {_case_frame_index(case) for case in cases}
    frame_indices.discard(None)
    audit_directories = tuple(audit.glob("frame-*")) if audit.exists() else ()
    rejected_count = 0
    for directory in audit_directories:
        rows = _read_json(directory / "ground_truth.json", [])
        if isinstance(rows, list):
            rejected_count += sum(
                isinstance(row, dict) and row.get("status") == "rejected" for row in rows
            )
    attempted_families: set[str] = set()
    answered_families: set[str] = set()
    for directory in audit_directories:
        frame = _read_json(directory / "frame.json", {})
        if isinstance(frame, dict):
            attempted_families.update(frame.get("attempted_families", []))
            answered_families.update(frame.get("answered_families", []))
    answered_families.update(
        family.name
        for case in cases
        for family in AVAILABLE_FAMILIES
        if case.id.endswith(f"-{family.name}")
    )
    _write_json_atomic(
        audit / "run.json",
        {
            "generation": {"dataset": recording},
            "families": sorted(answered_families),
            "attempted_families": sorted(attempted_families),
            "frame_count": len(frame_indices),
            "audited_frame_count": len(audit_directories),
            "question_count": len(cases),
            "rejected_question_count": rejected_count,
            "submitted_frames": sorted(submitted_frames),
            "editor": True,
        },
    )


def _write_json_atomic(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}-", delete=False
        ) as handle:
            temporary = Path(handle.name)
            json.dump(value, handle, indent=2)
            handle.write("\n")
            handle.flush()
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _read_json(path: Path, default: Any) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return default


def _audit_families(rows: Any) -> tuple[list[str], list[str]]:
    if not isinstance(rows, list):
        return ([], [])
    attempted: set[str] = set()
    answered: set[str] = set()
    for row in rows:
        if not isinstance(row, dict):
            continue
        proposal = row.get("proposal")
        if not isinstance(proposal, dict) or not isinstance(proposal.get("family"), str):
            continue
        family = proposal["family"]
        attempted.add(family)
        if row.get("status") == "answered":
            answered.add(family)
    return (sorted(attempted), sorted(answered))
