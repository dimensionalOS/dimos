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

from collections.abc import Callable, Iterable, Iterator
from contextlib import contextmanager
import fcntl
import hashlib
from io import BytesIO
import json
import math
from pathlib import Path, PurePosixPath
import re
import shutil
import tempfile
from threading import RLock
from typing import TYPE_CHECKING, Any, BinaryIO

import jsonlines
import numpy as np
from PIL import Image as PILImage, ImageDraw
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
    TopDownFrame,
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
    depth_attempt_count: int = Field(default=0, ge=0)
    depth_answered_count: int = Field(default=0, ge=0)
    depth_rejections: tuple[str, ...] = ()

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
    total_questions: int
    has_topdown: bool
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
        self._loaded_dataset_version: tuple[bytes | None, bytes | None] = (None, None)
        self._dataset_lock: BinaryIO | None = None
        self._lock = RLock()

    def __enter__(self) -> VqaEditorSession:
        return self.start()

    def __exit__(self, *_: Any) -> None:
        self.stop()

    def start(self) -> VqaEditorSession:
        """Open recording streams and initialize or load the target dataset."""
        with self._lock:
            if self._image_count is not None:
                return self
            self._acquire_dataset_lock()
            try:
                self._load_dataset()
                self._loaded_dataset_version = _dataset_version(self.output)
                self._preprocessor.start()
                self._image_count = self._preprocessor.image_count
            except BaseException:
                try:
                    self._preprocessor.stop()
                finally:
                    self._image_count = None
                    self._release_dataset_lock()
                raise
        return self

    def stop(self) -> None:
        """Release recording and optional model resources."""
        with self._lock:
            try:
                self._preprocessor.stop()
            finally:
                self._image_count = None
                models, self._models = self._models, None
                try:
                    if models is not None:
                        author_model, detector_model, _, _ = models
                        try:
                            author_model.stop()
                        finally:
                            detector_model.stop()
                finally:
                    self._release_dataset_lock()

    def preload_generation_models(self) -> None:
        """Load the editor's local generation models before serving requests."""
        with self._lock:
            if self._frame_generator is None:
                self._generation_models()

    def state(self) -> EditorState:
        """Return immutable metadata used to initialize the browser."""
        with self._lock:
            dirty = set(self._dirty)
            total_questions = sum(
                _case_frame_index(case) not in dirty for case in self._cases
            ) + sum(len(self._drafts[index].questions) for index in dirty)
            return EditorState(
                recording=self.recording,
                output=self.output,
                image_count=self._require_image_count(),
                total_questions=total_questions,
                has_topdown=getattr(self._preprocessor, "topdown_available", False),
                existing_frames=tuple(sorted(self._existing_frame_indices())),
                dirty_frames=tuple(sorted(self._dirty)),
            )

    def raw_image(self, frame_index: int) -> Image:
        """Load one source image without applying camera rectification."""
        with self._lock:
            self._validate_index(frame_index)
            return self._preprocessor.load_raw_image(frame_index)

    def topdown_image(self, frame_index: int) -> bytes:
        """Render synchronized world-frame LiDAR and robot pose as a PNG."""
        with self._lock:
            self._validate_index(frame_index)
            return _render_topdown(self._preprocessor.load_topdown(frame_index))

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

    def generate(self, index: int) -> FrameDraft:
        """Generate and retain a draft for one recording frame."""
        with self._lock:
            self._validate_index(index)
            try:
                calibrated = self._preprocessor.load(index)
            except PointCloudFrameUnavailableError:
                source = GenerationFrame(index, self._preprocessor.load_image(index))
            else:
                source = GenerationFrame(index, calibrated.image, calibrated)
            frame = self._generate(source)
            labels = {label.id: label.answer for label in frame.labels}
            questions = tuple(
                EditableQuestion(
                    id=case.id,
                    question=case.question,
                    choices=case.choices,
                    answer=labels[case.id],
                )
                for case in frame.cases
            )
            draft = FrameDraft(
                index=index,
                questions=questions or self.draft(index).questions,
                depth_attempt_count=sum(_is_depth_audit_row(row) for row in frame.audit_rows),
                depth_answered_count=sum(
                    _is_depth_audit_row(row) and row.get("status") == "answered"
                    for row in frame.audit_rows
                ),
                depth_rejections=tuple(
                    str(row["reason"])
                    for row in frame.audit_rows
                    if _is_depth_audit_row(row)
                    and row.get("status") == "rejected"
                    and "reason" in row
                ),
            )
            self._drafts[index] = draft
            self._generated_frames[index] = frame
            self._dirty.add(index)
            return draft

    def submit(self) -> SubmitResult:
        """Merge dirty frame drafts into the target dataset with atomic file writes."""
        with self._lock:
            dirty_indices = tuple(sorted(self._dirty))
            if not dirty_indices:
                return SubmitResult(output=self.output, frame_count=0, question_count=0)
            dirty = set(dirty_indices)
            if _dataset_version(self.output) != self._loaded_dataset_version:
                raise RuntimeError(
                    "VQA dataset changed on disk; restart the editor before submitting"
                )

            cases = [case for case in self._cases if _case_frame_index(case) not in dirty]
            labels = {case.id: self._labels[case.id] for case in cases}
            new_cases: list[PublicCase] = []
            for index in dirty_indices:
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

            changed_paths = [
                self.output / "cases.jsonl",
                self.output / "labels.jsonl",
                self.output / "audit" / "run.json",
                *(self.output / "audit" / f"frame-{index:06d}" for index in dirty_indices),
                *(
                    self.output / "assets" / f"frame-{index:06d}.png"
                    for index in dirty_indices
                    if self._drafts[index].questions
                ),
            ]
            with _rollback_paths(changed_paths):
                assets = self.output / "assets"
                assets.mkdir(parents=True, exist_ok=True)
                for index in dirty_indices:
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

                cases_by_frame: dict[int, list[PublicCase]] = {index: [] for index in dirty_indices}
                for case in cases:
                    case_index = _case_frame_index(case)
                    if case_index in cases_by_frame:
                        cases_by_frame[case_index].append(case)
                for index, frame_cases in cases_by_frame.items():
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
                new_dataset_version = _dataset_version(self.output)
            self._cases = cases
            self._labels = labels
            self._loaded_dataset_version = new_dataset_version
            for index in dirty:
                self._generated_frames.pop(index, None)
            self._dirty.clear()
            return SubmitResult(
                output=self.output,
                frame_count=len(dirty_indices),
                question_count=len(new_cases),
            )

    def _load_dataset(self) -> None:
        cases_path = self.output / "cases.jsonl"
        labels_path = self.output / "labels.jsonl"
        if not self.output.exists():
            self.output.mkdir(parents=True)
            return
        if not self.output.is_dir():
            raise ValueError(f"VQA output is not a directory: {self.output}")
        if not cases_path.exists() and not labels_path.exists():
            if any(self.output.iterdir()):
                raise ValueError(
                    f"VQA output must be empty or contain cases.jsonl and labels.jsonl: {self.output}"
                )
            return
        if not cases_path.is_file() or not labels_path.is_file():
            raise ValueError(
                f"VQA output must contain both cases.jsonl and labels.jsonl: {self.output}"
            )
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
        for case in cases:
            image_path = PurePosixPath(case.image)
            if image_path.is_absolute() or ".." in image_path.parts:
                raise ValueError(f"VQA case has unsafe image path: {case.image!r}")
            EditableQuestion(
                id=case.id,
                question=case.question,
                choices=case.choices,
                answer=labels[case.id].answer,
            )
        self._cases = cases
        self._labels = labels

    def _existing_frame_indices(self) -> set[int]:
        return {index for case in self._cases if (index := _case_frame_index(case)) is not None}

    def _acquire_dataset_lock(self) -> None:
        if self._dataset_lock is not None:
            return
        self.output.parent.mkdir(parents=True, exist_ok=True)
        lock_path = self.output.parent / f".{self.output.name}.editor.lock"
        handle = lock_path.open("a+b")
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            handle.close()
            raise RuntimeError(
                f"VQA dataset is already open in another editor: {self.output}"
            ) from exc
        self._dataset_lock = handle

    def _release_dataset_lock(self) -> None:
        handle, self._dataset_lock = self._dataset_lock, None
        if handle is not None:
            try:
                fcntl.flock(handle, fcntl.LOCK_UN)
            finally:
                handle.close()

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
            from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
            from dimos.models.vl.moondream import MoondreamVlModel
            from dimos.models.vl.openai import OpenAIVlModel

            author_model = OpenAIVlModel()
            detector_model = MoondreamVlModel()
            try:
                detector_model.start()
                segmenter = EdgeTAMImageSegmenter()
                mask_estimator = EdgeTAMObjectMaskPipeline(detector_model, segmenter)
                range_estimator = LidarRangeEstimator(mask_estimator)
            except Exception:
                author_model.stop()
                detector_model.stop()
                raise
            self._models = (
                author_model,
                detector_model,
                range_estimator,
                mask_estimator,
            )
        author_model, detector_model, range_estimator, mask_estimator = self._models
        return (
            OpenAIQuestionAuthor(author_model),
            detector_model,
            range_estimator,
            mask_estimator,
        )


def _render_topdown(frame: TopDownFrame, width: int = 960, height: int = 480) -> bytes:
    """Render a robot-centered crop from the cached global LiDAR map."""
    robot_x, robot_y = frame.pose.x, frame.pose.y
    half_width_m = 6.0
    half_height_m = half_width_m * height / width
    scale = width / (2 * half_width_m)
    lidar_map = frame.lidar_map
    crop_width = round(2 * half_width_m / lidar_map.resolution)
    crop_height = round(2 * half_height_m / lidar_map.resolution)
    center_x = (robot_x - lidar_map.origin_x) / lidar_map.resolution
    center_y = (robot_y - lidar_map.origin_y) / lidar_map.resolution
    source_x0 = round(center_x - crop_width / 2)
    source_y0 = round(center_y - crop_height / 2)
    source_x1 = source_x0 + crop_width
    source_y1 = source_y0 + crop_height
    map_height, map_width = lidar_map.hits.shape
    clipped_x0, clipped_x1 = max(0, source_x0), min(map_width, source_x1)
    clipped_y0, clipped_y1 = max(0, source_y0), min(map_height, source_y1)
    cropped = np.zeros((crop_height, crop_width), dtype=lidar_map.hits.dtype)
    if clipped_x1 > clipped_x0 and clipped_y1 > clipped_y0:
        target_x0, target_y0 = clipped_x0 - source_x0, clipped_y0 - source_y0
        cropped[
            target_y0 : target_y0 + clipped_y1 - clipped_y0,
            target_x0 : target_x0 + clipped_x1 - clipped_x0,
        ] = lidar_map.hits[clipped_y0:clipped_y1, clipped_x0:clipped_x1]

    pixels = np.full((crop_height, crop_width, 3), 21, dtype=np.uint8)
    occupied = cropped > 0
    intensity = np.clip(105 + np.log1p(cropped) * 24, 0, 193).astype(np.uint8)
    pixels[occupied] = np.repeat(intensity[occupied][:, None], 3, axis=1)
    image = PILImage.fromarray(np.flipud(pixels), mode="RGB").resize(
        (width, height), PILImage.Resampling.NEAREST
    )
    draw = ImageDraw.Draw(image)

    def pixel(x: float, y: float) -> tuple[int, int]:
        return (
            round(width / 2 + (x - robot_x) * scale),
            round(height / 2 - (y - robot_y) * scale),
        )

    for world_x in range(math.floor(robot_x - half_width_m), math.ceil(robot_x + half_width_m)):
        x, _ = pixel(float(world_x), robot_y)
        draw.line((x, 0, x, height), fill="#2b2b2b")
    for world_y in range(math.floor(robot_y - half_height_m), math.ceil(robot_y + half_height_m)):
        _, y = pixel(robot_x, float(world_y))
        draw.line((0, y, width, y), fill="#2b2b2b")

    yaw = frame.pose.yaw
    cosine, sine = math.cos(yaw), math.sin(yaw)
    footprint = []
    for local_x, local_y in ((0.35, 0.2), (0.35, -0.2), (-0.35, -0.2), (-0.35, 0.2)):
        footprint.append(
            pixel(
                robot_x + local_x * cosine - local_y * sine,
                robot_y + local_x * sine + local_y * cosine,
            )
        )
    draw.polygon(footprint, fill="#404040", outline="#b0e1f0", width=3)
    center = pixel(robot_x, robot_y)
    heading = pixel(robot_x + cosine, robot_y + sine)
    draw.line((*center, *heading), fill="#b0e1f0", width=4)
    draw.ellipse(
        (heading[0] - 5, heading[1] - 5, heading[0] + 5, heading[1] + 5),
        fill="#b0e1f0",
    )
    draw.text((12, 10), "GLOBAL LIDAR MAP / 1M GRID", fill="#b0e1f0")
    draw.text((12, height - 22), "12M x 6M / FULL RECORDING", fill="#c1c1c1")

    output = BytesIO()
    image.save(output, format="PNG")
    return output.getvalue()


def _dataset_version(root: Path) -> tuple[bytes | None, bytes | None]:
    """Fingerprint the files rewritten by submission to detect another editor."""

    def digest(path: Path) -> bytes | None:
        return hashlib.sha256(path.read_bytes()).digest() if path.is_file() else None

    return digest(root / "cases.jsonl"), digest(root / "labels.jsonl")


def _case_frame_index(case: PublicCase) -> int | None:
    path = PurePosixPath(case.image)
    match = _FRAME_IMAGE.fullmatch(path.as_posix())
    return int(match.group(1)) if match else None


def _is_depth_audit_row(row: dict[str, object]) -> bool:
    proposal = row.get("proposal")
    return isinstance(proposal, dict) and proposal.get("family") in {
        "object_distance",
        "closest_object",
    }


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


@contextmanager
def _rollback_paths(paths: Iterable[Path]) -> Iterator[None]:
    unique_paths = tuple(dict.fromkeys(paths))
    with tempfile.TemporaryDirectory(prefix=".vqa-editor-", dir=unique_paths[0].parent) as raw:
        backup_root = Path(raw)
        existing: set[int] = set()
        for index, path in enumerate(unique_paths):
            if not path.exists():
                continue
            existing.add(index)
            backup = backup_root / str(index)
            if path.is_dir():
                shutil.copytree(path, backup)
            else:
                shutil.copy2(path, backup)
        try:
            yield
        except Exception:
            for index, path in enumerate(unique_paths):
                if path.is_dir():
                    shutil.rmtree(path)
                else:
                    path.unlink(missing_ok=True)
                if index not in existing:
                    continue
                backup = backup_root / str(index)
                path.parent.mkdir(parents=True, exist_ok=True)
                if backup.is_dir():
                    shutil.copytree(backup, path)
                else:
                    shutil.copy2(backup, path)
            raise


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
    attempted_families: set[str] = set()
    answered_families: set[str] = set()
    for directory in audit_directories:
        rows = _read_json(directory / "ground_truth.json", [])
        if isinstance(rows, list):
            rejected_count += sum(
                isinstance(row, dict) and row.get("status") == "rejected" for row in rows
            )
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
