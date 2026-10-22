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
from contextlib import AbstractContextManager, nullcontext
from dataclasses import dataclass, replace
import json
from pathlib import Path
import re
import shutil
import tempfile
from typing import TYPE_CHECKING, TypeVar

from pydantic import BaseModel, ConfigDict, Field, model_validator

from dimos.constants import STATE_DIR
from dimos.evals.vqa.author import OpenAIQuestionAuthor, QuestionAuthor
from dimos.evals.vqa.clip import (
    clip_duration_s,
    ego_memory_clip,
    format_clip_span,
    write_image_clip,
)
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
from dimos.evals.vqa.localize_coverage import (
    DEFAULT_LOCALIZE_COVERAGE_K,
    CoverageCandidate,
    case_family_from_id,
    case_frame_from_id,
    collect_coverage_candidates_from_audit,
    select_localize_coverage_ids,
)
from dimos.evals.vqa.pointcloud_frame import (
    PointCloudFrame,
    PointCloudFrameLoader,
    PointCloudFrameUnavailableError,
    select_quality_indices,
)
from dimos.evals.vqa.primitives.cross_view_identity import (
    IdentityPair,
    answer_cross_view_same_object,
    collect_query_instances,
    pair_case_id,
    propose_identity_pairs,
    stack_pair_image,
)
from dimos.evals.vqa.primitives.edge_tam import EdgeTAMObjectMaskPipeline
from dimos.evals.vqa.primitives.localize_scene import LocalizeSceneEvidence
from dimos.evals.vqa.primitives.range import LidarRangeEstimator
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.evals.vqa.localize_recording import LocalizeWindowResult
    from dimos.models.vl.base import VlModel

logger = setup_logger()

_UNORDERED_FAMILIES = frozenset(
    family.name for family in AVAILABLE_FAMILIES if not family.object_order_matters
)


class GenerationRequest(BaseModel):
    """Generate a standalone VQA dataset from a Memory recording.

    Two modes share this request:

    * **Frame mode** (default): ``image_index`` or ``start``/``stop`` selects
      color_image frames; the image author proposes the single-frame family
      bucket (presence, distance, …).
    * **Cross-view mode**: non-empty ``queries`` runs Dan localize and emits
      ``cross_view_same_object`` pairs. Frame-index fields must be omitted.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    dataset: NonEmptyString
    output: Path | None = None
    image_index: int | None = Field(default=None, ge=0)
    start: int | None = Field(default=None, ge=0)
    stop: int | None = Field(default=None, gt=0)
    stride: int | None = Field(default=None, ge=1)
    quality_window: float | None = Field(default=None, gt=0)
    min_brightness: float | None = Field(default=None, ge=0, le=1)
    with_localize: bool = False
    localize_queries: tuple[NonEmptyString, ...] = ()
    localize_pad_s: float = Field(default=10.0, gt=0)
    # None = default K when with_localize; 0 = disable coverage budget (publish all).
    localize_coverage_k: int | None = Field(default=None, ge=0)
    queries: tuple[NonEmptyString, ...] = ()
    from_s: float = Field(default=0.0, ge=0.0)
    duration_s: float | None = Field(default=None, gt=0.0)
    params: str = "go2"
    min_views: int = Field(default=3, ge=1)

    @property
    def cross_view(self) -> bool:
        return bool(self.queries)

    @model_validator(mode="after")
    def valid_selection(self) -> GenerationRequest:
        frame_fields = (
            self.image_index is not None
            or self.start is not None
            or self.stop is not None
            or self.stride is not None
            or self.quality_window is not None
            or self.min_brightness is not None
        )
        if self.queries:
            if frame_fields:
                raise ValueError(
                    "cross-view generation (--query) cannot combine with "
                    "image_index/start/stop/stride/quality_window"
                )
            if self.with_localize or self.localize_queries or self.localize_coverage_k is not None:
                raise ValueError("cross-view generation (--query) cannot combine with --with-localize")
            return self
        if self.from_s != 0.0 or self.duration_s is not None:
            raise ValueError("from_s/duration_s require cross-view mode (--query)")
        if self.localize_queries and not self.with_localize:
            raise ValueError("localize_queries requires with_localize=True")
        if self.localize_coverage_k is not None and not self.with_localize:
            raise ValueError("localize_coverage_k requires with_localize=True")
        if self.image_index is not None:
            if self.start is not None or self.stop is not None or self.stride is not None:
                raise ValueError("image_index cannot be combined with start, stop, or stride")
            if self.quality_window is not None or self.min_brightness is not None:
                raise ValueError(
                    "quality_window/min_brightness require range mode (start/stop), not image_index"
                )
            return self
        if self.start is None or self.stop is None:
            raise ValueError("provide image_index, start/stop, or --query for cross-view")
        if self.stop <= self.start:
            raise ValueError("stop must be greater than start")
        if self.quality_window is not None and self.stride is not None:
            raise ValueError("quality_window cannot be combined with stride")
        if self.min_brightness is not None and self.quality_window is None:
            raise ValueError("min_brightness requires quality_window")
        return self

    def frame_indices(self) -> range:
        if self.cross_view:
            raise ValueError("frame_indices is only valid in frame mode")
        if self.image_index is not None:
            return range(self.image_index, self.image_index + 1)
        assert self.start is not None and self.stop is not None
        return range(self.start, self.stop, self.stride or 1)

    def output_directory(self) -> Path:
        if self.output is not None:
            return self.output.expanduser()
        suffix = "cross-view" if self.cross_view else "frames"
        return STATE_DIR / "datasets" / "vqa" / f"{Path(self.dataset).stem}-{suffix}"


def resolve_generation_frame_indices(
    request: GenerationRequest,
    image_count: int,
) -> tuple[int, ...]:
    """Resolve uniform color_image indices (no quality window)."""
    if request.quality_window is not None:
        raise ValueError("quality_window selection requires PointCloudFrameLoader")
    anchors = tuple(request.frame_indices())
    if not anchors:
        raise ValueError("generation selected no frames")
    if anchors[-1] >= image_count:
        raise IndexError(
            f"color_image index {anchors[-1]} is out of range for {image_count} images"
        )
    return anchors


class VqaGenerationConfig(BaseModel):
    """Processing policy shared by VQA generation runs."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    synchronization_tolerance_s: float = Field(default=0.1, gt=0)


class PublicCase(BaseModel):
    """One evaluator-facing multiple-choice VQA case.

    Exactly one visual modality: ``image`` (single-frame), ``clip`` (ego memory),
    or ``clips`` (cross-view short dense pair).
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    id: NonEmptyString
    question: NonEmptyString
    choices: tuple[NonEmptyString, ...]
    image: NonEmptyString | None = None
    clip: NonEmptyString | None = None
    clips: tuple[NonEmptyString, ...] | None = None

    @model_validator(mode="after")
    def choices_and_modality(self) -> PublicCase:
        unique_choices = {choice.casefold() for choice in self.choices}
        if len(self.choices) < 2 or len(unique_choices) != len(self.choices):
            raise ValueError("a VQA case requires at least two unique choices")
        modalities = sum(
            (
                self.image is not None,
                self.clip is not None,
                self.clips is not None,
            )
        )
        if modalities != 1:
            raise ValueError("a VQA case requires exactly one of image, clip, or clips")
        if self.clips is not None and len(self.clips) < 2:
            raise ValueError("clips requires at least two clip directories")
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
    """One indexed image with optional point-cloud or localize evidence."""

    index: int
    image: Image
    pointcloud_frame: PointCloudFrame | None = None
    localize_scene: LocalizeSceneEvidence | None = None


@dataclass(frozen=True)
class _GeneratedFrame:
    """Per-frame generation result. Image bytes are written immediately and not retained."""

    index: int
    image_ts: float
    cases: tuple[PublicCase, ...]
    labels: tuple[PrivateLabel, ...]
    audit_rows: tuple[dict[str, object], ...]
    families: tuple[FamilySpec, ...]


def generate_dataset(
    request: GenerationRequest,
    config: VqaGenerationConfig | None = None,
) -> GenerationResult:
    """Generate a standalone dataset from a Memory recording.

    Dispatches to the single-frame family bucket or ``cross_view_same_object``
    when ``request.queries`` is set.
    """
    if request.cross_view:
        return _generate_cross_view_dataset(request)

    # Load optional model dependencies only when generation is requested.
    from dimos.models.vl.moondream import MoondreamVlModel
    from dimos.models.vl.openai import OpenAIVlModel

    config = config or VqaGenerationConfig()
    with PointCloudFrameLoader(
        request.dataset,
        tolerance_s=config.synchronization_tolerance_s,
    ) as loader:
        if request.quality_window is not None:
            assert request.start is not None and request.stop is not None
            indices = select_quality_indices(
                loader.images,
                request.start,
                request.stop,
                quality_window=request.quality_window,
                min_brightness=request.min_brightness,
            )
        else:
            indices = resolve_generation_frame_indices(request, loader.image_count)

        localize_ctx: AbstractContextManager[LocalizeWindowResult | None] = nullcontext(None)
        if request.with_localize:
            from dimos.evals.vqa.localize_recording import localize_window

            timestamps = tuple(
                float(loader.images.offset(index).limit(1).first().ts) for index in indices
            )
            after = min(timestamps) - request.localize_pad_s
            before = max(timestamps) + request.localize_pad_s
            localize_ctx = localize_window(request.dataset, request.localize_queries, after, before)

        with localize_ctx as localize_result:
            author_model = OpenAIVlModel()
            detector_model = MoondreamVlModel()
            try:
                detector = detector_model
                model_names = {
                    "author": author_model.config.model_name,
                    "detector": detector_model.config.model_name,
                }
                if request.with_localize:
                    model_names["localize"] = "dandetect"
                    if localize_result is not None:
                        model_names["localize_queries"] = ", ".join(localize_result.queries)
                        model_names["localize_hits"] = str(len(localize_result.named_hits))
                mask_estimator = EdgeTAMObjectMaskPipeline(detector)

                def pointcloud_frames() -> Iterable[GenerationFrame]:
                    from dimos.evals.vqa.localize_recording import scene_at_timestamp

                    for index in indices:
                        try:
                            pointcloud_frame = loader.load(index)
                            image = pointcloud_frame.image
                            pc_frame = pointcloud_frame
                        except PointCloudFrameUnavailableError:
                            image = loader.load_image(index)
                            pc_frame = None
                        localize_scene = None
                        if localize_result is not None:
                            localize_scene = scene_at_timestamp(
                                localize_result.named_hits,
                                localize_result.rig,
                                image.ts,
                                first_seen_by_instance=getattr(
                                    localize_result, "first_seen_by_instance", {}
                                ),
                            )
                        yield GenerationFrame(index, image, pc_frame, localize_scene=localize_scene)

                color_frames = (
                    getattr(localize_result, "color_frames", ())
                    if localize_result is not None
                    else ()
                )
                return generate_frames_dataset(
                    request,
                    pointcloud_frames(),
                    OpenAIQuestionAuthor(author_model),
                    detector,
                    LidarRangeEstimator(mask_estimator),
                    mask_estimator,
                    config=config,
                    model_names=model_names,
                    color_frames=color_frames,
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
    color_frames: Sequence[tuple[float, Image]] = (),
) -> GenerationResult:
    """Generate and write one dataset from already loaded indexed images."""
    config = config or VqaGenerationConfig()
    output = request.output_directory()
    _prepare_output(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        with tempfile.TemporaryDirectory(prefix=f".{output.name}-", dir=output.parent) as temporary:
            staging = Path(temporary)
            all_cases: list[PublicCase] = []
            all_labels: list[PrivateLabel] = []
            generated_frames: list[_GeneratedFrame] = []
            frame_count = 0
            rejected_count = 0
            used_family_names: set[str] = set()
            for source in frames:
                frame, image = _generate_frame(
                    source, author, detector, range_estimator, mask_estimator
                )
                _write_frame(staging, frame, image)
                generated_frames.append(frame)
                all_cases.extend(frame.cases)
                all_labels.extend(frame.labels)
                frame_count += 1
                used_family_names.update(family.name for family in frame.families)
                rejected_count += sum(row.get("status") == "rejected" for row in frame.audit_rows)

            if frame_count == 0:
                raise ValueError("generation selected no frames")
            if not all_cases:
                raise ValueError("no selected frame produced an answerable question")

            coverage_k = _effective_localize_coverage_k(request)
            coverage_stats: dict[str, object] | None = None
            if coverage_k is not None:
                all_cases, all_labels, coverage_stats = _apply_localize_coverage_budget(
                    generated_frames, all_cases, all_labels, k=coverage_k
                )
                if not all_cases:
                    raise ValueError("localize coverage budget removed every published case")

            if request.with_localize and color_frames:
                all_cases, all_labels = _materialize_ego_memory_clips(
                    staging,
                    generated_frames,
                    all_cases,
                    all_labels,
                    color_frames=color_frames,
                )

            _write_jsonl(
                staging / "cases.jsonl",
                [case.model_dump(mode="json", exclude_none=True) for case in all_cases],
            )
            _write_jsonl(
                staging / "labels.jsonl",
                [label.model_dump(mode="json") for label in all_labels],
            )
            run_payload: dict[str, object] = {
                "generation": request.model_dump(mode="json", exclude={"output"}),
                "configuration": config.model_dump(mode="json"),
                "families": [
                    family.name for family in AVAILABLE_FAMILIES if family.name in used_family_names
                ],
                "models": model_names or {},
                "frame_count": frame_count,
                "question_count": len(all_cases),
                "rejected_question_count": rejected_count,
            }
            if coverage_stats is not None:
                run_payload["localize_coverage"] = coverage_stats
            _write_json(staging / "audit" / "run.json", run_payload)
            if output.exists():
                output.rmdir()
            staging.replace(output)
        return GenerationResult(output=output, cases=tuple(all_cases))
    except Exception:
        if output.exists():
            shutil.rmtree(output)
        raise


def _generate_cross_view_dataset(request: GenerationRequest) -> GenerationResult:
    """Localize with traces → pair → write the same on-disk VQA layout."""
    from dimos.memory.cli.dataset import open_dataset
    from dimos.perception.localize.dandetect import DanDetector
    from dimos.perception.localize.localize import LocalizeTrace, embed_index
    from dimos.perception.localize.rig import Rig, window_tf
    from dimos.perception.localize.tools.tool_localize import GO2_PARAMS, PRESETS, parse_params

    output = request.output_directory()
    _prepare_output(output)

    base, fields = parse_params(request.params, PRESETS.get("go2", GO2_PARAMS))
    params = replace(base, **fields) if fields else base
    params = replace(params, min_views=request.min_views)

    store = open_dataset(request.dataset)
    store.start()
    try:
        rig = Rig.from_store(store)
        lo, hi = rig.time_range()
        after = lo + request.from_s
        before = lo + request.from_s + request.duration_s if request.duration_s is not None else hi
        if before <= after:
            raise ValueError("localize window before must be greater than after")
        rig.tf = window_tf(store, after, before)
        if rig.lidar is not None:
            rig.lidar = rig.lidar.after(after).before(before)

        queries = [str(q).strip() for q in request.queries if str(q).strip()]
        traces = [LocalizeTrace() for _ in queries]
        with DanDetector("siglip", "owlv2", "edgetam") as dan:
            embedded = embed_index(rig, dan.embedder, after, before)
            results = dan.localize(queries, index=embedded, rig=rig, params=params, traces=traces)

        all_pairs: list[IdentityPair] = []
        n_instances = 0
        for query, hits, trace in zip(queries, results, traces, strict=True):
            logger.info(f"cross-view {query!r}: {len(hits)} verified hit(s)")
            by_id = collect_query_instances(query, hits, list(trace.answers))
            n_instances += len(by_id)
            pairs = propose_identity_pairs(by_id, min_views=request.min_views)
            logger.info(f"cross-view {query!r}: {len(pairs)} pair proposal(s)")
            all_pairs.extend(pairs)
    finally:
        store.stop()

    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f".{output.name}-", dir=output.parent) as temporary:
        staging = Path(temporary)
        (staging / "assets").mkdir(parents=True, exist_ok=True)
        (staging / "audit").mkdir(parents=True, exist_ok=True)
        cases: list[PublicCase] = []
        labels: list[PrivateLabel] = []
        audit_rows: list[dict[str, object]] = []
        rejected = 0

        for pair_index, pair in enumerate(all_pairs):
            try:
                answered = answer_cross_view_same_object(pair)
            except InsufficientEvidenceError as exc:
                rejected += 1
                audit_rows.append(
                    {
                        "status": "rejected",
                        "reason": str(exc),
                        "query": pair.query,
                        "same_object": pair.same_object,
                    }
                )
                continue

            case_id = pair_case_id(pair, pair_index)
            image = stack_pair_image(pair.left.image, pair.right.image)
            rel = f"assets/{case_id}.jpg"
            image_path = staging / rel
            image_path.parent.mkdir(parents=True, exist_ok=True)
            if not image.save(str(image_path)):
                raise RuntimeError(f"failed to write cross-view image: {image_path}")
            cases.append(
                PublicCase(
                    id=case_id,
                    image=rel,
                    question=answered.question,
                    choices=answered.choices,
                )
            )
            labels.append(PrivateLabel(id=case_id, answer=answered.answer))
            audit_rows.append(
                {
                    "status": "answered",
                    "case_id": case_id,
                    "answer": answered.model_dump(mode="json"),
                }
            )

        if not cases:
            raise ValueError("cross-view generation produced no answerable pairs")

        _write_jsonl(
            staging / "cases.jsonl",
            [case.model_dump(mode="json") for case in cases],
        )
        _write_jsonl(
            staging / "labels.jsonl",
            [label.model_dump(mode="json") for label in labels],
        )
        _write_json(
            staging / "audit" / "run.json",
            {
                "generation": request.model_dump(mode="json", exclude={"output"}),
                "families": ["cross_view_same_object"],
                "n_instances_considered": n_instances,
                "n_pairs_proposed": len(all_pairs),
                "question_count": len(cases),
                "rejected_question_count": rejected,
            },
        )
        _write_json(staging / "audit" / "pairs.json", audit_rows)
        # Never delete a completed dataset from a concurrent writer.
        if output.exists() and any(output.iterdir()):
            raise FileExistsError(
                f"VQA output directory already exists after generation: {output}"
            )
        if output.exists():
            output.rmdir()
        shutil.move(str(staging), str(output))

    return GenerationResult(output=output, cases=tuple(cases))


def _generate_frame(
    source: GenerationFrame,
    author: QuestionAuthor,
    detector: VlModel,
    range_estimator: ObjectRangeEstimator | None,
    mask_estimator: ObjectMaskEstimator | None,
) -> tuple[_GeneratedFrame, Image]:
    image_index = source.index
    image = source.image
    # Empty verified-hit scenes must not expose localize families (author would propose
    # unresolvable referents; deterministic seeding would also be a no-op).
    localize_ready = source.localize_scene is not None and len(source.localize_scene.instances) > 0
    families = tuple(
        family
        for family in AVAILABLE_FAMILIES
        if family.name != "cross_view_same_object"
        and not (family.requires_masks and mask_estimator is None)
        and not (
            family.requires_pointcloud
            and (source.pointcloud_frame is None or range_estimator is None)
        )
        and not (family.requires_localize and not localize_ready)
    )
    proposals = author.propose(image, families)
    if localize_ready and any(family.requires_localize for family in families):
        from dimos.evals.vqa.localize_recording import proposals_from_localize_scene

        assert source.localize_scene is not None
        proposals = (
            *proposals,
            *proposals_from_localize_scene(source.localize_scene),
        )
    proposals = _deduplicate_proposals(proposals)
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
                source.localize_scene,
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
    return (
        _GeneratedFrame(
            index=image_index,
            image_ts=float(image.ts),
            cases=cases,
            labels=labels,
            audit_rows=tuple(audit_rows),
            families=tuple(families),
        ),
        image,
    )


_EGO_CLIP_FAMILIES = frozenset(
    family.name for family in AVAILABLE_FAMILIES if family.requires_localize
)
# First sighting → question time must fit in one short memory clip.
MAX_EGO_MEMORY_SPAN_S = 8.0
# Cone/FOV proxy: outside ahead cone at question time (hard memory band).
_HARD_EGO_FORE_AFT = frozenset({"behind"})
_HARD_EGO_TURN = frozenset({"left", "right", "turn around"})


def _ego_memory_band(family: str, answer: str) -> str | None:
    """Return ``hard`` / ``soft`` for publishable ego answers, else ``None``."""
    if family not in _EGO_CLIP_FAMILIES:
        return None
    if family == "ego_fore_aft":
        return "hard" if answer in _HARD_EGO_FORE_AFT else "soft"
    if family == "ego_turn":
        return "hard" if answer in _HARD_EGO_TURN else "soft"
    # ego_any_fore_aft, height_order, …
    return "soft"


_T = TypeVar("_T")


def _balance_ego_memory_bands(hard: Sequence[_T], soft: Sequence[_T]) -> list[_T]:
    """Keep hard:soft ≈ 1:1; if one side is empty, keep the other."""
    if not hard:
        return list(soft)
    if not soft:
        return list(hard)
    n = min(len(hard), len(soft))
    return list(hard[:n]) + list(soft[:n])


def _materialize_ego_memory_clips(
    staging: Path,
    frames: Sequence[_GeneratedFrame],
    cases: Sequence[PublicCase],
    labels: Sequence[PrivateLabel],
    *,
    color_frames: Sequence[tuple[float, Image]],
) -> tuple[list[PublicCase], list[PrivateLabel]]:
    """Publish ego cases as first-spot→query clips; balance hard/soft; no image fallback."""
    audit_by_id: dict[str, dict[str, object]] = {}
    frame_ts_by_id: dict[str, float] = {}
    for frame in frames:
        for case in frame.cases:
            frame_ts_by_id[case.id] = frame.image_ts
            family = case_family_from_id(case.id)
            if family not in _EGO_CLIP_FAMILIES:
                continue
            for row in frame.audit_rows:
                if row.get("status") != "answered":
                    continue
                answer = row.get("answer")
                if not isinstance(answer, dict):
                    continue
                if answer.get("question") == case.question:
                    audit_by_id[case.id] = answer
                    break

    label_by_id = {label.id: label for label in labels}
    non_ego_cases: list[PublicCase] = []
    non_ego_labels: list[PrivateLabel] = []
    # Defer disk write until after 1:1 hard/soft balance so unused clips are not left behind.
    EgoClipRow = tuple[PublicCase, PrivateLabel, tuple[tuple[float, Image], ...], float, float]
    hard: list[EgoClipRow] = []
    soft: list[EgoClipRow] = []

    for case in cases:
        family = case_family_from_id(case.id)
        if family not in _EGO_CLIP_FAMILIES:
            non_ego_cases.append(case)
            non_ego_labels.append(label_by_id[case.id])
            continue

        label = label_by_id[case.id]
        band = _ego_memory_band(family, label.answer)
        if band is None:
            continue

        query_ts = frame_ts_by_id.get(case.id)
        answered = audit_by_id.get(case.id)
        if query_ts is None or answered is None:
            # Ego must be memory-clip; never fall back to a single RGB frame.
            continue

        evidence = answered.get("evidence")
        first_spot = None
        if isinstance(evidence, dict):
            raw = evidence.get("first_seen_timestamp")
            if isinstance(raw, (int, float)):
                first_spot = float(raw)
        if first_spot is None:
            first_spot = query_ts
        if query_ts - first_spot > MAX_EGO_MEMORY_SPAN_S:
            continue

        try:
            picked, _t0 = ego_memory_clip(
                color_frames,
                first_spot_ts=first_spot,
                query_ts=query_ts,
            )
        except ValueError:
            continue
        if clip_duration_s(picked) > MAX_EGO_MEMORY_SPAN_S + 1e-6:
            continue

        rel = f"assets/clips/{case.id}"
        duration = clip_duration_s(picked)
        span = format_clip_span(duration)
        question = (
            f"This clip covers {span} (first sighting of the referent through the "
            f"question time). {case.question}"
        )
        clip_case = PublicCase(
            id=case.id,
            clip=rel,
            question=question,
            choices=case.choices,
        )
        bucket = hard if band == "hard" else soft
        bucket.append((clip_case, label, picked, first_spot, query_ts))

    balanced = _balance_ego_memory_bands(hard, soft)
    out_cases: list[PublicCase] = list(non_ego_cases)
    out_labels: list[PrivateLabel] = list(non_ego_labels)
    for clip_case, label, picked, first_spot, query_ts in balanced:
        assert clip_case.clip is not None
        write_image_clip(
            staging / clip_case.clip,
            picked,
            anchors=(first_spot, query_ts),
        )
        out_cases.append(clip_case)
        out_labels.append(label)
    return out_cases, out_labels


def _effective_localize_coverage_k(request: GenerationRequest) -> int | None:
    """Resolve publish budget: default K with localize, ``0`` disables, else explicit K."""
    if not request.with_localize:
        return None
    if request.localize_coverage_k is None:
        return DEFAULT_LOCALIZE_COVERAGE_K
    if request.localize_coverage_k == 0:
        return None
    return request.localize_coverage_k


def _apply_localize_coverage_budget(
    frames: Sequence[_GeneratedFrame],
    cases: Sequence[PublicCase],
    labels: Sequence[PrivateLabel],
    *,
    k: int,
) -> tuple[list[PublicCase], list[PrivateLabel], dict[str, object]]:
    """Keep image-path cases; budget up to ``K`` localize cases per frame."""
    candidates: list[CoverageCandidate] = []
    localize_ids: set[str] = set()
    for frame in frames:
        for case in frame.cases:
            family = case_family_from_id(case.id)
            candidate = collect_coverage_candidates_from_audit(
                case_id=case.id,
                family=family,
                question=case.question,
                audit_rows=frame.audit_rows,
            )
            if candidate is None:
                continue
            candidates.append(candidate)
            localize_ids.add(case.id)

    kept_localize = select_localize_coverage_ids(candidates, k=k)
    kept_ids = {case.id for case in cases if case.id not in localize_ids} | kept_localize
    filtered_cases = [case for case in cases if case.id in kept_ids]
    filtered_labels = [label for label in labels if label.id in kept_ids]
    by_cell: dict[str, int] = {}
    by_category: dict[str, int] = {}
    by_frame: dict[str, int] = {}
    for case_id, cell, _pose, category in candidates:
        if case_id not in kept_localize:
            continue
        key = f"{cell[0]}|{cell[1]}"
        by_cell[key] = by_cell.get(key, 0) + 1
        by_category[category] = by_category.get(category, 0) + 1
        frame_key = case_frame_from_id(case_id)
        by_frame[frame_key] = by_frame.get(frame_key, 0) + 1
    stats: dict[str, object] = {
        "k": k,
        "k_meaning": "max_localize_cases_per_frame",
        "localize_before": len(localize_ids),
        "localize_after": len(kept_localize),
        "non_localize_kept": len(kept_ids) - len(kept_localize),
        "total_before": len(cases),
        "total_after": len(filtered_cases),
        "frames_with_localize": len(by_frame),
        "kept_per_cell": dict(sorted(by_cell.items())),
        "kept_per_category": dict(sorted(by_category.items())),
        "kept_per_frame": dict(sorted(by_frame.items())),
    }
    return filtered_cases, filtered_labels, stats


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
    """Claim ``output`` exclusively so concurrent runs cannot share the path."""
    try:
        output.mkdir(parents=True, exist_ok=False)
    except FileExistsError as exc:
        detail = "is not empty" if output.is_dir() and any(output.iterdir()) else "already exists"
        raise FileExistsError(f"VQA output directory {detail}: {output}") from exc


def _write_frame(output: Path, frame: _GeneratedFrame, image: Image) -> None:
    (output / "assets").mkdir(parents=True, exist_ok=True)
    frame_audit = output / "audit" / f"frame-{frame.index:06d}"
    frame_audit.mkdir(parents=True, exist_ok=True)

    image_name = f"assets/frame-{frame.index:06d}.png"
    image_path = output / image_name
    if not image.save(str(image_path)):
        raise RuntimeError(f"failed to write VQA image: {image_path}")

    case_rows = [case.model_dump(mode="json", exclude_none=True) for case in frame.cases]
    label_rows = [label.model_dump(mode="json") for label in frame.labels]
    _write_json(frame_audit / "cases.json", case_rows)
    _write_json(frame_audit / "labels.json", label_rows)
    _write_json(frame_audit / "ground_truth.json", frame.audit_rows)
    _write_json(
        frame_audit / "frame.json",
        {
            "frame_index": frame.index,
            "image": image_name,
            "timestamp": frame.image_ts,
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
