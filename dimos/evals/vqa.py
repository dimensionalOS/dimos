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

"""Source-neutral VQA scene facts, question families, and quality gates."""

from __future__ import annotations

from collections import Counter
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from itertools import combinations
import re
from typing import Any, Literal

import numpy as np

AnswerType = Literal["yes_no", "choice"]
QuestionFamily = Literal["semantic_presence", "spatial_left_right"]
SUPPORTED_QUESTION_FAMILIES: frozenset[str] = frozenset({"semantic_presence", "spatial_left_right"})


def normalize_category(value: str) -> str:
    """Return the stable identifier used in specs, snapshots, and manifests."""
    normalized = re.sub(r"[^a-z0-9]+", "_", value.strip().lower()).strip("_")
    if not normalized:
        raise ValueError("entity category must not be empty")
    return normalized


def humanize_category(value: str) -> str:
    return normalize_category(value).replace("_", " ")


@dataclass(frozen=True, kw_only=True)
class SceneEntity:
    """One simulator entity and its visibility in the exported RGB frame."""

    id: str
    category: str
    pixel_area: int
    centroid: tuple[float, float] | None = None
    bbox: tuple[int, int, int, int] | None = None  # x1, y1, x2, y2; inclusive

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise ValueError("entity id must not be empty")
        object.__setattr__(self, "category", normalize_category(self.category))
        if self.pixel_area < 0:
            raise ValueError("pixel_area must be non-negative")
        if self.pixel_area == 0 and (self.centroid is not None or self.bbox is not None):
            raise ValueError("invisible entities must not have a centroid or bbox")
        if self.pixel_area > 0 and (self.centroid is None or self.bbox is None):
            raise ValueError("visible entities require a centroid and bbox")
        if self.bbox is not None:
            x1, y1, x2, y2 = self.bbox
            if x1 > x2 or y1 > y2:
                raise ValueError("entity bbox bounds are inverted")

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> SceneEntity:
        centroid = value.get("centroid")
        bbox = value.get("bbox")
        return cls(
            id=str(value["id"]),
            category=str(value["category"]),
            pixel_area=int(value["pixel_area"]),
            centroid=(float(centroid[0]), float(centroid[1])) if centroid is not None else None,
            bbox=tuple(int(v) for v in bbox) if bbox is not None else None,  # type: ignore[arg-type]
        )


@dataclass(frozen=True, kw_only=True)
class SceneSnapshot:
    """One RGB observation plus private facts used only to derive labels."""

    image: np.ndarray[Any, Any]
    entities: tuple[SceneEntity, ...]
    provenance: Mapping[str, Any]

    def __post_init__(self) -> None:
        if self.image.dtype != np.uint8 or self.image.ndim != 3 or self.image.shape[2] != 3:
            raise ValueError("snapshot image must be an HxWx3 uint8 RGB array")
        if self.image.shape[0] == 0 or self.image.shape[1] == 0:
            raise ValueError("snapshot image must not be empty")
        ids = [entity.id for entity in self.entities]
        if len(ids) != len(set(ids)):
            raise ValueError("snapshot entity ids must be unique")

    @classmethod
    def from_mapping(
        cls, value: Mapping[str, Any], *, image: np.ndarray[Any, Any]
    ) -> SceneSnapshot:
        raw_entities = value.get("entities")
        if not isinstance(raw_entities, list):
            raise TypeError("snapshot entities must be a list")
        provenance = value.get("provenance", {})
        if not isinstance(provenance, dict):
            raise TypeError("snapshot provenance must be an object")
        return cls(
            image=image,
            entities=tuple(SceneEntity.from_mapping(entity) for entity in raw_entities),
            provenance=provenance,
        )


@dataclass(frozen=True, kw_only=True)
class GeneratedQuestion:
    inputs: str
    reference_outputs: str
    answer_type: AnswerType
    family: QuestionFamily
    oracle: Mapping[str, str]


@dataclass(frozen=True, kw_only=True)
class QuestionBatch:
    questions: tuple[GeneratedQuestion, ...]
    rejected: Mapping[str, int]


def generate_questions(
    snapshot: SceneSnapshot,
    *,
    targets: Sequence[str],
    families: Sequence[str],
    min_visible_pixels: int,
    min_horizontal_separation: float,
    max_bbox_overlap: float,
    max_spatial_pairs: int,
) -> QuestionBatch:
    """Generate deterministic, image-answerable questions from one snapshot."""
    if min_visible_pixels <= 0:
        raise ValueError("min_visible_pixels must be positive")
    if not 0 < min_horizontal_separation <= 1:
        raise ValueError("min_horizontal_separation must be in (0, 1]")
    if not 0 <= max_bbox_overlap <= 1:
        raise ValueError("max_bbox_overlap must be in [0, 1]")
    if max_spatial_pairs <= 0:
        raise ValueError("max_spatial_pairs must be positive")

    unknown = set(families) - SUPPORTED_QUESTION_FAMILIES
    if unknown:
        raise ValueError(f"unsupported question families: {sorted(unknown)}")

    questions: list[GeneratedQuestion] = []
    rejected: Counter[str] = Counter()
    normalized_targets = tuple(dict.fromkeys(normalize_category(target) for target in targets))

    if "semantic_presence" in families:
        questions.extend(
            _presence_questions(
                snapshot,
                targets=normalized_targets,
                min_visible_pixels=min_visible_pixels,
                rejected=rejected,
            )
        )
    if "spatial_left_right" in families:
        questions.extend(
            _left_right_questions(
                snapshot,
                targets=frozenset(normalized_targets),
                min_visible_pixels=min_visible_pixels,
                min_horizontal_separation=min_horizontal_separation,
                max_bbox_overlap=max_bbox_overlap,
                max_pairs=max_spatial_pairs,
                rejected=rejected,
            )
        )
    return QuestionBatch(questions=tuple(questions), rejected=dict(rejected))


def _presence_questions(
    snapshot: SceneSnapshot,
    *,
    targets: Sequence[str],
    min_visible_pixels: int,
    rejected: Counter[str],
) -> list[GeneratedQuestion]:
    by_category: dict[str, list[SceneEntity]] = {}
    for entity in snapshot.entities:
        by_category.setdefault(entity.category, []).append(entity)

    questions: list[GeneratedQuestion] = []
    for target in targets:
        areas = [entity.pixel_area for entity in by_category.get(target, [])]
        if areas and 0 < max(areas) < min_visible_pixels:
            rejected["presence_too_small"] += 1
            continue
        answer = "yes" if any(area >= min_visible_pixels for area in areas) else "no"
        questions.append(
            GeneratedQuestion(
                inputs=f"Is there a {humanize_category(target)} visible in the image?",
                reference_outputs=answer,
                answer_type="yes_no",
                family="semantic_presence",
                oracle={"family": "semantic_presence", "category": target},
            )
        )
    return questions


def _left_right_questions(
    snapshot: SceneSnapshot,
    *,
    targets: frozenset[str],
    min_visible_pixels: int,
    min_horizontal_separation: float,
    max_bbox_overlap: float,
    max_pairs: int,
    rejected: Counter[str],
) -> list[GeneratedQuestion]:
    eligible = [
        entity
        for entity in snapshot.entities
        if entity.category in targets and entity.pixel_area >= min_visible_pixels
    ]
    category_counts = Counter(entity.category for entity in eligible)
    eligible = [entity for entity in eligible if category_counts[entity.category] == 1]
    ambiguous = sum(count for count in category_counts.values() if count > 1)
    if ambiguous:
        rejected["spatial_ambiguous_label"] += ambiguous

    width = float(snapshot.image.shape[1])
    questions: list[GeneratedQuestion] = []
    pairs = tuple(combinations(sorted(eligible, key=lambda entity: entity.id), 2))
    for pair_index, (subject, reference) in enumerate(pairs):
        assert subject.centroid is not None and reference.centroid is not None
        if abs(subject.centroid[0] - reference.centroid[0]) / width < min_horizontal_separation:
            rejected["spatial_too_close"] += 1
            continue
        assert subject.bbox is not None and reference.bbox is not None
        if _bbox_overlap(subject.bbox, reference.bbox) > max_bbox_overlap:
            rejected["spatial_overlap"] += 1
            continue
        answer = "left" if subject.centroid[0] < reference.centroid[0] else "right"
        questions.append(
            GeneratedQuestion(
                inputs=(
                    f"Is the {humanize_category(subject.category)} to the left or right "
                    f"of the {humanize_category(reference.category)}?"
                ),
                reference_outputs=answer,
                answer_type="choice",
                family="spatial_left_right",
                oracle={
                    "family": "spatial_left_right",
                    "subject": subject.id,
                    "reference": reference.id,
                },
            )
        )
        if len(questions) == max_pairs:
            remaining = len(pairs) - pair_index - 1
            if remaining > 0:
                rejected["spatial_pair_limit"] += remaining
            break
    return questions


def _bbox_overlap(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    intersection_w = max(0, min(ax2, bx2) - max(ax1, bx1) + 1)
    intersection_h = max(0, min(ay2, by2) - max(ay1, by1) + 1)
    intersection = intersection_w * intersection_h
    area_a = (ax2 - ax1 + 1) * (ay2 - ay1 + 1)
    area_b = (bx2 - bx1 + 1) * (by2 - by1 + 1)
    return intersection / min(area_a, area_b)
