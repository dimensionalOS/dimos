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

"""Deterministic VQA proposals from open-vocab detections (no LLM author, no YOLO)."""

from __future__ import annotations

from collections import Counter
from collections.abc import Mapping, Sequence
from typing import Any, Protocol

import torch

from dimos.evals.vqa.contracts import FamilySpec, QuestionProposal
from dimos.evals.vqa.primitives.cross_view_identity import SightingView
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.detectors.owlv2 import Owlv2Detector
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

# Cap how many image-family proposals one frame emits before diversity filtering.
_MAX_NAMES_PER_FRAME = 8
_MAX_LARGEST_PAIRS = 4
_MAX_LARGEST_TRIPLES = 2


def _normalize_name(name: str) -> str:
    return " ".join(name.strip().casefold().split())


def names_from_detections(detections: ImageDetections2D[Any]) -> tuple[str, ...]:
    """Unique detection names, most frequent first, then alphabetical."""
    counts: Counter[str] = Counter()
    display: dict[str, str] = {}
    for det in detections.detections:
        raw = str(getattr(det, "name", "") or "").strip()
        if not raw:
            continue
        key = _normalize_name(raw)
        counts[key] += 1
        display.setdefault(key, raw)
    ranked = sorted(counts.items(), key=lambda row: (-row[1], row[0]))
    return tuple(display[key] for key, _ in ranked[:_MAX_NAMES_PER_FRAME])


def proposals_from_detected_names(
    names: Sequence[str],
    families: Sequence[FamilySpec],
) -> tuple[QuestionProposal, ...]:
    """Fill family templates from detected object names (no VLM)."""
    available = {family.name: family for family in families}
    cleaned = [name.strip() for name in names if name.strip()]
    if not cleaned:
        return ()

    out: list[QuestionProposal] = []

    def _add(family: str, object_names: tuple[str, ...]) -> None:
        spec = available.get(family)
        if spec is None:
            return
        proposal = QuestionProposal(family=family, object_names=object_names)
        try:
            spec.validate(proposal)
        except ValueError:
            return
        out.append(proposal)

    for name in cleaned:
        _add("presence", (name,))
        _add("object_count", (name,))
        _add("horizontal_direction", (name,))
        _add("image_coverage", (name,))
        _add("object_distance", (name,))

    # Pairwise / triple comparisons among top names.
    top = cleaned[:5]
    pair_budget = 0
    for i, left in enumerate(top):
        for right in top[i + 1 :]:
            if _normalize_name(left) == _normalize_name(right):
                continue
            _add("largest_visible_area", (left, right))
            _add("closest_object", (left, right))
            pair_budget += 1
            if pair_budget >= _MAX_LARGEST_PAIRS:
                break
        if pair_budget >= _MAX_LARGEST_PAIRS:
            break

    triple_budget = 0
    for i, a in enumerate(top):
        for j, b in enumerate(top[i + 1 :], start=i + 1):
            for c in top[j + 1 :]:
                keys = {_normalize_name(a), _normalize_name(b), _normalize_name(c)}
                if len(keys) < 3:
                    continue
                _add("largest_visible_area", (a, b, c))
                _add("closest_object", (a, b, c))
                triple_budget += 1
                if triple_budget >= _MAX_LARGEST_TRIPLES:
                    return tuple(out)
    return tuple(out)


class OpenVocabDetectionCatalog:
    """OWLv2 names + boxes for templates (no YOLO-E).

    Prefer a fixed ``queries`` list from Dan/VLM discovery. Pass ``detector`` to
    reuse the warm OWL from ``localize_window`` (ownership transfers here).
    Without queries, ``names_for`` falls back to a per-frame OpenAI JSON list.
    """

    def __init__(
        self,
        queries: Sequence[str] = (),
        *,
        candidate_floor: float = 0.18,
        detector: Any | None = None,
    ) -> None:
        self._queries = tuple(query.strip() for query in queries if query.strip())
        self._floor = float(candidate_floor)
        # Created or handed off — ``stop()`` always releases this instance.
        self._detector = detector if detector is not None else Owlv2Detector(dtype=torch.float16)
        self._cache_key: tuple[str, float, tuple[str, ...]] | None = None
        self._cache: ImageDetections2D[Any] | None = None
        self._vl_model: Any | None = None
        reused = detector is not None
        self.config = type(
            "config",
            (),
            {
                "model_name": (
                    "google/owlv2-base-patch16-ensemble+handoff"
                    if reused
                    else "google/owlv2-base-patch16-ensemble"
                )
            },
        )()

    def stop(self) -> None:
        if self._vl_model is not None:
            stop = getattr(self._vl_model, "stop", None)
            if callable(stop):
                stop()
            self._vl_model = None
        self._detector.stop()
        self._cache_key = None
        self._cache = None

    def _vl_names(self, image: Image) -> tuple[str, ...]:
        from dimos.evals.vqa.localize_recording import (
            _DISCOVERY_VLM_BAN,
            _DISCOVERY_VLM_PROMPT,
        )
        from dimos.models.vl.openai import OpenAIVlModel

        if self._vl_model is None:
            self._vl_model = OpenAIVlModel()
        payload = self._vl_model.query_json(image, _DISCOVERY_VLM_PROMPT)
        if isinstance(payload, dict):
            raw = payload.get("objects", payload.get("names", []))
            items = list(raw) if isinstance(raw, list) else []
        elif isinstance(payload, list):
            items = payload
        else:
            items = []
        names: list[str] = []
        seen: set[str] = set()
        for item in items:
            if isinstance(item, dict):
                name = str(item.get("name", "") or "").strip()
            else:
                name = str(item or "").strip()
            key = _normalize_name(name)
            if not name or key in _DISCOVERY_VLM_BAN or key in seen:
                continue
            seen.add(key)
            names.append(name)
            if len(names) >= _MAX_NAMES_PER_FRAME:
                break
        return tuple(names)

    def _detections_for(self, image: Image, queries: Sequence[str]) -> ImageDetections2D[Any]:
        cleaned = tuple(query.strip() for query in queries if query.strip())
        key = (str(getattr(image, "frame_id", "") or ""), float(image.ts), cleaned)
        if self._cache_key == key and self._cache is not None:
            return self._cache
        if not cleaned:
            empty = ImageDetections2D(image=image, detections=[])
            self._cache_key = key
            self._cache = empty
            return empty

        boxes, rows = self._detector.query_score_rows(image, list(cleaned), threshold=self._floor)
        detections: list[Detection2DBBox] = []
        if rows.size:
            for j, name in enumerate(cleaned):
                keep = rows[:, j] >= self._floor
                for box, score in zip(boxes[keep], rows[keep, j], strict=True):
                    det = Detection2DBBox(
                        bbox=(float(box[0]), float(box[1]), float(box[2]), float(box[3])),
                        track_id=len(detections),
                        class_id=j,
                        confidence=float(score),
                        name=name,
                        ts=image.ts,
                        image=image,
                    )
                    if det.is_valid():
                        detections.append(det)
        detected = ImageDetections2D(image=image, detections=detections)
        self._cache_key = key
        self._cache = detected
        return detected

    def names_for(self, image: Image) -> tuple[str, ...]:
        queries = self._queries if self._queries else self._vl_names(image)
        return names_from_detections(self._detections_for(image, queries))

    def query_detections(
        self, image: Image, query: str, **_kwargs: Any
    ) -> ImageDetections2D[Detection2DBBox]:
        """Score ``query`` with OWLv2 and return matching boxes."""
        want = _normalize_name(query)
        if not want:
            return ImageDetections2D(image=image, detections=[])
        # Prefer the fixed vocab hit when the query matches a discovery name;
        # otherwise score this string alone.
        if self._queries and any(_normalize_name(q) == want for q in self._queries):
            source = self._detections_for(image, self._queries)
        else:
            source = self._detections_for(image, (query.strip(),))
        kept = [
            det
            for det in source.detections
            if isinstance(det, Detection2DBBox)
            and det.is_valid()
            and _normalize_name(str(det.name)) == want
        ]
        return ImageDetections2D(image=image, detections=kept)


class NameCatalog(Protocol):
    def names_for(self, image: Image) -> tuple[str, ...]: ...


class LocalizeSightingCatalog:
    """Image-family names/boxes from Dan sightings — no second OWL pass.

    A eval frame only gets proposals for queries that have a sighting near its
    timestamp. ``query_detections`` returns those stored boxes (EdgeTAM can
    still segment from them).
    """

    def __init__(
        self,
        views_by_query: Mapping[str, Mapping[str, Sequence[SightingView]]],
        *,
        ts_tolerance_s: float = 0.25,
    ) -> None:
        if ts_tolerance_s < 0:
            raise ValueError("ts_tolerance_s must be non-negative")
        self._views = views_by_query
        self._tol = float(ts_tolerance_s)
        self.config = type("config", (), {"model_name": "localize-sightings"})()

    def stop(self) -> None:
        return

    def _sightings_near(self, image: Image, query: str | None = None) -> list[SightingView]:
        want = _normalize_name(query) if query is not None else None
        ts = float(image.ts)
        out: list[SightingView] = []
        for name, by_id in self._views.items():
            if want is not None and _normalize_name(name) != want:
                continue
            for views in by_id.values():
                for view in views:
                    if abs(float(view.timestamp) - ts) <= self._tol:
                        out.append(view)
        return out

    def names_for(self, image: Image) -> tuple[str, ...]:
        counts: Counter[str] = Counter()
        display: dict[str, str] = {}
        for view in self._sightings_near(image):
            key = _normalize_name(view.query)
            if not key:
                continue
            counts[key] += 1
            display.setdefault(key, view.query)
        ranked = sorted(counts.items(), key=lambda row: (-row[1], row[0]))
        return tuple(display[key] for key, _ in ranked[:_MAX_NAMES_PER_FRAME])

    def query_detections(
        self, image: Image, query: str, **_kwargs: Any
    ) -> ImageDetections2D[Detection2DBBox]:
        """Return Dan boxes for ``query`` on (or near) this frame's timestamp."""
        detections: list[Detection2DBBox] = []
        for view in self._sightings_near(image, query=query):
            x1, y1, x2, y2 = (float(v) for v in view.bbox)
            det = Detection2DBBox(
                bbox=(x1, y1, x2, y2),
                track_id=len(detections),
                class_id=0,
                confidence=float(view.confidence),
                name=view.query,
                ts=image.ts,
                image=image,
            )
            if det.is_valid():
                detections.append(det)
        return ImageDetections2D(image=image, detections=detections)


class TemplateDetectionAuthor:
    """Propose family inputs from a name catalog (template slots only)."""

    def __init__(self, catalog: NameCatalog) -> None:
        self._catalog = catalog

    def propose(self, image: Image, families: Sequence[FamilySpec]) -> Sequence[QuestionProposal]:
        return proposals_from_detected_names(self._catalog.names_for(image), families)
