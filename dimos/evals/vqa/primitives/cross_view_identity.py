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

"""Cross-view same-object identity pairs from localize sightings.

Keeps original full-frame images. Sightings that are too small, clipped, or
far from image center are rejected — no crop / privileged box for the agent.
"""

from __future__ import annotations

from dataclasses import dataclass
import itertools
import re
from typing import Any

from dimos.evals.vqa.contracts import FamilyAnswer, InsufficientEvidenceError
from dimos.msgs.sensor_msgs.Image import Image

# Framing gates (normalized image coords). Both views must pass.
# Slightly looser than desk-scale: Go2 room shots rarely park the referent dead-center.
CENTER_MAX_OFFSET = 0.35  # |cx-0.5| and |cy-0.5|
MIN_AREA_FRAC = 0.02
MAX_AREA_FRAC = 0.50
EDGE_MARGIN_FRAC = 0.02

# Instance pool: localize already verified the hit — only require enough views.
# No second semantic/confidence floor here (those fight LocalizeParams.accept_score).
MIN_INSTANCE_VIEWS = 3

# Pair sampling — prefer temporally separated revisits over near-duplicate views.
MAX_SAME_PAIRS_PER_INSTANCE = 1  # one first↔last revisit per instance
MAX_DIFF_PAIRS_PER_QUERY = 8
# Same-instance: revisit after leaving (not near-duplicate co-visible frames).
MIN_SAME_PAIR_TIME_GAP_S = 8.0
# Different-instance: only avoid the same full frame (co-visible A/B → ambiguous "no").
MIN_DIFF_PAIR_TIME_GAP_S = 1.0


@dataclass(frozen=True)
class SightingView:
    """One 2D observation belonging to a verified localize instance."""

    query: str
    instance_id: str
    image: Image
    bbox: tuple[float, float, float, float]
    confidence: float
    timestamp: float
    n_views: int
    semantic_score: float


@dataclass(frozen=True)
class IdentityPair:
    """Two full-frame views with a same/different ground-truth label."""

    query: str
    left: SightingView
    right: SightingView
    same_object: bool


def bbox_framing_ok(
    bbox: tuple[float, float, float, float],
    width: int,
    height: int,
    *,
    center_max_offset: float = CENTER_MAX_OFFSET,
    min_area_frac: float = MIN_AREA_FRAC,
    max_area_frac: float = MAX_AREA_FRAC,
    edge_margin_frac: float = EDGE_MARGIN_FRAC,
) -> bool:
    """True when the box is roughly centered, large enough, and not clipped."""
    if width < 1 or height < 1:
        return False
    x1, y1, x2, y2 = bbox
    if not (0.0 <= x1 < x2 and 0.0 <= y1 < y2):
        return False
    bw = x2 - x1
    bh = y2 - y1
    area_frac = (bw * bh) / float(width * height)
    if area_frac < min_area_frac or area_frac > max_area_frac:
        return False
    cx = ((x1 + x2) * 0.5) / width
    cy = ((y1 + y2) * 0.5) / height
    if abs(cx - 0.5) > center_max_offset or abs(cy - 0.5) > center_max_offset:
        return False
    mx = edge_margin_frac * width
    my = edge_margin_frac * height
    return not (x1 < mx or y1 < my or x2 > width - mx or y2 > height - my)


def sighting_framing_ok(view: SightingView) -> bool:
    rgb = view.image.to_rgb().data
    height, width = rgb.shape[:2]
    return bbox_framing_ok(view.bbox, width, height)


def instance_pool_ok(*, n_views: int, min_views: int = MIN_INSTANCE_VIEWS) -> bool:
    """True when localize reported enough distinct views (≥ ``min_views``, default 3)."""
    return n_views >= min_views


def views_from_localize(
    query: str,
    hit: Any,
    sightings: list[Any],
) -> tuple[SightingView, ...]:
    """Map one Localization + its Detection3DPC sightings into SightingView rows."""
    rows: list[SightingView] = []
    for det in sightings:
        image = det.image
        bbox = tuple(float(v) for v in det.bbox)
        if len(bbox) != 4:
            continue
        rows.append(
            SightingView(
                query=query,
                instance_id=str(hit.instance_id),
                image=image,
                bbox=(bbox[0], bbox[1], bbox[2], bbox[3]),
                confidence=float(getattr(det, "confidence", 0.0) or 0.0),
                timestamp=float(image.ts),
                n_views=int(hit.n_views),
                semantic_score=float(hit.semantic_score),
            )
        )
    return tuple(rows)


def collect_query_instances(
    query: str,
    hits: list[Any],
    answer_groups: list[Any],
) -> dict[str, tuple[SightingView, ...]]:
    """instance_id → SightingView tuple for one localize query."""
    if len(hits) != len(answer_groups):
        raise ValueError(
            f"localize trace length mismatch for {query!r}: "
            f"{len(hits)} hits vs {len(answer_groups)} answer groups"
        )
    by_id: dict[str, tuple[SightingView, ...]] = {}
    for hit, sightings in zip(hits, answer_groups, strict=True):
        views = views_from_localize(query, hit, list(sightings))
        if views:
            by_id[hit.instance_id] = views
    return by_id


def filter_instance_views(
    views: tuple[SightingView, ...],
    *,
    min_views: int = MIN_INSTANCE_VIEWS,
) -> tuple[SightingView, ...]:
    """Keep framing-OK sightings from a high-certainty instance."""
    if not views:
        return ()
    head = views[0]
    if not instance_pool_ok(n_views=head.n_views, min_views=min_views):
        return ()
    # Framing still picks agent-visible full frames; it is not an identity gate.
    return tuple(v for v in views if sighting_framing_ok(v))


def _pick_same_pairs(
    views: tuple[SightingView, ...],
    *,
    max_pairs: int = MAX_SAME_PAIRS_PER_INSTANCE,
    min_gap_s: float = MIN_SAME_PAIR_TIME_GAP_S,
) -> list[IdentityPair]:
    """Prefer first↔last revisit pairs (largest time gap) for same-instance identity."""
    ordered = sorted(views, key=lambda v: v.timestamp)
    candidates: list[tuple[float, SightingView, SightingView]] = []
    for left, right in itertools.combinations(ordered, 2):
        gap = abs(right.timestamp - left.timestamp)
        if gap < min_gap_s:
            continue
        candidates.append((gap, left, right))
    # Largest gap first: earliest sighting vs a later revisit.
    candidates.sort(key=lambda row: -row[0])
    pairs: list[IdentityPair] = []
    used: set[float] = set()
    for _gap, left, right in candidates:
        if len(pairs) >= max_pairs:
            break
        # Prefer spreading timestamps: skip if both already used heavily.
        key_l, key_r = left.timestamp, right.timestamp
        if key_l in used and key_r in used and len(pairs) > 0:
            continue
        pairs.append(IdentityPair(query=left.query, left=left, right=right, same_object=True))
        used.add(key_l)
        used.add(key_r)
    return pairs


def _pick_diff_pairs(
    by_instance: dict[str, tuple[SightingView, ...]],
    *,
    max_pairs: int = MAX_DIFF_PAIRS_PER_QUERY,
    min_gap_s: float = MIN_DIFF_PAIR_TIME_GAP_S,
) -> list[IdentityPair]:
    ids = sorted(by_instance)
    if len(ids) < 2:
        return []
    # Rank instances by semantic score, take best framing view each.
    ranked = sorted(
        ids,
        key=lambda i: (-by_instance[i][0].semantic_score, -by_instance[i][0].n_views, i),
    )
    pairs: list[IdentityPair] = []
    for a, b in itertools.combinations(ranked, 2):
        if len(pairs) >= max_pairs:
            break
        # Prefer distinct timestamps so A/B are not the same full frame (two
        # co-visible instances would otherwise look identical with answer "no").
        best: tuple[float, SightingView, SightingView] | None = None
        for left in by_instance[a]:
            for right in by_instance[b]:
                gap = abs(right.timestamp - left.timestamp)
                if gap < min_gap_s:
                    continue
                score = left.confidence + right.confidence
                if best is None or score > best[0]:
                    ordered = (left, right) if left.timestamp <= right.timestamp else (right, left)
                    best = (score, ordered[0], ordered[1])
        if best is None:
            continue
        pairs.append(
            IdentityPair(query=best[1].query, left=best[1], right=best[2], same_object=False)
        )
    return pairs


def propose_identity_pairs(
    instances: dict[str, tuple[SightingView, ...]],
    *,
    min_views: int = MIN_INSTANCE_VIEWS,
) -> tuple[IdentityPair, ...]:
    """Build same/different pairs for one query's filtered instance→views map."""
    kept: dict[str, tuple[SightingView, ...]] = {}
    for instance_id, views in instances.items():
        filtered = filter_instance_views(views, min_views=min_views)
        if len(filtered) >= 2:
            kept[instance_id] = filtered
        elif len(filtered) == 1:
            # Still usable as a different-pair endpoint.
            kept[instance_id] = filtered

    same: list[IdentityPair] = []
    for views in kept.values():
        if len(views) >= 2:
            same.extend(_pick_same_pairs(views))

    # Diff only among instances that have ≥1 framing-OK view.
    diff_pool = {i: v for i, v in kept.items() if v}
    different = _pick_diff_pairs(diff_pool)
    return tuple(same + different)


def answer_cross_view_same_object(
    pair: IdentityPair,
    *,
    first_clip_duration_s: float,
    second_clip_duration_s: float,
) -> FamilyAnswer:
    """Deterministic yes/no from instance_id equality (already encoded in pair).

    ``first_clip`` / ``second_clip`` are short dense MemoryStore streams; the
    question names each clip's relative ``[0, duration]`` span so the agent is
    not asked to search an unspecified long video.
    """
    if pair.left.query.casefold() != pair.right.query.casefold():
        raise InsufficientEvidenceError("cross-view pair requires the same category query")
    if not sighting_framing_ok(pair.left) or not sighting_framing_ok(pair.right):
        raise InsufficientEvidenceError("one or both views fail center/size framing gates")
    min_gap = MIN_SAME_PAIR_TIME_GAP_S if pair.same_object else MIN_DIFF_PAIR_TIME_GAP_S
    if abs(pair.right.timestamp - pair.left.timestamp) < min_gap:
        raise InsufficientEvidenceError(
            f"cross-view {'same' if pair.same_object else 'diff'} pair needs "
            f"≥{min_gap:g}s between views"
        )
    if pair.same_object and pair.left.instance_id != pair.right.instance_id:
        raise InsufficientEvidenceError("same-object pair has mismatched instance ids")
    if (not pair.same_object) and pair.left.instance_id == pair.right.instance_id:
        raise InsufficientEvidenceError("different-object pair has identical instance ids")
    if first_clip_duration_s < 0 or second_clip_duration_s < 0:
        raise InsufficientEvidenceError("cross-view clip durations must be non-negative")

    answer = "yes" if pair.same_object else "no"
    category = pair.query
    from dimos.evals.vqa.clip import format_clip_span

    first_span = format_clip_span(first_clip_duration_s)
    second_span = format_clip_span(second_clip_duration_s)
    return FamilyAnswer(
        question=(
            f"Stream first_clip covers {first_span} and stream second_clip covers "
            f"{second_span}. Is the {category} the same physical object instance in "
            f"both clips?"
        ),
        choices=("yes", "no"),
        answer=answer,
        evidence={
            "primitive": "cross_view_identity",
            "query": category,
            "same_object": pair.same_object,
            "left_instance_id": pair.left.instance_id,
            "right_instance_id": pair.right.instance_id,
            "left_bbox": list(pair.left.bbox),
            "right_bbox": list(pair.right.bbox),
            "left_confidence": pair.left.confidence,
            "right_confidence": pair.right.confidence,
            "left_ts": pair.left.timestamp,
            "right_ts": pair.right.timestamp,
            "semantic_score_left": pair.left.semantic_score,
            "semantic_score_right": pair.right.semantic_score,
            "n_views_left": pair.left.n_views,
            "n_views_right": pair.right.n_views,
            "first_clip_duration_s": first_clip_duration_s,
            "second_clip_duration_s": second_clip_duration_s,
            "center_max_offset": CENTER_MAX_OFFSET,
            "min_area_frac": MIN_AREA_FRAC,
        },
    )


def _path_token(value: str) -> str:
    """Filesystem-safe token for asset paths (no slashes or spaces)."""
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "-", value).strip("-._")
    return cleaned or "query"


def pair_case_id(pair: IdentityPair, index: int) -> str:
    kind = "same" if pair.same_object else "diff"
    q = _path_token(pair.query)
    left_id = _path_token(pair.left.instance_id[:8])
    right_id = _path_token(pair.right.instance_id[:8])
    return f"xview-{q}-{kind}-{index:04d}-{left_id}-{right_id}"
