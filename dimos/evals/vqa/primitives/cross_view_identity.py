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
from typing import Any

import numpy as np

from dimos.evals.vqa.contracts import FamilyAnswer, InsufficientEvidenceError
from dimos.msgs.sensor_msgs.Image import Image

# Framing gates (normalized image coords). Both views must pass.
CENTER_MAX_OFFSET = 0.25  # |cx-0.5| and |cy-0.5|
MIN_AREA_FRAC = 0.03
MAX_AREA_FRAC = 0.45
EDGE_MARGIN_FRAC = 0.02

# Instance pool gates (localize Localization + sighting confidences).
MIN_INSTANCE_VIEWS = 3
MIN_SEMANTIC_SCORE = 0.55
MIN_SIGHTING_CONF = 0.25

# Pair sampling.
MAX_SAME_PAIRS_PER_INSTANCE = 2
MAX_DIFF_PAIRS_PER_QUERY = 8
MIN_PAIR_TIME_GAP_S = 1.0


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
    if x1 < mx or y1 < my or x2 > width - mx or y2 > height - my:
        return False
    return True


def sighting_framing_ok(view: SightingView) -> bool:
    rgb = view.image.to_rgb().data
    height, width = rgb.shape[:2]
    return bbox_framing_ok(view.bbox, width, height)


def instance_pool_ok(
    *,
    n_views: int,
    semantic_score: float,
    sighting_confidences: tuple[float, ...],
    min_views: int = MIN_INSTANCE_VIEWS,
    min_score: float = MIN_SEMANTIC_SCORE,
    min_sighting_conf: float = MIN_SIGHTING_CONF,
) -> bool:
    """High-certainty instance gate before pairing."""
    if n_views < min_views or semantic_score < min_score:
        return False
    if not sighting_confidences:
        return False
    return min(sighting_confidences) >= min_sighting_conf


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


def filter_instance_views(views: tuple[SightingView, ...]) -> tuple[SightingView, ...]:
    """Keep framing-OK sightings from a high-certainty instance."""
    if not views:
        return ()
    head = views[0]
    if not instance_pool_ok(
        n_views=head.n_views,
        semantic_score=head.semantic_score,
        sighting_confidences=tuple(v.confidence for v in views),
    ):
        return ()
    return tuple(v for v in views if sighting_framing_ok(v))


def _pick_same_pairs(
    views: tuple[SightingView, ...],
    *,
    max_pairs: int = MAX_SAME_PAIRS_PER_INSTANCE,
    min_gap_s: float = MIN_PAIR_TIME_GAP_S,
) -> list[IdentityPair]:
    ordered = sorted(views, key=lambda v: v.timestamp)
    candidates: list[tuple[float, SightingView, SightingView]] = []
    for left, right in itertools.combinations(ordered, 2):
        gap = abs(right.timestamp - left.timestamp)
        if gap < min_gap_s:
            continue
        candidates.append((gap, left, right))
    candidates.sort(key=lambda row: -row[0])
    pairs: list[IdentityPair] = []
    used: set[float] = set()
    for gap, left, right in candidates:
        if len(pairs) >= max_pairs:
            break
        # Prefer spreading timestamps: skip if both already used heavily.
        key_l, key_r = left.timestamp, right.timestamp
        if key_l in used and key_r in used and len(pairs) > 0:
            continue
        pairs.append(
            IdentityPair(query=left.query, left=left, right=right, same_object=True)
        )
        used.add(key_l)
        used.add(key_r)
    return pairs


def _pick_diff_pairs(
    by_instance: dict[str, tuple[SightingView, ...]],
    *,
    max_pairs: int = MAX_DIFF_PAIRS_PER_QUERY,
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
        left = max(by_instance[a], key=lambda v: (v.confidence, -abs(v.timestamp)))
        right = max(by_instance[b], key=lambda v: (v.confidence, -abs(v.timestamp)))
        # Order by time for stable ids.
        if left.timestamp > right.timestamp:
            left, right = right, left
        pairs.append(
            IdentityPair(query=left.query, left=left, right=right, same_object=False)
        )
    return pairs


def propose_identity_pairs(
    instances: dict[str, tuple[SightingView, ...]],
) -> tuple[IdentityPair, ...]:
    """Build same/different pairs for one query's filtered instance→views map."""
    kept: dict[str, tuple[SightingView, ...]] = {}
    for instance_id, views in instances.items():
        filtered = filter_instance_views(views)
        if len(filtered) >= 2:
            kept[instance_id] = filtered
        elif len(filtered) == 1:
            # Still usable as a different-pair endpoint.
            kept[instance_id] = filtered

    same: list[IdentityPair] = []
    for instance_id, views in kept.items():
        if len(views) >= 2:
            same.extend(_pick_same_pairs(views))

    # Diff only among instances that have ≥1 framing-OK view.
    diff_pool = {i: v for i, v in kept.items() if v}
    different = _pick_diff_pairs(diff_pool)
    return tuple(same + different)


def stack_pair_image(left: Image, right: Image) -> Image:
    """Side-by-side full frames (no object crop). Heights matched; tiny A/B tags."""
    import cv2

    left_bgr = left.to_opencv()
    right_bgr = right.to_opencv()
    h = min(left_bgr.shape[0], right_bgr.shape[0])

    def _resize(img: np.ndarray) -> np.ndarray:
        if img.shape[0] == h:
            return img
        scale = h / float(img.shape[0])
        return cv2.resize(img, (max(1, int(img.shape[1] * scale)), h))

    a = _resize(left_bgr)
    b = _resize(right_bgr)
    gap = np.full((h, 8, 3), 32, dtype=np.uint8)
    canvas = np.concatenate([a, gap, b], axis=1)
    cv2.putText(canvas, "A", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    cv2.putText(
        canvas,
        "B",
        (a.shape[1] + 20, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 255, 255),
        2,
    )
    ts = max(float(left.ts), float(right.ts))
    return Image.from_opencv(canvas, frame_id=left.frame_id, ts=ts)


def answer_cross_view_same_object(pair: IdentityPair) -> FamilyAnswer:
    """Deterministic yes/no from instance_id equality (already encoded in pair)."""
    if pair.left.query.casefold() != pair.right.query.casefold():
        raise InsufficientEvidenceError("cross-view pair requires the same category query")
    if not sighting_framing_ok(pair.left) or not sighting_framing_ok(pair.right):
        raise InsufficientEvidenceError("one or both views fail center/size framing gates")
    if pair.same_object and pair.left.instance_id != pair.right.instance_id:
        raise InsufficientEvidenceError("same-object pair has mismatched instance ids")
    if (not pair.same_object) and pair.left.instance_id == pair.right.instance_id:
        raise InsufficientEvidenceError("different-object pair has identical instance ids")

    answer = "yes" if pair.same_object else "no"
    category = pair.query
    return FamilyAnswer(
        question=(
            f"Looking at image A and image B, is the {category} the same physical object "
            f"instance in both images?"
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
            "center_max_offset": CENTER_MAX_OFFSET,
            "min_area_frac": MIN_AREA_FRAC,
        },
    )


def pair_case_id(pair: IdentityPair, index: int) -> str:
    kind = "same" if pair.same_object else "diff"
    q = pair.query.replace(" ", "-")
    return (
        f"xview-{q}-{kind}-{index:04d}-"
        f"{pair.left.instance_id[:8]}-{pair.right.instance_id[:8]}"
    )
