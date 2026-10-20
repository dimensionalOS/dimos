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

"""Robot-centric geometry over verified localize instances.

This is a VQA evidence adapter only. It does not call or modify
``dimos.perception.localize``; callers build :class:`LocalizeSceneEvidence`
from localize results (or fixtures) and pass it into family answerers.

Referent strings (``object_names`` entries) may be:

- bare category: exactly one verified hit (rare / unique objects)
- ``nearest <category>``: closest hit to the robot
- ``<anchor> next to <landmark>``: anchor instance nearest a landmark category

``left`` / ``right`` ``<category>`` referents are parsed only so legacy audits can be
ignored; they are not resolved or proposed.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math
from typing import TYPE_CHECKING, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from dimos.evals.vqa.contracts import InsufficientEvidenceError, NonEmptyString

if TYPE_CHECKING:
    from dimos.perception.localize.types import Localization

ForeAft = Literal["ahead", "behind"]
EgoTurn = Literal["left", "right", "ahead", "turn around"]
ReferentSelector = Literal["unique", "nearest", "left", "right", "next_to"]

# Shared ego cones — see experience/ego-spatial-sectors.md
AHEAD_ABS_DEG = 45.0
BEHIND_ABS_DEG = 135.0
RANGE_MIN_M = 0.5
RANGE_MAX_M = 5.0
HEIGHT_MARGIN_M = 0.15
NEXT_TO_MAX_RANGE_M = 1.50
NEXT_TO_UNIQUENESS_MARGIN_M = 0.35
_NEXT_TO_TOKEN = " next to "

# left/right kept for legacy audit parsing only — not proposed or resolved.
_SELECTOR_PREFIXES: tuple[tuple[ReferentSelector, str], ...] = (
    ("nearest", "nearest "),
    ("left", "left "),
    ("right", "right "),
)


@dataclass(frozen=True)
class ParsedReferent:
    """Structured referent: selector, primary category, optional landmark."""

    selector: ReferentSelector
    category: str
    landmark: str | None = None


class LocalizeInstanceEvidence(BaseModel):
    """One verified object instance in the world frame."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    object_name: NonEmptyString
    position_world_xyz: tuple[float, float, float]
    extent_m: float | None = Field(default=None, gt=0)
    n_views: int = Field(default=1, ge=1)
    semantic_score: float = Field(default=1.0, ge=0)

    @classmethod
    def from_localization(cls, object_name: str, hit: Localization) -> LocalizeInstanceEvidence:
        """Map one Dan localize hit; ``object_name`` is the caller's query label."""
        return cls(
            object_name=object_name,
            position_world_xyz=hit.position_world_xyz,
            n_views=hit.n_views,
            semantic_score=hit.semantic_score,
        )


class LocalizeSceneEvidence(BaseModel):
    """Robot pose plus verified instances for one localize window."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    robot_position_xyz: tuple[float, float, float]
    robot_yaw_rad: float
    instances: tuple[LocalizeInstanceEvidence, ...] = ()

    @model_validator(mode="after")
    def has_finite_pose(self) -> LocalizeSceneEvidence:
        if not all(math.isfinite(v) for v in self.robot_position_xyz):
            raise ValueError("robot_position_xyz must be finite")
        if not math.isfinite(self.robot_yaw_rad):
            raise ValueError("robot_yaw_rad must be finite")
        return self


def instances_named(
    scene: LocalizeSceneEvidence, object_name: str
) -> tuple[LocalizeInstanceEvidence, ...]:
    """All instances whose label matches ``object_name`` (case-insensitive)."""
    key = object_name.casefold()
    return tuple(inst for inst in scene.instances if inst.object_name.casefold() == key)


def parse_referent(referent: str) -> ParsedReferent:
    """Parse bare / nearest / left / right / ``A next to B`` referents."""
    text = referent.strip()
    if not text:
        raise InsufficientEvidenceError("localize referent must be non-empty")
    lower = text.casefold()
    split_at = lower.find(_NEXT_TO_TOKEN)
    if split_at >= 0:
        anchor = text[:split_at].strip()
        landmark = text[split_at + len(_NEXT_TO_TOKEN) :].strip()
        if not anchor or not landmark:
            raise InsufficientEvidenceError(
                f"localize referent {referent!r} must name both sides of 'next to'"
            )
        if anchor.casefold() == landmark.casefold():
            raise InsufficientEvidenceError(
                f"localize referent {referent!r} cannot use the same category on both sides"
            )
        return ParsedReferent(selector="next_to", category=anchor, landmark=landmark)
    for selector, prefix in _SELECTOR_PREFIXES:
        if lower.startswith(prefix):
            category = text[len(prefix) :].strip()
            if not category:
                raise InsufficientEvidenceError(
                    f"localize referent {referent!r} is missing a category after {prefix!r}"
                )
            return ParsedReferent(selector=selector, category=category)
    return ParsedReferent(selector="unique", category=text)


def format_referent(
    selector: ReferentSelector,
    category: str,
    landmark: str | None = None,
) -> str:
    """Build the canonical referent string used in proposals and questions."""
    if selector == "unique":
        return category
    if selector == "next_to":
        if landmark is None or not landmark.strip():
            raise ValueError("next_to referents require a landmark category")
        return f"{category}{_NEXT_TO_TOKEN}{landmark}"
    return f"{selector} {category}"


def require_one_instance(
    scene: LocalizeSceneEvidence, object_name: str
) -> LocalizeInstanceEvidence:
    """Exactly one instance of ``object_name``, else reject."""
    hits = instances_named(scene, object_name)
    if len(hits) != 1:
        raise InsufficientEvidenceError(
            f"localize scene requires exactly one verified {object_name!r}, got {len(hits)}"
        )
    return hits[0]


def resolve_referent(scene: LocalizeSceneEvidence, referent: str) -> LocalizeInstanceEvidence:
    """Resolve a unique / nearest / next-to referent to one instance."""
    parsed = parse_referent(referent)
    if parsed.selector in ("left", "right"):
        raise InsufficientEvidenceError(
            "left/right category referents are not supported"
        )
    if parsed.selector == "next_to":
        assert parsed.landmark is not None
        return _resolve_next_to(scene, parsed.category, parsed.landmark, referent=referent)
    hits = instances_named(scene, parsed.category)
    if parsed.selector == "unique":
        if len(hits) != 1:
            raise InsufficientEvidenceError(
                f"localize scene requires exactly one verified {parsed.category!r}, got {len(hits)}"
            )
        return hits[0]
    if not hits:
        raise InsufficientEvidenceError(f"localize scene has no verified {parsed.category!r}")
    # nearest
    return min(
        hits,
        key=lambda inst: math.hypot(*robot_relative_xy(scene, inst.position_world_xyz)),
    )


def _horizontal_distance_m(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _resolve_next_to(
    scene: LocalizeSceneEvidence,
    anchor_category: str,
    landmark_category: str,
    *,
    referent: str,
) -> LocalizeInstanceEvidence:
    anchors = instances_named(scene, anchor_category)
    landmarks = instances_named(scene, landmark_category)
    if not anchors:
        raise InsufficientEvidenceError(f"localize scene has no verified {anchor_category!r}")
    if not landmarks:
        raise InsufficientEvidenceError(f"localize scene has no verified {landmark_category!r}")
    pairs: list[tuple[float, LocalizeInstanceEvidence, LocalizeInstanceEvidence]] = []
    for anchor in anchors:
        for landmark in landmarks:
            dist = _horizontal_distance_m(anchor.position_world_xyz, landmark.position_world_xyz)
            pairs.append((dist, anchor, landmark))
    pairs.sort(key=lambda item: item[0])
    best_dist, best_anchor, _best_landmark = pairs[0]
    if best_dist > NEXT_TO_MAX_RANGE_M:
        raise InsufficientEvidenceError(
            f"{referent!r} has no pair within {NEXT_TO_MAX_RANGE_M} m "
            f"(nearest pair is {best_dist:.2f} m)"
        )
    if len(pairs) >= 2 and pairs[1][0] - best_dist < NEXT_TO_UNIQUENESS_MARGIN_M:
        raise InsufficientEvidenceError(
            f"{referent!r} is ambiguous: top pairs differ by less than "
            f"{NEXT_TO_UNIQUENESS_MARGIN_M} m"
        )
    return best_anchor


def next_to_referents(scene: LocalizeSceneEvidence, *, max_pairs: int = 8) -> tuple[str, ...]:
    """Unambiguous ``<anchor> next to <landmark>`` strings, nearest pairs first."""
    if max_pairs < 1:
        return ()
    by_category: dict[str, list[LocalizeInstanceEvidence]] = {}
    display_name: dict[str, str] = {}
    for instance in scene.instances:
        key = instance.object_name.casefold()
        display_name.setdefault(key, instance.object_name)
        by_category.setdefault(key, []).append(instance)
    categories = sorted(display_name, key=lambda key: display_name[key].casefold())
    candidates: list[tuple[float, int, str]] = []
    for anchor_key in categories:
        for landmark_key in categories:
            if anchor_key == landmark_key:
                continue
            referent = format_referent(
                "next_to",
                display_name[anchor_key],
                landmark=display_name[landmark_key],
            )
            try:
                anchor = resolve_referent(scene, referent)
                landmark_hits = instances_named(scene, display_name[landmark_key])
                dist = min(
                    _horizontal_distance_m(anchor.position_world_xyz, landmark.position_world_xyz)
                    for landmark in landmark_hits
                )
            except InsufficientEvidenceError:
                continue
            # Prefer multi-instance anchors (relational disambiguation is most useful there).
            multi_penalty = 0 if len(by_category[anchor_key]) >= 2 else 1
            candidates.append((multi_penalty, dist, referent))
    candidates.sort(key=lambda item: (item[0], item[1], item[2].casefold()))
    # Cap by unique anchor category so one label does not consume the whole budget.
    chosen: list[str] = []
    used_anchors: set[str] = set()
    for _penalty, _dist, referent in candidates:
        parsed = parse_referent(referent)
        anchor_key = parsed.category.casefold()
        if anchor_key in used_anchors:
            continue
        chosen.append(referent)
        used_anchors.add(anchor_key)
        if len(chosen) >= max_pairs:
            break
    return tuple(chosen)


def robot_relative_xy(
    scene: LocalizeSceneEvidence, world_xyz: tuple[float, float, float]
) -> tuple[float, float]:
    """Point in robot frame: ``(+forward, +left)`` meters."""
    dx = world_xyz[0] - scene.robot_position_xyz[0]
    dy = world_xyz[1] - scene.robot_position_xyz[1]
    c, s = math.cos(scene.robot_yaw_rad), math.sin(scene.robot_yaw_rad)
    forward = c * dx + s * dy
    left = -s * dx + c * dy
    return forward, left


def robot_bearing_deg(
    scene: LocalizeSceneEvidence, world_xyz: tuple[float, float, float]
) -> tuple[float, float]:
    """Bearing ``θ=atan2(left, forward)`` in degrees (left-positive) and range ``r``."""
    forward, left = robot_relative_xy(scene, world_xyz)
    range_m = math.hypot(forward, left)
    return math.degrees(math.atan2(left, forward)), range_m


def _require_eligible_range(range_m: float, referent: str) -> None:
    if range_m < RANGE_MIN_M or range_m > RANGE_MAX_M:
        raise InsufficientEvidenceError(
            f"{referent!r} is outside the eligible range "
            f"[{RANGE_MIN_M}, {RANGE_MAX_M}] m (got {range_m:.2f} m)"
        )


def _in_ahead_cone(theta_deg: float, range_m: float) -> bool:
    return RANGE_MIN_M <= range_m <= RANGE_MAX_M and abs(theta_deg) <= AHEAD_ABS_DEG


def fore_aft_of(scene: LocalizeSceneEvidence, referent: str) -> ForeAft:
    """Whether the resolved referent lies in the ahead or behind cone."""
    inst = resolve_referent(scene, referent)
    theta_deg, range_m = robot_bearing_deg(scene, inst.position_world_xyz)
    _require_eligible_range(range_m, referent)
    abs_theta = abs(theta_deg)
    if abs_theta <= AHEAD_ABS_DEG:
        return "ahead"
    if abs_theta >= BEHIND_ABS_DEG:
        return "behind"
    raise InsufficientEvidenceError(
        f"{referent!r} is off to the side for fore/aft "
        f"(|θ|={abs_theta:.1f}°, need ≤{AHEAD_ABS_DEG:g}° or ≥{BEHIND_ABS_DEG:g}°)"
    )


def ego_turn_to(scene: LocalizeSceneEvidence, referent: str) -> EgoTurn:
    """Turn needed for the robot yaw to face the resolved referent."""
    inst = resolve_referent(scene, referent)
    theta_deg, range_m = robot_bearing_deg(scene, inst.position_world_xyz)
    _require_eligible_range(range_m, referent)
    abs_theta = abs(theta_deg)
    if abs_theta <= AHEAD_ABS_DEG:
        return "ahead"
    if abs_theta >= BEHIND_ABS_DEG:
        return "turn around"
    return "left" if theta_deg > 0 else "right"


def any_ahead(scene: LocalizeSceneEvidence, category: str) -> bool:
    """Whether any verified instance of ``category`` lies in the ahead cone."""
    hits = instances_named(scene, category)
    if not hits:
        raise InsufficientEvidenceError(f"localize scene has no verified {category!r}")
    return any(
        _in_ahead_cone(*robot_bearing_deg(scene, inst.position_world_xyz)) for inst in hits
    )


def higher_object(scene: LocalizeSceneEvidence, object_a: str, object_b: str) -> str:
    """Name of the higher of two resolved referents."""
    a = resolve_referent(scene, object_a)
    b = resolve_referent(scene, object_b)
    za, zb = a.position_world_xyz[2], b.position_world_xyz[2]
    if abs(za - zb) < HEIGHT_MARGIN_M:
        raise InsufficientEvidenceError(
            f"height difference between {object_a!r} and {object_b!r} is under {HEIGHT_MARGIN_M} m"
        )
    return object_a if za > zb else object_b


def scene_from_named_hits(
    robot_position_xyz: tuple[float, float, float],
    robot_yaw_rad: float,
    named_hits: Sequence[tuple[str, Localization]],
) -> LocalizeSceneEvidence:
    """Build scene evidence from ``(query_name, Localization)`` pairs."""
    return LocalizeSceneEvidence(
        robot_position_xyz=robot_position_xyz,
        robot_yaw_rad=robot_yaw_rad,
        instances=tuple(
            LocalizeInstanceEvidence.from_localization(name, hit) for name, hit in named_hits
        ),
    )
