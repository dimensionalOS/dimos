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
- ``left <category>`` / ``right <category>``: lateral extremes with margin
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import TYPE_CHECKING, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from dimos.evals.vqa.contracts import InsufficientEvidenceError, NonEmptyString

if TYPE_CHECKING:
    from dimos.perception.localize.types import Localization

ForeAft = Literal["ahead", "behind"]
EgoTurn = Literal["left", "right", "ahead"]
ReferentSelector = Literal["unique", "nearest", "left", "right"]

FORE_AFT_MIN_FORWARD_M = 0.30
TURN_AHEAD_ABS_RAD = math.pi / 6  # 30 degrees
HEIGHT_MARGIN_M = 0.15
EGO_TURN_MIN_RANGE_M = 0.30
SPATIAL_LATERAL_MARGIN_M = 0.30
SPATIAL_SIDE_SEPARATION_M = 0.40

_SELECTOR_PREFIXES: tuple[tuple[ReferentSelector, str], ...] = (
    ("nearest", "nearest "),
    ("left", "left "),
    ("right", "right "),
)


class LocalizeInstanceEvidence(BaseModel):
    """One verified object instance in the world frame."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    object_name: NonEmptyString
    position_world_xyz: tuple[float, float, float]
    extent_m: float | None = Field(default=None, gt=0)
    n_views: int = Field(default=1, ge=1)
    semantic_score: float = Field(default=1.0, ge=0)

    @classmethod
    def from_localization(
        cls, object_name: str, hit: Localization
    ) -> LocalizeInstanceEvidence:
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


def parse_referent(referent: str) -> tuple[ReferentSelector, str]:
    """Split ``nearest chair`` / ``left chair`` / bare ``chair`` into selector + category."""
    text = referent.strip()
    if not text:
        raise InsufficientEvidenceError("localize referent must be non-empty")
    lower = text.casefold()
    for selector, prefix in _SELECTOR_PREFIXES:
        if lower.startswith(prefix):
            category = text[len(prefix) :].strip()
            if not category:
                raise InsufficientEvidenceError(
                    f"localize referent {referent!r} is missing a category after {prefix!r}"
                )
            return selector, category
    return "unique", text


def format_referent(selector: ReferentSelector, category: str) -> str:
    """Build the canonical referent string used in proposals and questions."""
    if selector == "unique":
        return category
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


def resolve_referent(
    scene: LocalizeSceneEvidence, referent: str
) -> LocalizeInstanceEvidence:
    """Resolve a unique / nearest / left / right referent to one instance."""
    selector, category = parse_referent(referent)
    hits = instances_named(scene, category)
    if selector == "unique":
        if len(hits) != 1:
            raise InsufficientEvidenceError(
                f"localize scene requires exactly one verified {category!r}, got {len(hits)}"
            )
        return hits[0]
    if not hits:
        raise InsufficientEvidenceError(f"localize scene has no verified {category!r}")
    if selector == "nearest":
        return min(
            hits,
            key=lambda inst: math.hypot(
                *robot_relative_xy(scene, inst.position_world_xyz)
            ),
        )
    if selector == "left":
        return _extreme_lateral(scene, hits, side="left", referent=referent)
    return _extreme_lateral(scene, hits, side="right", referent=referent)


def _extreme_lateral(
    scene: LocalizeSceneEvidence,
    hits: Sequence[LocalizeInstanceEvidence],
    *,
    side: Literal["left", "right"],
    referent: str,
) -> LocalizeInstanceEvidence:
    scored: list[tuple[float, LocalizeInstanceEvidence]] = []
    for inst in hits:
        _, left = robot_relative_xy(scene, inst.position_world_xyz)
        if side == "left" and left >= SPATIAL_LATERAL_MARGIN_M:
            scored.append((left, inst))
        elif side == "right" and left <= -SPATIAL_LATERAL_MARGIN_M:
            scored.append((left, inst))
    if not scored:
        raise InsufficientEvidenceError(
            f"{referent!r} has no instance clearly on the {side} of the robot"
        )
    scored.sort(key=lambda item: item[0], reverse=(side == "left"))
    if len(scored) >= 2 and abs(scored[0][0] - scored[1][0]) < SPATIAL_SIDE_SEPARATION_M:
        raise InsufficientEvidenceError(
            f"{referent!r} is ambiguous: top lateral candidates are closer than "
            f"{SPATIAL_SIDE_SEPARATION_M} m"
        )
    return scored[0][1]


def spatial_side_referents(
    scene: LocalizeSceneEvidence, category: str
) -> tuple[str, str] | None:
    """Return ``(left <cat>, right <cat>)`` when both sides are unambiguous, else ``None``."""
    hits = instances_named(scene, category)
    if len(hits) < 2:
        return None
    left_name = format_referent("left", category)
    right_name = format_referent("right", category)
    try:
        left_inst = resolve_referent(scene, left_name)
        right_inst = resolve_referent(scene, right_name)
    except InsufficientEvidenceError:
        return None
    _, left_l = robot_relative_xy(scene, left_inst.position_world_xyz)
    _, right_l = robot_relative_xy(scene, right_inst.position_world_xyz)
    if left_l - right_l < SPATIAL_SIDE_SEPARATION_M:
        return None
    return left_name, right_name


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


def fore_aft_of(scene: LocalizeSceneEvidence, referent: str) -> ForeAft:
    """Whether the resolved referent lies ahead of or behind the robot."""
    inst = resolve_referent(scene, referent)
    forward, _ = robot_relative_xy(scene, inst.position_world_xyz)
    if abs(forward) < FORE_AFT_MIN_FORWARD_M:
        raise InsufficientEvidenceError(
            f"{referent!r} is too close to the robot lateral plane for fore/aft"
        )
    return "ahead" if forward > 0 else "behind"


def ego_turn_to(scene: LocalizeSceneEvidence, referent: str) -> EgoTurn:
    """Turn needed for the robot yaw to face the resolved referent."""
    inst = resolve_referent(scene, referent)
    forward, left = robot_relative_xy(scene, inst.position_world_xyz)
    if math.hypot(forward, left) < EGO_TURN_MIN_RANGE_M:
        raise InsufficientEvidenceError(
            f"{referent!r} is too close to the robot for an ego-turn bearing"
        )
    bearing = math.atan2(left, forward)
    if abs(bearing) <= TURN_AHEAD_ABS_RAD:
        return "ahead"
    return "left" if bearing > 0 else "right"


def any_ahead(scene: LocalizeSceneEvidence, category: str) -> bool:
    """Whether any verified instance of ``category`` lies clearly ahead of the robot."""
    hits = instances_named(scene, category)
    if not hits:
        raise InsufficientEvidenceError(f"localize scene has no verified {category!r}")
    return any(
        robot_relative_xy(scene, inst.position_world_xyz)[0] >= FORE_AFT_MIN_FORWARD_M
        for inst in hits
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
