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

"""Capability-coverage budget for localize-backed VQA cases.

Localize proposals are deterministic and repeat across frames. This module
publishes up to ``K`` ego/localize cases **per frame**, round-robining
``(family, selector)`` cells across frames so every frame with candidates
keeps at least one ego question and later frames are not starved by early
trajectory order. Within a chosen cell, prefer under-represented categories.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Sequence
import math
from typing import Any

from dimos.evals.vqa.families import AVAILABLE_FAMILIES
from dimos.evals.vqa.primitives.localize_scene import parse_referent

_LOCALIZE_FAMILIES = frozenset(
    family.name for family in AVAILABLE_FAMILIES if family.requires_localize
)

# Dropped from publish path (awkward wording); ignore if present in older audits.
_DROPPED_SELECTORS = frozenset({"left", "right"})

DEFAULT_LOCALIZE_COVERAGE_K = 5

# case_id, (family, selector), pose_bucket, category
CoverageCandidate = tuple[str, tuple[str, str], tuple[object, ...] | None, str]


def case_family_from_id(case_id: str) -> str:
    """Parse the family suffix from a generated case id."""
    parts = case_id.split("-")
    return parts[-1] if not parts[-1].isdigit() else parts[-2]


def case_frame_from_id(case_id: str) -> str:
    """Parse ``frame-XXXXXX`` from a generated case id."""
    parts = case_id.split("-")
    if len(parts) >= 2 and parts[0] == "frame":
        return f"frame-{parts[1]}"
    return "frame-unknown"


def coverage_cell(family: str, object_names: Sequence[str]) -> tuple[str, str]:
    """Return ``(family, selector)`` for a localize proposal."""
    if family == "ego_any_fore_aft":
        return family, "any_ahead"
    if family == "height_order":
        return family, "unique"
    if not object_names:
        return family, "unknown"
    parsed = parse_referent(object_names[0])
    return family, parsed.selector


def coverage_category(
    family: str,
    object_names: Sequence[str],
    evidence: dict[str, Any] | None = None,
) -> str:
    """Primary category used for within-cell diversity."""
    if evidence:
        category = evidence.get("category")
        if isinstance(category, str) and category.strip():
            return category.strip().casefold()
    if family == "height_order" and len(object_names) >= 2:
        left, right = sorted((object_names[0].casefold(), object_names[1].casefold()))
        return f"{left}|{right}"
    if not object_names:
        return "unknown"
    return parse_referent(object_names[0]).category.casefold()


def _pose_bucket(evidence: dict[str, Any]) -> tuple[object, ...] | None:
    position = evidence.get("robot_position_xyz")
    yaw = evidence.get("robot_yaw_rad")
    if not isinstance(position, (list, tuple)) or len(position) < 2 or yaw is None:
        return None
    try:
        return (
            round(float(position[0])),
            round(float(position[1])),
            round(float(yaw) / (math.pi / 6.0)),
        )
    except (TypeError, ValueError):
        return None


def select_localize_coverage_ids(
    candidates: Sequence[CoverageCandidate],
    *,
    k: int,
) -> set[str]:
    """Keep up to ``k`` localize case ids **per frame**.

    Frames are visited in id order. Across picks, ``(family, selector)`` cells
    rotate so consecutive frames (and successive picks on one frame) cycle ego
    question types. Every frame that still has unused candidates receives at
    least one case when ``k >= 1``.

    Inside the target cell, prefer categories under-represented **in that cell**
    so relational / any-ahead menus do not collapse onto a few objects after
    nearest picks have already filled the global category budget. Global
    category counts break remaining ties.
    """
    if k < 1:
        raise ValueError("coverage k must be at least 1")

    by_frame: dict[str, list[CoverageCandidate]] = defaultdict(list)
    cells_present: set[tuple[str, str]] = set()
    for candidate in candidates:
        case_id, cell, _pose, _category = candidate
        if cell[1] in _DROPPED_SELECTORS:
            continue
        by_frame[case_frame_from_id(case_id)].append(candidate)
        cells_present.add(cell)
    if not by_frame:
        return set()

    cell_order = sorted(cells_present)
    category_counts: dict[str, int] = defaultdict(int)
    cell_category_counts: dict[tuple[str, str, str], int] = defaultdict(int)
    kept: set[str] = set()
    cursor = 0

    def _pick_from_frame(
        frame_candidates: Sequence[CoverageCandidate],
        *,
        prefer_cell: tuple[str, str] | None,
    ) -> CoverageCandidate | None:
        unused = [row for row in frame_candidates if row[0] not in kept]
        if not unused:
            return None
        pool = [row for row in unused if prefer_cell is None or row[1] == prefer_cell]
        if not pool:
            return None

        def sort_key(row: CoverageCandidate) -> tuple[object, ...]:
            _case_id, cell, pose, category = row
            pose_penalty = 0 if pose is not None else 1
            return (
                cell_category_counts[(cell[0], cell[1], category)],
                category_counts[category],
                pose_penalty,
                _case_id,
            )

        return min(pool, key=sort_key)

    for frame in sorted(by_frame):
        frame_candidates = by_frame[frame]
        for _slot in range(k):
            chosen: CoverageCandidate | None = None
            if cell_order:
                for offset in range(len(cell_order)):
                    cell = cell_order[(cursor + offset) % len(cell_order)]
                    chosen = _pick_from_frame(frame_candidates, prefer_cell=cell)
                    if chosen is None:
                        continue
                    cursor = cursor + offset + 1
                    break
            if chosen is None:
                # Frame lacks the rotated cell; still guarantee coverage.
                chosen = _pick_from_frame(frame_candidates, prefer_cell=None)
            if chosen is None:
                break
            case_id, cell, _pose, category = chosen
            kept.add(case_id)
            category_counts[category] += 1
            cell_category_counts[(cell[0], cell[1], category)] += 1
    return kept


def collect_coverage_candidates_from_audit(
    *,
    case_id: str,
    family: str,
    question: str,
    audit_rows: Sequence[dict[str, Any]],
) -> CoverageCandidate | None:
    """Build one coverage candidate from a published case + frame audit rows."""
    if family not in _LOCALIZE_FAMILIES:
        return None
    for row in audit_rows:
        if row.get("status") != "answered":
            continue
        answer = row.get("answer") or {}
        if answer.get("question") != question:
            continue
        proposal = row.get("proposal") or {}
        object_names = tuple(proposal.get("object_names") or ())
        cell = coverage_cell(family, object_names)
        evidence = answer.get("evidence") or {}
        if not isinstance(evidence, dict):
            evidence = {}
        category = coverage_category(family, object_names, evidence)
        return case_id, cell, _pose_bucket(evidence), category
    # Localize case without matching audit row: still count under family/unknown.
    return case_id, (family, "unknown"), None, "unknown"
