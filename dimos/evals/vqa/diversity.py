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

"""Select a diverse, difficulty-aware VQA publish set (not coverage-first)."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from dimos.evals.vqa.localize_coverage import (
    case_family_from_id,
    coverage_category,
)
from dimos.evals.vqa.primitives.localize_scene import parse_referent

DEFAULT_MAX_PER_CATEGORY = 2
DEFAULT_MAX_PER_FAMILY_CATEGORY = 1


@dataclass(frozen=True)
class DiversityCandidate:
    case_id: str
    family: str
    category: str
    difficulty: float
    answer: str


def _proposal_names(audit_rows: Sequence[Mapping[str, object]], case_id: str) -> tuple[str, ...]:
    # audit rows are parallel to answered proposals; match by rebuilt case order is
    # caller-owned. Here we recover names from the answered audit entry when present.
    del case_id
    for row in audit_rows:
        if row.get("status") != "answered":
            continue
        proposal = row.get("proposal")
        if not isinstance(proposal, Mapping):
            continue
        names = proposal.get("object_names")
        if isinstance(names, (list, tuple)) and names:
            return tuple(str(name) for name in names)
    return ()


def difficulty_for(
    family: str,
    answer: str,
    object_names: Sequence[str],
) -> float:
    """Higher = prefer when thinning duplicates (depth over easy repeats)."""
    ans = answer.casefold().strip()
    if family == "presence":
        return 0.2 if ans == "yes" else 1.0
    if family == "object_count":
        return {
            "one": 0.2,
            "two": 0.5,
            "three": 0.8,
            "four or more": 1.0,
        }.get(ans, 0.4)
    if family == "horizontal_direction":
        return {"center": 0.3, "left": 0.7, "right": 0.7}.get(ans, 0.5)
    if family == "image_coverage":
        if "75%" in ans or "85%" in ans:
            return 1.0
        if "50%" in ans or "65%" in ans:
            return 0.8
        if "25%" in ans or "35%" in ans:
            return 0.5
        return 0.3
    if family in {"largest_visible_area", "closest_object", "height_order"}:
        return 0.6 + 0.1 * min(len(object_names), 3)
    if family == "object_distance":
        return {
            "under 1 meter": 0.4,
            "1 to under 2 meters": 0.6,
            "2 to under 3 meters": 0.8,
            "3 meters or more": 1.0,
        }.get(ans, 0.5)
    if family == "ego_fore_aft":
        return 1.0 if ans == "behind" else 0.4
    if family == "ego_turn":
        return {"ahead": 0.3, "left": 0.8, "right": 0.8, "turn around": 1.0}.get(ans, 0.5)
    if family == "ego_any_fore_aft":
        return 0.9 if ans == "yes" else 0.35
    if family == "cross_view_same_object":
        return 0.7 if ans == "yes" else 0.9
    # Prefer nearest / next-to referents slightly over bare names.
    if object_names:
        selector = parse_referent(object_names[0]).selector
        if selector == "next_to":
            return 0.85
        if selector == "nearest":
            return 0.7
    return 0.5


def candidates_from_answered_audit(
    *,
    case_id: str,
    family: str,
    question: str,
    answer: str,
    audit_rows: Sequence[Mapping[str, object]],
) -> DiversityCandidate:
    """Build one diversity candidate from a published case + its answered audit row."""
    object_names: tuple[str, ...] = ()
    evidence: dict[str, object] | None = None
    for row in audit_rows:
        if row.get("status") != "answered":
            continue
        ans_obj = row.get("answer")
        if not isinstance(ans_obj, Mapping):
            continue
        if str(ans_obj.get("question") or "") != question:
            continue
        proposal = row.get("proposal")
        if isinstance(proposal, Mapping):
            names = proposal.get("object_names")
            if isinstance(names, (list, tuple)):
                object_names = tuple(str(name) for name in names)
        ev = ans_obj.get("evidence")
        if isinstance(ev, dict):
            evidence = ev
        break
    if not object_names:
        object_names = _proposal_names(audit_rows, case_id)
    category = coverage_category(family, object_names, evidence)
    return DiversityCandidate(
        case_id=case_id,
        family=family,
        category=category,
        difficulty=difficulty_for(family, answer, object_names),
        answer=answer,
    )


def select_diverse_case_ids(
    candidates: Sequence[DiversityCandidate],
    *,
    max_per_category: int = DEFAULT_MAX_PER_CATEGORY,
    max_per_family_category: int = DEFAULT_MAX_PER_FAMILY_CATEGORY,
) -> set[str]:
    """Greedy pick: hardest first, then under-filled family, with category caps.

    Goal is a balanced stress set (difficulty + diversity), not spatial/frame coverage.
    """
    if max_per_category < 1:
        raise ValueError("max_per_category must be at least 1")
    if max_per_family_category < 1:
        raise ValueError("max_per_family_category must be at least 1")
    if not candidates:
        return set()

    by_family: dict[str, list[DiversityCandidate]] = defaultdict(list)
    for cand in candidates:
        by_family[cand.family].append(cand)
    for family in by_family:
        by_family[family].sort(key=lambda c: (-c.difficulty, c.category, c.case_id))

    kept: list[DiversityCandidate] = []
    category_counts: dict[str, int] = defaultdict(int)
    family_category_counts: dict[tuple[str, str], int] = defaultdict(int)
    family_counts: dict[str, int] = defaultdict(int)
    cursors = dict.fromkeys(by_family, 0)

    def _eligible(cand: DiversityCandidate) -> bool:
        if category_counts[cand.category] >= max_per_category:
            return False
        if family_category_counts[(cand.family, cand.category)] >= max_per_family_category:
            return False
        return True

    remaining = sum(len(rows) for rows in by_family.values())
    while remaining > 0:
        # Prefer families that currently have the fewest kept cases (balance).
        order = sorted(
            by_family.keys(),
            key=lambda family: (family_counts[family], family),
        )
        progress = False
        for family in order:
            rows = by_family[family]
            idx = cursors[family]
            while idx < len(rows) and not _eligible(rows[idx]):
                idx += 1
                remaining -= 1
            cursors[family] = idx
            if idx >= len(rows):
                continue
            pick = rows[idx]
            cursors[family] = idx + 1
            remaining -= 1
            kept.append(pick)
            category_counts[pick.category] += 1
            family_category_counts[(pick.family, pick.category)] += 1
            family_counts[pick.family] += 1
            progress = True
            break
        if not progress:
            break

    return {cand.case_id for cand in kept}


def family_of_case_id(case_id: str) -> str:
    return case_family_from_id(case_id)
