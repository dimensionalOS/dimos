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

Localize proposals are deterministic and repeat across frames. Publishing every
answered proposal turns the set into a trajectory sweep. This module keeps at
most ``K`` published cases per ``(family, selector)`` cell so the eval measures
capability coverage rather than exhaustive enumeration.
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

DEFAULT_LOCALIZE_COVERAGE_K = 5


def case_family_from_id(case_id: str) -> str:
    """Parse the family suffix from a generated case id."""
    parts = case_id.split("-")
    return parts[-1] if not parts[-1].isdigit() else parts[-2]


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
    candidates: Sequence[tuple[str, tuple[str, str], tuple[object, ...] | None]],
    *,
    k: int,
) -> set[str]:
    """Keep up to ``k`` case ids per coverage cell, preferring distinct pose buckets."""
    if k < 1:
        raise ValueError("coverage k must be at least 1")
    by_cell: dict[tuple[str, str], list[tuple[str, tuple[object, ...] | None]]] = defaultdict(list)
    for case_id, cell, pose in candidates:
        by_cell[cell].append((case_id, pose))

    kept: set[str] = set()
    for cell_candidates in by_cell.values():
        selected: list[str] = []
        used_poses: set[tuple[object, ...]] = set()
        # Pass 1: unique pose buckets.
        for case_id, pose in cell_candidates:
            if len(selected) >= k:
                break
            if pose is not None and pose in used_poses:
                continue
            selected.append(case_id)
            if pose is not None:
                used_poses.add(pose)
        # Pass 2: fill remaining slots for sparse cells.
        if len(selected) < k:
            selected_set = set(selected)
            for case_id, _pose in cell_candidates:
                if len(selected) >= k:
                    break
                if case_id in selected_set:
                    continue
                selected.append(case_id)
        kept.update(selected)
    return kept


def collect_coverage_candidates_from_audit(
    *,
    case_id: str,
    family: str,
    question: str,
    audit_rows: Sequence[dict[str, Any]],
) -> tuple[str, tuple[str, str], tuple[object, ...] | None] | None:
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
        return case_id, cell, _pose_bucket(evidence)
    # Localize case without matching audit row: still count under family/unknown.
    return case_id, (family, "unknown"), None
