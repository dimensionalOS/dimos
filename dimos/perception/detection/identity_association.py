# Copyright 2025-2026 Dimensional Inc.
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

"""Typed identity-association evidence and policy helpers."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(frozen=True)
class AssociationEvidence:
    """Evidence for one observation-to-entity candidate."""

    observation_name: str
    candidate_id: str
    reason: str
    distance: float | None = None
    distance_score: float | None = None
    track_match: bool = False
    label_match: bool | None = None
    support: int = 0
    appearance_similarity: float | None = None
    semantic_similarity: float | None = None
    visual_similarity: float | None = None
    appearance_mismatch: bool = False
    semantic_mismatch: bool = False
    visual_mismatch: bool = False
    score: float = 0.0
    accepted: bool = False
    ambiguous: bool = False
    margin: float | None = None
    vetoes: tuple[str, ...] = field(default_factory=tuple)

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["vetoes"] = list(self.vetoes)
        return data


@dataclass(frozen=True)
class IdentityAssociationCandidate:
    """Candidate entity for an observation-to-identity decision."""

    candidate_id: str
    distance: float
    support: int
    evidence: AssociationEvidence
    departed: bool = False


@dataclass(frozen=True)
class AssociationDecision:
    """Result of a pure association-policy decision."""

    candidate_id: str | None
    reason: str
    accepted: bool
    ambiguous: bool = False
    margin: float | None = None
    evidence: AssociationEvidence | None = None


class IdentityAssociationPolicy:
    """Choose among already-compatible identity candidates."""

    def choose_position_candidate(
        self,
        candidates: Sequence[IdentityAssociationCandidate],
        *,
        sticky_support: bool,
        ambiguity_margin: float = 0.0,
    ) -> AssociationDecision:
        if not candidates:
            return AssociationDecision(candidate_id=None, reason="none", accepted=False)

        if any(candidate.departed for candidate in candidates):
            ranked = sorted(candidates, key=lambda candidate: candidate.distance)
        else:
            ranked = sorted(
                candidates,
                key=lambda candidate: (
                    -candidate.evidence.score,
                    -candidate.support if sticky_support else 0,
                    candidate.distance,
                    candidate.candidate_id,
                ),
            )

        best = ranked[0]
        margin = None
        if len(ranked) > 1:
            margin = best.evidence.score - ranked[1].evidence.score
            if margin < ambiguity_margin:
                return AssociationDecision(
                    candidate_id=best.candidate_id,
                    reason="position_ambiguous",
                    accepted=False,
                    ambiguous=True,
                    margin=margin,
                    evidence=best.evidence,
                )

        return AssociationDecision(
            candidate_id=best.candidate_id,
            reason="position",
            accepted=True,
            margin=margin,
            evidence=best.evidence,
        )

    def choose_observation_candidate(
        self,
        candidates: Sequence[IdentityAssociationCandidate],
        *,
        sticky_support: bool,
        ambiguity_margin: float = 0.0,
    ) -> AssociationDecision:
        """Choose among identity candidates by evidence score."""
        if not candidates:
            return AssociationDecision(candidate_id=None, reason="none", accepted=False)

        ranked = sorted(
            candidates,
            key=lambda candidate: (
                -self._candidate_score(candidate, sticky_support=sticky_support),
                -candidate.support if sticky_support else 0,
                candidate.distance,
                candidate.candidate_id,
                candidate.evidence.reason,
            ),
        )
        best = ranked[0]
        margin = None
        if len(ranked) > 1:
            margin = self._candidate_score(best, sticky_support=sticky_support) - self._candidate_score(
                ranked[1],
                sticky_support=sticky_support,
            )
            if margin < ambiguity_margin:
                return AssociationDecision(
                    candidate_id=best.candidate_id,
                    reason="identity_ambiguous",
                    accepted=False,
                    ambiguous=True,
                    margin=margin,
                    evidence=best.evidence,
                )

        return AssociationDecision(
            candidate_id=best.candidate_id,
            reason=best.evidence.reason,
            accepted=True,
            margin=margin,
            evidence=best.evidence,
        )

    def choose_frame_position_assignments(
        self,
        candidate_sets: Mapping[Any, Sequence[IdentityAssociationCandidate]],
        *,
        sticky_support: bool,
    ) -> dict[Any, AssociationDecision]:
        """Choose one-to-one position assignments for a full frame."""
        nonempty = {key: tuple(cands) for key, cands in candidate_sets.items() if cands}
        assignments: dict[Any, AssociationDecision] = {}
        for component_keys in self._assignment_components(nonempty):
            component = {key: nonempty[key] for key in component_keys}
            assignments.update(
                self._choose_assignment_component(component, sticky_support=sticky_support)
            )
        return assignments

    def _assignment_components(
        self,
        candidate_sets: Mapping[Any, Sequence[IdentityAssociationCandidate]],
    ) -> list[tuple[Any, ...]]:
        obs_to_eids = {
            key: {candidate.candidate_id for candidate in candidates}
            for key, candidates in candidate_sets.items()
        }
        eid_to_obs: dict[str, set[Any]] = {}
        for key, eids in obs_to_eids.items():
            for eid in eids:
                eid_to_obs.setdefault(eid, set()).add(key)

        components: list[tuple[Any, ...]] = []
        unseen = set(obs_to_eids)
        while unseen:
            root = unseen.pop()
            stack = [root]
            component = {root}
            while stack:
                key = stack.pop()
                for eid in obs_to_eids.get(key, ()):
                    for other in eid_to_obs.get(eid, ()):
                        if other in unseen:
                            unseen.remove(other)
                            component.add(other)
                            stack.append(other)
            components.append(tuple(component))
        return components

    def _candidate_score(
        self,
        candidate: IdentityAssociationCandidate,
        *,
        sticky_support: bool,
    ) -> float:
        score = float(candidate.evidence.score)
        if sticky_support:
            score += 1e-6 * float(candidate.support)
        score -= 1e-9 * float(candidate.distance)
        return score

    def _choose_assignment_component(
        self,
        candidate_sets: Mapping[Any, Sequence[IdentityAssociationCandidate]],
        *,
        sticky_support: bool,
    ) -> dict[Any, AssociationDecision]:
        # Keep pathological components bounded.
        entity_ids = {candidate.candidate_id for candidates in candidate_sets.values() for candidate in candidates}
        if len(candidate_sets) > 10 or len(entity_ids) > 12:
            return self._choose_assignment_component_greedy(
                candidate_sets,
                sticky_support=sticky_support,
            )

        ordered_keys = tuple(sorted(candidate_sets, key=lambda key: (len(candidate_sets[key]), str(key))))
        ordered_candidates = {
            key: tuple(
                sorted(
                    candidate_sets[key],
                    key=lambda candidate: (
                        -self._candidate_score(candidate, sticky_support=sticky_support),
                        candidate.distance,
                        candidate.candidate_id,
                    ),
                )
            )
            for key in ordered_keys
        }
        best_metric = (-1, float("-inf"), float("-inf"))
        best_assignment: dict[Any, IdentityAssociationCandidate] = {}

        def search(
            index: int,
            used: set[str],
            score: float,
            count: int,
            distance_sum: float,
            current: dict[Any, IdentityAssociationCandidate],
        ) -> None:
            nonlocal best_metric, best_assignment
            if index >= len(ordered_keys):
                metric = (count, score, -distance_sum)
                if metric > best_metric:
                    best_metric = metric
                    best_assignment = dict(current)
                return

            key = ordered_keys[index]
            search(index + 1, used, score, count, distance_sum, current)
            for candidate in ordered_candidates[key]:
                if candidate.candidate_id in used:
                    continue
                candidate_score = self._candidate_score(candidate, sticky_support=sticky_support)
                if candidate_score <= 0.0:
                    continue
                used.add(candidate.candidate_id)
                current[key] = candidate
                search(
                    index + 1,
                    used,
                    score + candidate_score,
                    count + 1,
                    distance_sum + float(candidate.distance),
                    current,
                )
                current.pop(key, None)
                used.remove(candidate.candidate_id)

        search(0, set(), 0.0, 0, 0.0, {})
        return {
            key: AssociationDecision(
                candidate_id=candidate.candidate_id,
                reason="position",
                accepted=True,
                evidence=candidate.evidence,
            )
            for key, candidate in best_assignment.items()
        }

    def _choose_assignment_component_greedy(
        self,
        candidate_sets: Mapping[Any, Sequence[IdentityAssociationCandidate]],
        *,
        sticky_support: bool,
    ) -> dict[Any, AssociationDecision]:
        ranked: list[tuple[float, float, str, Any, IdentityAssociationCandidate]] = []
        for key, candidates in candidate_sets.items():
            for candidate in candidates:
                score = self._candidate_score(candidate, sticky_support=sticky_support)
                if score > 0.0:
                    ranked.append((score, -candidate.distance, candidate.candidate_id, key, candidate))
        ranked.sort(reverse=True)
        used_keys: set[Any] = set()
        used_entities: set[str] = set()
        assignments: dict[Any, AssociationDecision] = {}
        for _, _, _, key, candidate in ranked:
            if key in used_keys or candidate.candidate_id in used_entities:
                continue
            used_keys.add(key)
            used_entities.add(candidate.candidate_id)
            assignments[key] = AssociationDecision(
                candidate_id=candidate.candidate_id,
                reason="position",
                accepted=True,
                evidence=candidate.evidence,
            )
        return assignments
