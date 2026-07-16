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

"""Deterministic replay of saved observations into evaluation bundles."""

from collections import defaultdict
from collections.abc import Sequence
from itertools import permutations
from pathlib import Path

from dimos.benchmark.spatiotemporal.bundles import load_bundle, write_bundle
from dimos.benchmark.spatiotemporal.generation import (
    generate_spatial_question_cases,
    generate_temporal_question_cases,
)
from dimos.benchmark.spatiotemporal.intervals import build_relation_intervals
from dimos.benchmark.spatiotemporal.models import (
    ObjectObservation,
    RelationFact,
)
from dimos.benchmark.spatiotemporal.observation_io import read_observations
from dimos.benchmark.spatiotemporal.ports import (
    ObservationBundleGenerator,
    ReplayBundleResult,
    ReplayInsufficiencyCode,
    ReplayInsufficiencyError,
    replay_bundle_logical_sha256,
)
from dimos.benchmark.spatiotemporal.relations import (
    SpatialRelationCandidate,
    derive_above,
    derive_left_of,
)
from dimos.benchmark.spatiotemporal.utilities import SCHEMA_VERSION, stable_id

_RELATION_DERIVERS = (derive_left_of, derive_above)


def _relation_facts(
    observations: Sequence[ObjectObservation], margin: float
) -> tuple[RelationFact, ...]:
    by_sample: dict[tuple[int, float], list[ObjectObservation]] = defaultdict(list)
    for observation in observations:
        by_sample[(observation.frame_id, observation.timestamp_s)].append(observation)

    facts: dict[tuple[int, str], RelationFact] = {}
    for sample in by_sample.values():
        for subject, object_ in permutations(sample, 2):
            for derive in _RELATION_DERIVERS:
                candidate = derive(subject, object_, margin)
                if candidate is None:
                    continue
                fact = _relation_fact(candidate)
                facts[(fact.frame_id, fact.relation_id)] = fact
    return tuple(facts[key] for key in sorted(facts))


def _relation_fact(candidate: SpatialRelationCandidate) -> RelationFact:
    subject = candidate.subject
    object_ = candidate.object
    relation_id = stable_id(
        "relation",
        {
            "object_ids": (subject.object_id, object_.object_id),
            "predicate": candidate.predicate.value,
            "schema_version": SCHEMA_VERSION,
        },
    )
    return RelationFact(
        relation_id=relation_id,
        episode_id=subject.episode_id,
        frame_id=subject.frame_id,
        timestamp_s=subject.timestamp_s,
        subject_id=subject.object_id,
        predicate=candidate.predicate,
        object_id=object_.object_id,
        evidence_frame_ids=(subject.frame_id,),
    )


class DeterministicObservationBundleGenerator:
    """Generate deterministic bundles from canonical object observations."""

    def __init__(
        self,
        *,
        relation_margin: float = 0.0,
        sample_schedule: Sequence[tuple[int, float]] | None = None,
    ) -> None:
        self._relation_margin = relation_margin
        self._sample_schedule = tuple(sample_schedule) if sample_schedule is not None else None

    def generate(
        self,
        observations: Sequence[ObjectObservation],
        output_root: Path,
        source_video_sha256: str,
    ) -> ReplayBundleResult:
        """Generate one public/oracle evaluation bundle."""
        if not observations:
            raise ReplayInsufficiencyError(
                ReplayInsufficiencyCode.EMPTY_OBSERVATIONS,
                "saved observations are empty; capture at least one sampled frame",
            )
        episodes = {observation.episode_id for observation in observations}
        if len(episodes) != 1:
            raise ReplayInsufficiencyError(
                ReplayInsufficiencyCode.MIXED_EPISODES,
                "saved observations contain multiple episodes; replay one episode at a time",
            )

        facts = _relation_facts(observations, self._relation_margin)
        if not facts:
            raise ReplayInsufficiencyError(
                ReplayInsufficiencyCode.NO_RELATIONS,
                "saved observations contain no spatially separated object pairs",
            )
        intervals = build_relation_intervals(facts, sample_schedule=self._sample_schedule)
        sample_frame_ids = (
            tuple(frame_id for frame_id, _ in self._sample_schedule)
            if self._sample_schedule is not None
            else tuple(sorted({observation.frame_id for observation in observations}))
        )
        spatial_cases = generate_spatial_question_cases(facts, sample_frame_ids=sample_frame_ids)
        temporal_cases = generate_temporal_question_cases(intervals)
        questions = tuple(
            sorted(
                (
                    *(question for question, _ in spatial_cases),
                    *(question for question, _ in temporal_cases),
                ),
                key=lambda question: question.question_id,
            )
        )
        if not questions:
            raise ReplayInsufficiencyError(
                ReplayInsufficiencyCode.NO_QUESTIONS,
                "accepted relations produced no evaluation questions; inspect generation inputs",
            )
        answers = tuple(
            sorted(
                (
                    *(answer for _, answer in spatial_cases),
                    *(answer for _, answer in temporal_cases),
                ),
                key=lambda answer: answer.question_id,
            )
        )
        episode_id = next(iter(episodes))
        write_bundle(
            output_root,
            episode_id=episode_id,
            source_video_sha256=source_video_sha256,
            questions=questions,
            observations=observations,
            relation_facts=facts,
            relation_intervals=intervals,
            answers=answers,
        )
        bundle = load_bundle(output_root)
        return ReplayBundleResult(
            public_manifest=bundle.public_manifest,
            oracle_manifest=bundle.oracle_manifest,
            logical_sha256=replay_bundle_logical_sha256(
                bundle.public_manifest, bundle.oracle_manifest
            ),
        )


def replay_observations(
    observations_path: Path,
    output_root: Path,
    source_video_sha256: str,
    *,
    sample_schedule: Sequence[tuple[int, float]] | None = None,
    generator: ObservationBundleGenerator | None = None,
) -> ReplayBundleResult:
    """Read canonical saved observations and generate an evaluation bundle."""
    if generator is not None and sample_schedule is not None:
        raise ValueError("sample_schedule cannot be combined with a custom generator")
    observations = read_observations(observations_path)
    if generator is None:
        generator = DeterministicObservationBundleGenerator(sample_schedule=sample_schedule)
    return generator.generate(observations, output_root, source_video_sha256)
