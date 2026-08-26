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

"""Phase 3 checks for provider-neutral episode lifecycle helpers."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import pytest

from dimos.e2e_tests.episode import (
    prepare_episode,
    start_episode,
    validate_episode_activation,
)
from dimos.sim2.episodes import PublicEpisodeContext, PublicEpisodeRole
from dimos.sim2.evaluation import (
    EpisodeActivationResult,
    EpisodeBoundary,
    EpisodeEvaluationResult,
    EvaluationCase,
    PreparedEpisode,
)


@dataclass(frozen=True)
class _EpisodeRequest:
    case_id: str = "distribution-case"
    provider_name = "fake"


class _Provider:
    provider_name = "fake"
    supported_family_ids = ("distribution",)

    def __init__(self) -> None:
        self.prepare_calls: list[int | None] = []
        self.validate_calls: list[int] = []
        self.start_calls = 0
        self.activate_calls: list[int] = []
        self.events: list[str] = []
        self.active_sample_index: int | None = None
        self.sequence = 0
        self.rejected_samples: set[int] = set()
        self.start_sample_index: int | None = None
        self.start_case_id: str | None = None
        self.start_digest: str | None = None
        self.stopped = False

    def prepare(
        self,
        case: EvaluationCase,
        output_dir: Path,
        *,
        sample_index: int | None = None,
    ) -> PreparedEpisode:
        del output_dir
        self.prepare_calls.append(sample_index)
        selected = 0 if sample_index is None else sample_index
        return _prepared_episode(case=case, sample_index=selected)

    def validate_activation(
        self,
        episode: PreparedEpisode,
        sample_index: int,
    ) -> None:
        self.events.append(f"validate:{sample_index}")
        self.validate_calls.append(sample_index)
        if episode.episode_id != "episode-1":
            raise ValueError("unknown prepared episode")
        if sample_index in self.rejected_samples:
            raise ValueError(f"sample {sample_index} requires another topology")
        if sample_index < 0 or sample_index >= (episode.bounded_sample_count or 1):
            raise IndexError(sample_index)

    def start(self, episode: PreparedEpisode) -> EpisodeActivationResult:
        self.events.append("start")
        self.start_calls += 1
        self.sequence = 1
        sample_index = (
            episode.initial_sample_index
            if self.start_sample_index is None
            else self.start_sample_index
        )
        self.active_sample_index = sample_index
        return _activation_result(
            episode,
            sample_index=sample_index,
            previous_sample_index=None,
            sequence=self.sequence,
            case_id=self.start_case_id,
            sample_digest=self.start_digest,
        )

    def activate(
        self,
        episode: PreparedEpisode,
        sample_index: int,
    ) -> EpisodeActivationResult:
        self.events.append(f"activate:{sample_index}")
        self.activate_calls.append(sample_index)
        previous = self.active_sample_index
        self.active_sample_index = sample_index
        self.sequence += 1
        return _activation_result(
            episode,
            sample_index=sample_index,
            previous_sample_index=previous,
            sequence=self.sequence,
        )

    def evaluate(self, episode: PreparedEpisode) -> EpisodeEvaluationResult:
        sample_index = self.active_sample_index
        if sample_index is None:
            raise RuntimeError("provider is not started")
        return EpisodeEvaluationResult(
            provider_name=self.provider_name,
            episode_id=episode.episode_id,
            case_id=episode.case_id,
            passed=sample_index % 2 == 0,
            summary=f"private outcome for sample {sample_index}",
            metrics={"sample_index": float(sample_index)},
            sample_index=sample_index,
            sample_digest=_digest(sample_index),
            provenance={"sample_index": sample_index},
        )

    def stop(self) -> None:
        self.stopped = True


def _evaluation_case() -> EvaluationCase:
    return EvaluationCase(
        episode_request=_EpisodeRequest(),
        blueprint_name="robot-sim",
        required_roles=("object",),
    )


def _context(
    sample_index: int,
    *,
    case_id: str = "distribution-case",
) -> PublicEpisodeContext:
    return PublicEpisodeContext(
        case_id=case_id,
        task_family_id="lift-object",
        instruction=f"Lift object {sample_index}",
        roles={
            "object": PublicEpisodeRole(
                role_id="object",
                entity_id=f"object-{sample_index}",
                name=f"object {sample_index}",
            )
        },
    )


def _prepared_episode(
    *,
    case: EvaluationCase | None = None,
    sample_index: int = 0,
) -> PreparedEpisode:
    selected_case = _evaluation_case() if case is None else case
    return PreparedEpisode(
        provider_name="fake",
        episode_id="episode-1",
        case_id=selected_case.case_id,
        blueprint_name=selected_case.blueprint_name,
        simulator="mujoco",
        context=_context(sample_index, case_id=selected_case.case_id),
        required_modules=("Mechanics",),
        initial_sample_index=sample_index,
        distribution_id="bounded-distribution",
        distribution_revision="1",
        distribution_seed=20260824,
        distribution_digest="a" * 64,
        initial_sample_digest=_digest(sample_index),
        bounded_sample_count=8,
    )


def _activation_result(
    episode: PreparedEpisode,
    *,
    sample_index: int,
    previous_sample_index: int | None,
    sequence: int,
    case_id: str | None = None,
    sample_digest: str | None = None,
) -> EpisodeActivationResult:
    selected_case_id = episode.case_id if case_id is None else case_id
    return EpisodeActivationResult(
        provider_name=episode.provider_name,
        episode_id=episode.episode_id,
        case_id=selected_case_id,
        sample_index=sample_index,
        context=_context(sample_index, case_id=selected_case_id),
        boundary=EpisodeBoundary(
            provider_name=episode.provider_name,
            episode_id=episode.episode_id,
            previous_sample_index=previous_sample_index,
            sample_index=sample_index,
            sequence=sequence,
        ),
        initial_conditions_passed=True,
        sample_digest=_digest(sample_index) if sample_digest is None else sample_digest,
        provenance={
            "distribution_digest": "a" * 64,
            "sample_index": sample_index,
            "sample_digest": _digest(sample_index),
        },
    )


def _digest(sample_index: int) -> str:
    return f"{sample_index + 1:064x}"


def test_prepare_episode_forwards_an_explicit_sample_index(tmp_path: Path) -> None:
    provider = _Provider()

    episode = prepare_episode(
        provider,
        _evaluation_case(),
        tmp_path,
        sample_index=4,
    )

    assert provider.prepare_calls == [4]
    assert episode.initial_sample_index == 4
    assert episode.initial_sample_digest == _digest(4)


def test_validate_episode_activation_is_non_mutating() -> None:
    provider = _Provider()
    episode = _prepared_episode()
    start_episode(provider, episode)
    active_before = provider.active_sample_index

    validate_episode_activation(provider, episode, 5)

    assert provider.validate_calls == [5]
    assert provider.active_sample_index == active_before
    assert provider.activate_calls == []


def test_start_episode_validates_provider_episode_case_and_sample_identity() -> None:
    episode = _prepared_episode(sample_index=2)
    provider = _Provider()

    result = start_episode(provider, episode)

    assert result.sample_index == 2
    assert result.sample_digest == episode.initial_sample_digest
    assert result.boundary.previous_sample_index is None

    wrong_sample = _Provider()
    wrong_sample.start_sample_index = 3
    with pytest.raises(ValueError, match="started sample 3"):
        start_episode(wrong_sample, episode)

    wrong_case = _Provider()
    wrong_case.start_case_id = "another-case"
    with pytest.raises(ValueError, match="result identity"):
        start_episode(wrong_case, episode)

    wrong_digest = _Provider()
    wrong_digest.start_digest = "f" * 64
    with pytest.raises(ValueError, match="started sample digest"):
        start_episode(wrong_digest, episode)
