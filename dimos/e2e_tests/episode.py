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

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time

from dimos.porcelain.dimos import Dimos
from dimos.sim2.evaluation import (
    EpisodeActivationResult,
    EpisodeEvaluationResult,
    EpisodeProvider,
    EvaluationCase,
    PreparedEpisode,
)


def _check_identity(
    case: EvaluationCase,
    episode: PreparedEpisode,
    *,
    provider_name: str,
) -> None:
    if episode.provider_name != provider_name:
        raise ValueError(
            f"provider {provider_name!r} prepared an episode for {episode.provider_name!r}"
        )
    if episode.case_id != case.case_id:
        raise ValueError(f"provider prepared case {episode.case_id!r}; expected {case.case_id!r}")
    if episode.blueprint_name != case.blueprint_name:
        raise ValueError(
            f"provider changed blueprint {case.blueprint_name!r} to {episode.blueprint_name!r}"
        )


def prepare_episode(
    provider: EpisodeProvider,
    case: EvaluationCase,
    output_dir: Path,
    *,
    sample_index: int | None = None,
) -> PreparedEpisode:
    episode = provider.prepare(case, output_dir, sample_index=sample_index)
    _check_identity(case, episode, provider_name=provider.provider_name)
    missing_roles = set(case.required_roles) - set(episode.context.roles)
    if missing_roles:
        raise ValueError(f"prepared episode is missing roles: {sorted(missing_roles)}")
    return episode


def validate_episode_activation(
    provider: EpisodeProvider,
    episode: PreparedEpisode,
    sample_index: int,
) -> None:
    """Preflight one sample without mutating or launching provider state."""

    provider.validate_activation(episode, sample_index)


def start_episode(
    provider: EpisodeProvider,
    episode: PreparedEpisode,
) -> EpisodeActivationResult:
    """Start the provider and validate its sole initial activation."""

    result = provider.start(episode)
    _check_result_identity(episode, result.provider_name, result.episode_id, result.case_id)
    if result.sample_index != episode.initial_sample_index:
        raise ValueError(
            f"provider started sample {result.sample_index}; "
            f"expected {episode.initial_sample_index}"
        )
    if result.boundary.previous_sample_index is not None:
        raise ValueError("provider startup activation must not have a previous sample")
    if (
        episode.initial_sample_digest is not None
        and result.sample_digest != episode.initial_sample_digest
    ):
        raise ValueError(
            f"provider started sample digest {result.sample_digest!r}; "
            f"expected {episode.initial_sample_digest!r}"
        )
    return result


def activate_episode(
    provider: EpisodeProvider,
    episode: PreparedEpisode,
    sample_index: int,
) -> EpisodeActivationResult:
    """Activate one exact provider-owned sample and verify its identity."""

    result = provider.activate(episode, sample_index)
    _check_result_identity(episode, result.provider_name, result.episode_id, result.case_id)
    if result.sample_index != sample_index:
        raise ValueError(
            f"provider activated sample {result.sample_index}; expected {sample_index}"
        )
    return result


def evaluate_episode(
    provider: EpisodeProvider,
    episode: PreparedEpisode,
) -> EpisodeEvaluationResult:
    result = provider.evaluate(episode)
    _check_result_identity(episode, result.provider_name, result.episode_id, result.case_id)
    return result


def _check_result_identity(
    episode: PreparedEpisode,
    provider_name: str,
    episode_id: str,
    case_id: str,
) -> None:
    actual = (provider_name, episode_id, case_id)
    expected = (episode.provider_name, episode.episode_id, episode.case_id)
    if actual != expected:
        raise ValueError(f"episode result identity {actual!r} does not match {expected!r}")


@dataclass(frozen=True)
class EpisodeRun:
    """One prepared episode and the live public DimOS application under test."""

    case: EvaluationCase
    episode: PreparedEpisode
    activation: EpisodeActivationResult
    app: Dimos
    provider: EpisodeProvider

    def role(self, role_id: str) -> str:
        return self.episode.context.role(role_id).name

    def evaluate_goal(self) -> EpisodeEvaluationResult:
        return evaluate_episode(self.provider, self.episode)

    def wait_for_goal(self, timeout: float = 5.0) -> EpisodeEvaluationResult:
        deadline = time.monotonic() + timeout
        result = self.evaluate_goal()
        while not result.passed and time.monotonic() < deadline:
            time.sleep(0.1)
            result = self.evaluate_goal()
        return result


__all__ = [
    "EpisodeRun",
    "activate_episode",
    "evaluate_episode",
    "prepare_episode",
    "start_episode",
    "validate_episode_activation",
]
