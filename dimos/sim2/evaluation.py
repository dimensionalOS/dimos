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

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import StrEnum
import importlib.metadata as importlib_metadata
import math
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

from dimos.sim2.episodes import PublicEpisodeContext

ENTRY_POINT_GROUP = "dimos.simulation.episode_providers"


class TrialIsolationMode(StrEnum):
    """Explicit lifecycle isolation chosen by one InteractiveEval."""

    EPISODE_BOUNDARY = "episode-boundary"
    PROCESS = "process"


@dataclass(frozen=True, kw_only=True)
class EpisodeBoundary:
    provider_name: str
    episode_id: str
    previous_sample_index: int | None
    sample_index: int
    sequence: int

    def to_wire_dict(self) -> dict[str, Any]:
        raise NotImplementedError

    @classmethod
    def from_wire_dict(cls, raw: Mapping[str, Any]) -> EpisodeBoundary:
        raise NotImplementedError


@runtime_checkable
class EpisodeBoundaryListener(Protocol):
    def on_episode_boundary(self, boundary: EpisodeBoundary) -> None: ...


def _required_text(value: str, label: str) -> str:
    raise NotImplementedError


def _text_mapping(values: Mapping[str, str], label: str) -> Mapping[str, str]:
    raise NotImplementedError


def _unique_text(values: tuple[str, ...], label: str) -> tuple[str, ...]:
    raise NotImplementedError


@runtime_checkable
class EpisodeRequestContract(Protocol):
    """Provider-owned selection referenced by one DimOS evaluation case."""

    @property
    def case_id(self) -> str: ...


@dataclass(frozen=True, kw_only=True)
class EvaluationCase:
    """One provider request and the ordinary DimOS application under evaluation."""

    episode_request: EpisodeRequestContract
    blueprint_name: str
    required_modules: tuple[str, ...] = ()
    required_roles: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        raise NotImplementedError

    @property
    def case_id(self) -> str:
        """Return the identity owned by the provider request."""

        raise NotImplementedError


@dataclass(frozen=True)
class PreparedEpisode:
    """Provider output required to start one ordinary DimOS blueprint."""

    provider_name: str
    episode_id: str
    case_id: str
    blueprint_name: str
    simulator: str | None
    context: PublicEpisodeContext
    global_args: tuple[str, ...] = ()
    extra_env: Mapping[str, str] = field(default_factory=dict)
    required_modules: tuple[str, ...] = ()
    private_role_reset_positions: Mapping[str, tuple[float, float, float]] = field(
        default_factory=dict
    )
    initial_sample_index: int = 0
    distribution_id: str | None = None
    distribution_revision: str | None = None
    distribution_seed: int | None = None
    distribution_digest: str | None = None
    initial_sample_digest: str | None = None
    bounded_sample_count: int | None = None

    def __post_init__(self) -> None:
        raise NotImplementedError

    def _validate_distribution_contract(self) -> None:
        raise NotImplementedError


@dataclass(frozen=True, kw_only=True)
class EpisodeActivationResult:
    """Provider-neutral result of activating one exact distribution sample."""

    provider_name: str
    episode_id: str
    case_id: str
    sample_index: int
    context: PublicEpisodeContext
    boundary: EpisodeBoundary
    initial_conditions_passed: bool
    failed_conditions: tuple[str, ...] = ()
    sample_digest: str | None = None
    provenance: Mapping[str, str | int | float | bool] = field(default_factory=dict)

    def __post_init__(self) -> None:
        raise NotImplementedError


@dataclass(frozen=True)
class EpisodeEvaluationResult:
    """Private physical evaluation result returned to the DimOS test runner."""

    provider_name: str
    episode_id: str
    case_id: str
    passed: bool
    summary: str
    metrics: Mapping[str, float] = field(default_factory=dict)
    sample_index: int = 0
    sample_digest: str | None = None
    provenance: Mapping[str, str | int | float | bool] = field(default_factory=dict)

    def __post_init__(self) -> None:
        raise NotImplementedError

    def _validate_distribution_result(self) -> None:
        raise NotImplementedError


class EpisodeUnavailableError(RuntimeError):
    """The provider is installed, but optional episode data is unavailable."""


@runtime_checkable
class EpisodeProvider(Protocol):
    """Prepare and privately evaluate episodes for the DimOS E2E runner."""

    provider_name: str
    supported_family_ids: tuple[str, ...]

    def prepare(self, case: EvaluationCase, output_dir: Path) -> PreparedEpisode: ...

    def start(self, episode: PreparedEpisode) -> None: ...

    def activate(
        self,
        episode: PreparedEpisode,
        sample_index: int,
    ) -> EpisodeActivationResult: ...

    def evaluate(self, episode: PreparedEpisode) -> EpisodeEvaluationResult: ...

    def stop(self) -> None: ...


def load_episode_provider(name: str) -> EpisodeProvider:
    raise NotImplementedError


__all__ = [
    "ENTRY_POINT_GROUP",
    "EpisodeActivationResult",
    "EpisodeBoundary",
    "EpisodeBoundaryListener",
    "EpisodeEvaluationResult",
    "EpisodeProvider",
    "EpisodeRequestContract",
    "EpisodeUnavailableError",
    "EvaluationCase",
    "PreparedEpisode",
    "TrialIsolationMode",
    "load_episode_provider",
]
