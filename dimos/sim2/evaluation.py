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
from types import MappingProxyType
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

    def __post_init__(self) -> None:
        for field_name in ("provider_name", "episode_id"):
            object.__setattr__(
                self,
                field_name,
                _required_text(getattr(self, field_name), f"episode boundary {field_name}"),
            )
        for field_name in ("sample_index", "sequence"):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"episode boundary {field_name} must be a non-negative integer")
        if self.previous_sample_index is not None and (
            isinstance(self.previous_sample_index, bool)
            or not isinstance(self.previous_sample_index, int)
            or self.previous_sample_index < 0
        ):
            raise ValueError(
                "episode boundary previous_sample_index must be a non-negative integer or null"
            )

    def to_wire_dict(self) -> dict[str, Any]:
        return {
            "provider_name": self.provider_name,
            "episode_id": self.episode_id,
            "previous_sample_index": self.previous_sample_index,
            "sample_index": self.sample_index,
            "sequence": self.sequence,
        }

    @classmethod
    def from_wire_dict(cls, raw: Mapping[str, Any]) -> EpisodeBoundary:
        previous = raw.get("previous_sample_index")
        if previous is not None and (isinstance(previous, bool) or not isinstance(previous, int)):
            raise TypeError("episode boundary previous_sample_index must be an integer or null")
        sample_index = raw.get("sample_index")
        sequence = raw.get("sequence")
        if isinstance(sample_index, bool) or not isinstance(sample_index, int):
            raise TypeError("episode boundary sample_index must be an integer")
        if isinstance(sequence, bool) or not isinstance(sequence, int):
            raise TypeError("episode boundary sequence must be an integer")
        return cls(
            provider_name=_required_text(
                str(raw.get("provider_name", "")), "episode boundary provider_name"
            ),
            episode_id=_required_text(
                str(raw.get("episode_id", "")), "episode boundary episode_id"
            ),
            previous_sample_index=previous,
            sample_index=sample_index,
            sequence=sequence,
        )


@runtime_checkable
class EpisodeBoundaryListener(Protocol):
    def on_episode_boundary(self, boundary: EpisodeBoundary) -> None: ...


def _required_text(value: str, label: str) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{label} must be a string")
    normalized = value.strip()
    if not normalized:
        raise ValueError(f"{label} must not be empty")
    return normalized


def _text_mapping(values: Mapping[str, str], label: str) -> Mapping[str, str]:
    normalized: dict[str, str] = {}
    for key, value in values.items():
        normalized_key = _required_text(str(key), f"{label} key")
        if not isinstance(value, str):
            raise TypeError(f"{label} values must be strings")
        normalized[normalized_key] = value
    return MappingProxyType(normalized)


def _unique_text(values: tuple[str, ...], label: str) -> tuple[str, ...]:
    normalized = tuple(_required_text(value, label) for value in values)
    if len(normalized) != len(set(normalized)):
        raise ValueError(f"{label} values must be unique")
    return normalized


@runtime_checkable
class EpisodeRequestContract(Protocol):
    """Provider-owned selection referenced by one DimOS evaluation case."""

    @property
    def case_id(self) -> str: ...


@runtime_checkable
class ProviderEpisodeRequestContract(EpisodeRequestContract, Protocol):
    """Public episode reference that also selects its lifecycle provider."""

    @property
    def provider_name(self) -> str: ...


@dataclass(frozen=True, kw_only=True)
class EvaluationCase:
    """One provider request and the ordinary DimOS application under evaluation."""

    episode_request: EpisodeRequestContract
    blueprint_name: str
    required_modules: tuple[str, ...] = ()
    required_roles: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.episode_request, EpisodeRequestContract):
            raise TypeError("evaluation case episode_request does not expose case_id")
        _required_text(self.episode_request.case_id, "evaluation case request case_id")
        object.__setattr__(
            self,
            "blueprint_name",
            _required_text(self.blueprint_name, "evaluation blueprint_name"),
        )
        object.__setattr__(
            self,
            "required_modules",
            _unique_text(tuple(self.required_modules), "required module"),
        )
        object.__setattr__(
            self,
            "required_roles",
            _unique_text(tuple(self.required_roles), "required role"),
        )

    @property
    def case_id(self) -> str:
        """Return the identity owned by the provider request."""

        return self.episode_request.case_id


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
        for field_name in ("provider_name", "episode_id", "case_id", "blueprint_name"):
            object.__setattr__(
                self,
                field_name,
                _required_text(getattr(self, field_name), f"prepared episode {field_name}"),
            )
        if self.context.case_id != self.case_id:
            raise ValueError("prepared episode context belongs to another case")
        if self.simulator is not None:
            object.__setattr__(
                self,
                "simulator",
                _required_text(self.simulator, "prepared episode simulator"),
            )
        global_args = tuple(self.global_args)
        if any(not isinstance(value, str) or not value for value in global_args):
            raise ValueError("prepared episode global arguments must be non-empty strings")
        object.__setattr__(self, "global_args", global_args)
        object.__setattr__(
            self,
            "extra_env",
            _text_mapping(self.extra_env, "prepared episode environment"),
        )
        object.__setattr__(
            self,
            "required_modules",
            _unique_text(tuple(self.required_modules), "prepared required module"),
        )
        reset_positions: dict[str, tuple[float, float, float]] = {}
        for role_id, raw_position in self.private_role_reset_positions.items():
            role = _required_text(str(role_id), "private reset role")
            position = tuple(float(value) for value in raw_position)
            if len(position) != 3 or not all(math.isfinite(value) for value in position):
                raise ValueError("private role reset positions must contain three finite values")
            if role not in self.context.roles:
                raise ValueError(f"private reset role {role!r} is absent from public context")
            reset_positions[role] = position
        object.__setattr__(
            self,
            "private_role_reset_positions",
            MappingProxyType(reset_positions),
        )
        if (
            isinstance(self.initial_sample_index, bool)
            or not isinstance(self.initial_sample_index, int)
            or self.initial_sample_index < 0
        ):
            raise ValueError("prepared episode initial_sample_index must be non-negative")
        self._validate_distribution_contract()

    def _validate_distribution_contract(self) -> None:
        identity = (
            self.distribution_id,
            self.distribution_revision,
            self.distribution_seed,
            self.distribution_digest,
            self.initial_sample_digest,
        )
        if all(value is None for value in identity):
            if self.initial_sample_index != 0 or self.bounded_sample_count is not None:
                raise ValueError("ordinary prepared episode has distribution-only fields")
            return
        if any(value is None for value in identity):
            raise ValueError("prepared distribution episode has incomplete identity")
        assert self.distribution_id is not None
        assert self.distribution_revision is not None
        assert self.distribution_seed is not None
        assert self.distribution_digest is not None
        assert self.initial_sample_digest is not None
        object.__setattr__(
            self,
            "distribution_id",
            _required_text(self.distribution_id, "distribution ID"),
        )
        object.__setattr__(
            self,
            "distribution_revision",
            _required_text(self.distribution_revision, "distribution revision"),
        )
        if (
            isinstance(self.distribution_seed, bool)
            or not isinstance(self.distribution_seed, int)
            or self.distribution_seed < 0
        ):
            raise ValueError("distribution seed must be a non-negative integer")
        for value, label in (
            (self.distribution_digest, "distribution digest"),
            (self.initial_sample_digest, "initial sample digest"),
        ):
            if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
                raise ValueError(f"{label} must be a lowercase SHA-256 digest")
        if self.bounded_sample_count is not None:
            if (
                isinstance(self.bounded_sample_count, bool)
                or not isinstance(self.bounded_sample_count, int)
                or self.bounded_sample_count < 1
            ):
                raise ValueError("bounded_sample_count must be a positive integer")
            if self.initial_sample_index >= self.bounded_sample_count:
                raise ValueError("initial sample index is outside the bounded distribution")


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
        for field_name in ("provider_name", "episode_id", "case_id"):
            object.__setattr__(
                self,
                field_name,
                _required_text(getattr(self, field_name), f"activation {field_name}"),
            )
        if self.context.case_id != self.case_id:
            raise ValueError("activation context belongs to another case")
        if (
            isinstance(self.sample_index, bool)
            or not isinstance(self.sample_index, int)
            or self.sample_index < 0
        ):
            raise ValueError("activation sample index must be non-negative")
        if (
            self.boundary.provider_name != self.provider_name
            or self.boundary.episode_id != self.episode_id
            or self.boundary.sample_index != self.sample_index
        ):
            raise ValueError("activation boundary identity does not match the result")
        if not isinstance(self.initial_conditions_passed, bool):
            raise TypeError("initial_conditions_passed must be a boolean")
        object.__setattr__(
            self,
            "failed_conditions",
            tuple(_required_text(value, "failed condition") for value in self.failed_conditions),
        )
        if self.initial_conditions_passed == bool(self.failed_conditions):
            raise ValueError("activation initial-condition result is inconsistent")
        if self.sample_digest is not None and not _is_sha256(self.sample_digest):
            raise ValueError("activation sample_digest must be a lowercase SHA-256 digest")
        object.__setattr__(self, "provenance", _scalar_mapping(self.provenance, "activation"))


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
        for field_name in ("provider_name", "episode_id", "case_id", "summary"):
            object.__setattr__(
                self,
                field_name,
                _required_text(getattr(self, field_name), f"evaluation {field_name}"),
            )
        if not isinstance(self.passed, bool):
            raise TypeError("evaluation passed must be a boolean")
        if (
            isinstance(self.sample_index, bool)
            or not isinstance(self.sample_index, int)
            or self.sample_index < 0
        ):
            raise ValueError("evaluation sample index must be non-negative")
        metrics = {str(name): float(value) for name, value in self.metrics.items()}
        if any(not name.strip() for name in metrics) or any(
            not math.isfinite(value) for value in metrics.values()
        ):
            raise ValueError("evaluation metrics require names and finite values")
        object.__setattr__(self, "metrics", MappingProxyType(metrics))
        object.__setattr__(self, "provenance", _scalar_mapping(self.provenance, "evaluation"))
        self._validate_distribution_result()

    def _validate_distribution_result(self) -> None:
        if self.sample_digest is None:
            if self.sample_index != 0:
                raise ValueError("evaluation without distribution identity must use sample 0")
            return
        if not _is_sha256(self.sample_digest):
            raise ValueError("evaluation sample_digest must be a lowercase SHA-256 digest")


class EpisodeUnavailableError(RuntimeError):
    """The provider is installed, but optional episode data is unavailable."""


@runtime_checkable
class EpisodeProvider(Protocol):
    """Prepare and privately evaluate episodes for the DimOS E2E runner."""

    provider_name: str
    supported_family_ids: tuple[str, ...]

    def prepare(
        self,
        case: EvaluationCase,
        output_dir: Path,
        *,
        sample_index: int | None = None,
    ) -> PreparedEpisode: ...

    def validate_activation(
        self,
        episode: PreparedEpisode,
        sample_index: int,
    ) -> None: ...

    def start(self, episode: PreparedEpisode) -> EpisodeActivationResult: ...

    def activate(
        self,
        episode: PreparedEpisode,
        sample_index: int,
    ) -> EpisodeActivationResult: ...

    def evaluate(self, episode: PreparedEpisode) -> EpisodeEvaluationResult: ...

    def stop(self) -> None: ...


def load_episode_provider(name: str) -> EpisodeProvider:
    provider_name = _required_text(name, "episode provider name")
    matches = tuple(
        entry
        for entry in importlib_metadata.entry_points().select(group=ENTRY_POINT_GROUP)
        if entry.name == provider_name
    )
    if not matches:
        available = sorted(
            entry.name
            for entry in importlib_metadata.entry_points().select(group=ENTRY_POINT_GROUP)
        )
        raise KeyError(
            f"unknown simulation episode provider {provider_name!r}; available: "
            f"{', '.join(available) or 'none'}"
        )
    if len(matches) != 1:
        raise RuntimeError(f"multiple entry points register episode provider {provider_name!r}")
    loaded = matches[0].load()
    provider = loaded() if isinstance(loaded, type) else loaded
    if not isinstance(provider, EpisodeProvider):
        raise TypeError(f"episode provider {provider_name!r} does not implement the contract")
    if provider.provider_name != provider_name:
        raise ValueError(
            f"episode provider entry point {provider_name!r} loaded {provider.provider_name!r}"
        )
    return provider


def _is_sha256(value: str) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _scalar_mapping(
    values: Mapping[str, str | int | float | bool],
    label: str,
) -> Mapping[str, str | int | float | bool]:
    normalized: dict[str, str | int | float | bool] = {}
    for key, value in values.items():
        name = _required_text(str(key), f"{label} provenance key")
        if not isinstance(value, str | int | float | bool):
            raise TypeError(f"{label} provenance values must be scalar")
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError(f"{label} provenance floats must be finite")
        normalized[name] = value
    return MappingProxyType(normalized)


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
    "ProviderEpisodeRequestContract",
    "TrialIsolationMode",
    "load_episode_provider",
]
