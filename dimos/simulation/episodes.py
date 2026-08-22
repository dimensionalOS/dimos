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
import importlib.metadata as importlib_metadata
import math
from pathlib import Path
from types import MappingProxyType
from typing import Any, Protocol, runtime_checkable

ENTRY_POINT_GROUP = "dimos.simulation.episode_providers"


def _required_text(value: str, label: str) -> str:
    normalized = value.strip()
    if not normalized:
        raise ValueError(f"{label} must not be empty")
    return normalized


def _text_mapping(values: Mapping[str, str], label: str) -> Mapping[str, str]:
    normalized = {
        _required_text(str(key), f"{label} key"): _required_text(str(value), f"{label} value")
        for key, value in values.items()
    }
    return MappingProxyType(normalized)


def _unique_text(values: tuple[str, ...], label: str) -> tuple[str, ...]:
    normalized = tuple(_required_text(value, label) for value in values)
    if len(normalized) != len(set(normalized)):
        raise ValueError(f"{label}s must be unique")
    return normalized


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
        if not isinstance(self.episode_request, EpisodeRequestContract):
            raise TypeError("evaluation case episode_request must provide case_id")
        _required_text(self.episode_request.case_id, "episode request case_id")
        object.__setattr__(
            self, "blueprint_name", _required_text(self.blueprint_name, "blueprint_name")
        )
        object.__setattr__(
            self,
            "required_modules",
            _unique_text(self.required_modules, "required module"),
        )
        object.__setattr__(
            self,
            "required_roles",
            _unique_text(self.required_roles, "required role"),
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
    global_args: tuple[str, ...] = ()
    extra_env: Mapping[str, str] = field(default_factory=dict)
    required_modules: tuple[str, ...] = ()
    role_names: Mapping[str, str] = field(default_factory=dict)
    role_reset_positions: Mapping[str, tuple[float, float, float]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for name in ("provider_name", "episode_id", "case_id", "blueprint_name"):
            object.__setattr__(self, name, _required_text(getattr(self, name), name))
        if self.simulator is not None:
            object.__setattr__(self, "simulator", _required_text(self.simulator, "simulator"))
        object.__setattr__(
            self,
            "global_args",
            tuple(_required_text(value, "global argument") for value in self.global_args),
        )
        object.__setattr__(self, "extra_env", _text_mapping(self.extra_env, "environment"))
        object.__setattr__(
            self,
            "required_modules",
            _unique_text(self.required_modules, "required module"),
        )
        object.__setattr__(self, "role_names", _text_mapping(self.role_names, "role name"))
        positions = {
            _required_text(str(role_id), "role reset position key"): tuple(
                float(value) for value in position
            )
            for role_id, position in self.role_reset_positions.items()
        }
        if any(len(position) != 3 for position in positions.values()):
            raise ValueError("role reset positions must contain three values")
        if any(not math.isfinite(value) for position in positions.values() for value in position):
            raise ValueError("role reset positions must be finite")
        object.__setattr__(self, "role_reset_positions", MappingProxyType(positions))


@dataclass(frozen=True)
class EpisodeResetResult:
    """Typed result of applying an episode's initial physical state."""

    provider_name: str
    episode_id: str
    case_id: str
    initial_conditions_passed: bool
    failed_conditions: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        for name in ("provider_name", "episode_id", "case_id"):
            object.__setattr__(self, name, _required_text(getattr(self, name), name))
        object.__setattr__(
            self,
            "failed_conditions",
            tuple(_required_text(value, "failed condition") for value in self.failed_conditions),
        )
        if self.initial_conditions_passed and self.failed_conditions:
            raise ValueError("a successful reset cannot contain failed conditions")


@dataclass(frozen=True)
class EpisodeEvaluationResult:
    """Private physical evaluation result returned to the DimOS test runner."""

    provider_name: str
    episode_id: str
    case_id: str
    passed: bool
    summary: str
    metrics: Mapping[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for name in ("provider_name", "episode_id", "case_id", "summary"):
            object.__setattr__(self, name, _required_text(getattr(self, name), name))
        metrics = {
            _required_text(str(name), "metric name"): float(value)
            for name, value in self.metrics.items()
        }
        if any(not math.isfinite(value) for value in metrics.values()):
            raise ValueError("episode metrics must be finite")
        object.__setattr__(self, "metrics", MappingProxyType(metrics))


class EpisodeUnavailableError(RuntimeError):
    """The provider is installed, but optional episode data is unavailable."""


@runtime_checkable
class EpisodeProvider(Protocol):
    """Prepare and privately evaluate episodes for the DimOS E2E runner."""

    provider_name: str
    supported_family_ids: tuple[str, ...]

    def prepare(self, case: EvaluationCase, output_dir: Path) -> PreparedEpisode: ...

    def start(self, episode: PreparedEpisode) -> None: ...

    def reset(self, episode: PreparedEpisode) -> EpisodeResetResult: ...

    def evaluate(self, episode: PreparedEpisode) -> EpisodeEvaluationResult: ...

    def stop(self) -> None: ...


def load_episode_provider(name: str) -> EpisodeProvider:
    matches = list(importlib_metadata.entry_points(group=ENTRY_POINT_GROUP, name=name))
    if not matches:
        available = sorted(
            entry_point.name
            for entry_point in importlib_metadata.entry_points(group=ENTRY_POINT_GROUP)
        )
        suffix = f" Available providers: {', '.join(available)}." if available else ""
        raise ValueError(f"Episode provider {name!r} is not installed.{suffix}")
    if len(matches) > 1:
        raise ValueError(f"Episode provider {name!r} is registered more than once")
    factory: Any = matches[0].load()
    if not callable(factory):
        raise TypeError(f"Episode provider {name!r} entry point must be callable")
    provider = factory()
    if not isinstance(provider, EpisodeProvider):
        raise TypeError(
            f"Episode provider {name!r} must implement EpisodeProvider, got {provider!r}"
        )
    if provider.provider_name != name:
        raise ValueError(
            f"Episode provider entry point {name!r} returned provider {provider.provider_name!r}"
        )
    return provider


__all__ = [
    "ENTRY_POINT_GROUP",
    "EpisodeEvaluationResult",
    "EpisodeProvider",
    "EpisodeRequestContract",
    "EpisodeResetResult",
    "EpisodeUnavailableError",
    "EvaluationCase",
    "PreparedEpisode",
    "load_episode_provider",
]
