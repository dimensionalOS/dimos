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

"""Provider-neutral topology-aware scheduling before episode worker launch."""

from __future__ import annotations

from dataclasses import dataclass
import importlib.metadata as importlib_metadata
from pathlib import Path
from typing import Protocol, runtime_checkable

from dimos.sim2.evaluation import ProviderEpisodeRequestContract, TrialIsolationMode

EPISODE_SCHEDULE_RESOLVER_ENTRY_POINT_GROUP = "dimos.simulation.episode_schedule_resolvers"


@runtime_checkable
class ScheduledEpisodeRequestContract(ProviderEpisodeRequestContract, Protocol):
    """Provider request whose finite trials must be resolved before launch."""

    @property
    def schedule_seed(self) -> int: ...

    @property
    def start_index(self) -> int: ...


@dataclass(frozen=True, kw_only=True)
class ScheduledEpisodeTrial:
    population_id: str
    population_revision: str
    population_manifest_digest: str
    shard_id: str
    trial_index: int
    execution_index: int
    sample_index: int
    record_id: str
    record_digest: str
    topology_group_id: str
    topology_digest: str | None = None

    def __post_init__(self) -> None:
        for field_name in (
            "population_id",
            "population_revision",
            "shard_id",
            "record_id",
            "topology_group_id",
        ):
            object.__setattr__(
                self,
                field_name,
                _required_text(getattr(self, field_name), field_name.replace("_", " ")),
            )
        for field_name in ("trial_index", "execution_index", "sample_index"):
            _nonnegative_integer(getattr(self, field_name), field_name.replace("_", " "))
        for value, label in (
            (self.population_manifest_digest, "population manifest digest"),
            (self.record_digest, "episode record digest"),
        ):
            _require_sha256(value, label)
        if self.topology_digest is not None:
            _require_sha256(self.topology_digest, "prepared topology digest")


@dataclass(frozen=True, kw_only=True)
class EpisodeTrialBatch:
    batch_id: str
    topology_group_id: str
    isolation: TrialIsolationMode
    episode_request: ProviderEpisodeRequestContract
    trials: tuple[ScheduledEpisodeTrial, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "batch_id", _required_text(self.batch_id, "batch ID"))
        object.__setattr__(
            self,
            "topology_group_id",
            _required_text(self.topology_group_id, "topology group ID"),
        )
        object.__setattr__(self, "isolation", TrialIsolationMode(self.isolation))
        if not isinstance(self.episode_request, ProviderEpisodeRequestContract):
            raise TypeError("episode trial batch requires a provider episode request")
        trials = tuple(self.trials)
        if not trials:
            raise ValueError("episode trial batch requires at least one trial")
        if any(trial.topology_group_id != self.topology_group_id for trial in trials):
            raise ValueError("episode trial batch contains another topology group")
        if any(trial.sample_index != index for index, trial in enumerate(trials)):
            raise ValueError("topology batch sample indexes must be contiguous from zero")
        object.__setattr__(self, "trials", trials)


@dataclass(frozen=True, kw_only=True)
class PreparedEpisodeSchedule:
    provider_name: str
    request_case_id: str
    population_id: str
    population_revision: str
    population_manifest_digests: tuple[str, ...]
    schedule_seed: int
    schedule_digest: str
    trials: tuple[ScheduledEpisodeTrial, ...]
    batches: tuple[EpisodeTrialBatch, ...]

    def __post_init__(self) -> None:
        for field_name in (
            "provider_name",
            "request_case_id",
            "population_id",
            "population_revision",
        ):
            object.__setattr__(
                self,
                field_name,
                _required_text(getattr(self, field_name), field_name.replace("_", " ")),
            )
        _nonnegative_integer(self.schedule_seed, "schedule seed")
        _require_sha256(self.schedule_digest, "schedule digest")
        manifests = tuple(self.population_manifest_digests)
        if not manifests:
            raise ValueError("prepared schedule requires population manifest identity")
        for value in manifests:
            _require_sha256(value, "population manifest digest")
        if len(manifests) != len(set(manifests)):
            raise ValueError("population manifest digests must be unique")
        trials = tuple(self.trials)
        if not trials:
            raise ValueError("prepared schedule requires at least one trial")
        if tuple(trial.trial_index for trial in trials) != tuple(range(len(trials))):
            raise ValueError("prepared schedule trials must retain statistical order")
        if {trial.execution_index for trial in trials} != set(range(len(trials))):
            raise ValueError("prepared schedule execution indexes must be a permutation")
        if any(
            trial.population_id != self.population_id
            or trial.population_revision != self.population_revision
            or trial.population_manifest_digest not in manifests
            for trial in trials
        ):
            raise ValueError("prepared schedule trial identity disagrees with the schedule")
        batches = tuple(self.batches)
        if not batches:
            raise ValueError("prepared schedule requires at least one topology batch")
        batched = tuple(trial for batch in batches for trial in batch.trials)
        if tuple(sorted(batched, key=lambda value: value.trial_index)) != trials:
            raise ValueError("prepared schedule batches must cover every trial exactly once")
        if any(batch.episode_request.provider_name != self.provider_name for batch in batches):
            raise ValueError("prepared schedule batch provider does not match")
        group_ids = tuple(batch.topology_group_id for batch in batches)
        if len(group_ids) != len(set(group_ids)):
            raise ValueError("prepared schedule requires one batch per topology group")
        if len(batches) > 1 and any(
            batch.isolation is not TrialIsolationMode.PROCESS for batch in batches
        ):
            raise ValueError("cross-topology schedule batches require process isolation")
        object.__setattr__(self, "population_manifest_digests", manifests)
        object.__setattr__(self, "trials", trials)
        object.__setattr__(self, "batches", batches)


@runtime_checkable
class EpisodeScheduleResolver(Protocol):
    """Resolve/materialize a finite schedule outside episode runtime workers."""

    provider_name: str

    def supports(self, request: ProviderEpisodeRequestContract) -> bool: ...

    def prepare(
        self,
        request: ProviderEpisodeRequestContract,
        *,
        trial_count: int,
        output_dir: Path,
    ) -> PreparedEpisodeSchedule: ...

    def close(self) -> None: ...


def load_episode_schedule_resolver(provider_name: str) -> EpisodeScheduleResolver:
    name = _required_text(provider_name, "episode schedule provider name")
    matches = tuple(
        entry
        for entry in importlib_metadata.entry_points().select(
            group=EPISODE_SCHEDULE_RESOLVER_ENTRY_POINT_GROUP
        )
        if entry.name == name
    )
    if not matches:
        raise KeyError(f"no episode schedule resolver is registered for {name!r}")
    if len(matches) != 1:
        raise RuntimeError(f"multiple episode schedule resolvers are registered for {name!r}")
    loaded = matches[0].load()
    resolver = loaded() if isinstance(loaded, type) else loaded
    if not isinstance(resolver, EpisodeScheduleResolver):
        raise TypeError(f"episode schedule resolver {name!r} does not implement the contract")
    if resolver.provider_name != name:
        raise ValueError(f"episode schedule resolver {name!r} loaded {resolver.provider_name!r}")
    return resolver


def _required_text(value: str, label: str) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{label} must be a string")
    normalized = value.strip()
    if not normalized:
        raise ValueError(f"{label} must not be empty")
    return normalized


def _nonnegative_integer(value: int, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} must be a non-negative integer")
    return value


def _require_sha256(value: str, label: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise ValueError(f"{label} must be a lowercase SHA-256 digest")
    return value


__all__ = [
    "EPISODE_SCHEDULE_RESOLVER_ENTRY_POINT_GROUP",
    "EpisodeScheduleResolver",
    "EpisodeTrialBatch",
    "PreparedEpisodeSchedule",
    "ScheduledEpisodeRequestContract",
    "ScheduledEpisodeTrial",
    "load_episode_schedule_resolver",
]
