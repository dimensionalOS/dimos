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

"""Provider-neutral topology-aware trial schedule contracts."""

from dataclasses import dataclass, replace
from pathlib import Path
from typing import ClassVar

import pytest

from dimos.sim2.evaluation import EpisodeProvider, TrialIsolationMode
from dimos.sim2.scheduling import (
    EpisodeScheduleResolver,
    EpisodeTrialBatch,
    PreparedEpisodeSchedule,
    ScheduledEpisodeTrial,
)


@dataclass(frozen=True)
class _Request:
    case_id: str
    provider_name: ClassVar[str] = "test-provider"


class _Resolver:
    provider_name = "test-provider"

    def supports(self, request: object) -> bool:
        return isinstance(request, _Request)

    def prepare(
        self,
        request: object,
        *,
        trial_count: int,
        output_dir: Path,
    ) -> PreparedEpisodeSchedule:
        del request, trial_count, output_dir
        return _single_topology_schedule()

    def close(self) -> None:
        return None


def test_schedule_resolver_is_separate_from_episode_provider_lifecycle() -> None:
    resolver = _Resolver()

    assert isinstance(resolver, EpisodeScheduleResolver)
    assert not isinstance(resolver, EpisodeProvider)
    assert not hasattr(resolver, "activate")
    assert not hasattr(resolver, "evaluate")


def test_prepared_schedule_covers_each_interactive_eval_trial_exactly_once() -> None:
    schedule = _single_topology_schedule()

    assert tuple(trial.trial_index for trial in schedule.trials) == (0, 1, 2)
    assert tuple(
        sorted(trial.trial_index for batch in schedule.batches for trial in batch.trials)
    ) == (0, 1, 2)


def test_topology_batches_do_not_change_trial_identity_or_aggregate_order() -> None:
    schedule = _cross_topology_schedule()

    assert tuple(trial.record_id for trial in schedule.trials) == ("record-b", "record-a")
    assert tuple(trial.record_id for batch in schedule.batches for trial in batch.trials) == (
        "record-a",
        "record-b",
    )


def test_each_batch_supplies_one_provider_request_for_its_resident_topology() -> None:
    schedule = _cross_topology_schedule()

    assert tuple(batch.episode_request.case_id for batch in schedule.batches) == (
        "batch-a",
        "batch-b",
    )
    assert all(
        batch.episode_request.provider_name == schedule.provider_name for batch in schedule.batches
    )


def test_runner_uses_process_isolation_only_for_topology_changes() -> None:
    assert _single_topology_schedule().batches[0].isolation is TrialIsolationMode.EPISODE_BOUNDARY
    assert all(
        batch.isolation is TrialIsolationMode.PROCESS
        for batch in _cross_topology_schedule().batches
    )

    cross = _cross_topology_schedule()
    invalid_batches = tuple(
        replace(batch, isolation=TrialIsolationMode.EPISODE_BOUNDARY) for batch in cross.batches
    )
    with pytest.raises(ValueError, match="cross-topology"):
        replace(cross, batches=invalid_batches)


def test_schedule_retains_population_manifest_shard_record_and_topology_identity() -> None:
    trial = _single_topology_schedule().trials[0]

    assert trial.population_id == "population"
    assert trial.population_manifest_digest == "a" * 64
    assert trial.shard_id == "shard-a"
    assert trial.record_id == "record-0"
    assert trial.record_digest == "b" * 64
    assert trial.topology_group_id == "topology-a"


def test_schedule_can_reference_multiple_immutable_population_pack_manifests() -> None:
    schedule = _cross_topology_schedule()

    assert schedule.population_manifest_digests == ("a" * 64, "c" * 64)
    assert {trial.population_manifest_digest for trial in schedule.trials} == {
        "a" * 64,
        "c" * 64,
    }


def _trial(
    *,
    trial_index: int,
    execution_index: int,
    sample_index: int,
    record_id: str,
    topology_group_id: str,
    manifest_digest: str = "a" * 64,
    shard_id: str = "shard-a",
) -> ScheduledEpisodeTrial:
    return ScheduledEpisodeTrial(
        population_id="population",
        population_revision="1",
        population_manifest_digest=manifest_digest,
        shard_id=shard_id,
        trial_index=trial_index,
        execution_index=execution_index,
        sample_index=sample_index,
        record_id=record_id,
        record_digest="b" * 64,
        topology_group_id=topology_group_id,
    )


def _single_topology_schedule() -> PreparedEpisodeSchedule:
    trials = tuple(
        _trial(
            trial_index=index,
            execution_index=index,
            sample_index=index,
            record_id=f"record-{index}",
            topology_group_id="topology-a",
        )
        for index in range(3)
    )
    batch = EpisodeTrialBatch(
        batch_id="batch-a",
        topology_group_id="topology-a",
        isolation=TrialIsolationMode.EPISODE_BOUNDARY,
        episode_request=_Request("batch-a"),
        trials=trials,
    )
    return PreparedEpisodeSchedule(
        provider_name="test-provider",
        request_case_id="population",
        population_id="population",
        population_revision="1",
        population_manifest_digests=("a" * 64,),
        schedule_seed=7,
        schedule_digest="d" * 64,
        trials=trials,
        batches=(batch,),
    )


def _cross_topology_schedule() -> PreparedEpisodeSchedule:
    statistical = (
        _trial(
            trial_index=0,
            execution_index=1,
            sample_index=0,
            record_id="record-b",
            topology_group_id="topology-b",
            manifest_digest="c" * 64,
            shard_id="shard-b",
        ),
        _trial(
            trial_index=1,
            execution_index=0,
            sample_index=0,
            record_id="record-a",
            topology_group_id="topology-a",
        ),
    )
    by_topology = {trial.topology_group_id: trial for trial in statistical}
    batches = tuple(
        EpisodeTrialBatch(
            batch_id=f"batch-{suffix}",
            topology_group_id=topology,
            isolation=TrialIsolationMode.PROCESS,
            episode_request=_Request(f"batch-{suffix}"),
            trials=(by_topology[topology],),
        )
        for suffix, topology in (("a", "topology-a"), ("b", "topology-b"))
    )
    return PreparedEpisodeSchedule(
        provider_name="test-provider",
        request_case_id="population",
        population_id="population",
        population_revision="1",
        population_manifest_digests=("a" * 64, "c" * 64),
        schedule_seed=7,
        schedule_digest="d" * 64,
        trials=statistical,
        batches=batches,
    )
