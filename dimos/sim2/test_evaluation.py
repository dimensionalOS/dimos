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

"""Provider-neutral distribution lifecycle verification skeleton."""

from pathlib import Path

from dimos.e2e_tests.episode import activate_episode, start_episode
from dimos.e2e_tests.test_episode_provider_lifecycle import (
    _Provider,
    _digest,
    _evaluation_case,
    _prepared_episode,
)
from dimos.sim2.evaluation import EpisodeBoundary, TrialIsolationMode


def test_prepared_episode_declares_bounded_distribution_identity(tmp_path: Path) -> None:
    provider = _Provider()

    episode = provider.prepare(_evaluation_case(), tmp_path, sample_index=3)

    assert episode.distribution_id == "bounded-distribution"
    assert episode.distribution_revision == "1"
    assert episode.distribution_seed == 20260824
    assert episode.distribution_digest == "a" * 64
    assert episode.initial_sample_index == 3
    assert episode.initial_sample_digest == _digest(3)
    assert episode.bounded_sample_count == 8


def test_activate_is_the_only_provider_sample_transition() -> None:
    provider = _Provider()
    episode = _prepared_episode()
    start_episode(provider, episode)

    result = activate_episode(provider, episode, 1)

    assert result.sample_index == 1
    assert provider.activate_calls == [1]
    assert not hasattr(provider, "reset")
    assert not hasattr(provider, "resample")


def test_episode_boundary_round_trips_without_provider_private_state() -> None:
    boundary = EpisodeBoundary(
        provider_name="fake",
        episode_id="episode-1",
        previous_sample_index=2,
        sample_index=3,
        sequence=4,
    )

    encoded = boundary.to_wire_dict()

    assert EpisodeBoundary.from_wire_dict(encoded) == boundary
    assert encoded == {
        "provider_name": "fake",
        "episode_id": "episode-1",
        "previous_sample_index": 2,
        "sample_index": 3,
        "sequence": 4,
    }
    assert not ({"context", "oracle", "provenance", "randomization"} & encoded.keys())


def test_activation_returns_a_new_public_context_for_each_sample() -> None:
    provider = _Provider()
    episode = _prepared_episode()

    first = start_episode(provider, episode)
    second = activate_episode(provider, episode, 1)

    assert first.context is not second.context
    assert first.context.role("object").entity_id == "object-0"
    assert second.context.role("object").entity_id == "object-1"
    assert first.sample_digest != second.sample_digest


def test_process_isolation_and_episode_boundary_isolation_are_explicit() -> None:
    assert tuple(TrialIsolationMode) == (
        TrialIsolationMode.EPISODE_BOUNDARY,
        TrialIsolationMode.PROCESS,
    )
    assert TrialIsolationMode("episode-boundary") is TrialIsolationMode.EPISODE_BOUNDARY
    assert TrialIsolationMode("process") is TrialIsolationMode.PROCESS
