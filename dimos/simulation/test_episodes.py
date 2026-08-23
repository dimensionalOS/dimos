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

from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

import pytest

from dimos.e2e_tests.episode import evaluate_episode, prepare_episode, reset_episode
from dimos.sim2.episodes import PublicEpisodeContext, PublicEpisodeRole
from dimos.simulation import episodes
from dimos.simulation.episodes import (
    EpisodeEvaluationResult,
    EpisodeResetResult,
    EvaluationCase,
    PreparedEpisode,
)


@dataclass(frozen=True)
class _Request:
    case_id: str


def _case() -> EvaluationCase:
    return EvaluationCase(
        episode_request=_Request("place/cup/tray/scene-7/variation-2"),
        blueprint_name="test-arm-sim",
        required_modules=("PublicActions",),
        required_roles=("object", "target"),
    )


class _FakeProvider:
    provider_name: str = "fake"
    supported_family_ids: tuple[str, ...] = ("object-in-receptacle",)

    def __init__(self) -> None:
        self.active = False

    def prepare(self, case: EvaluationCase, output_dir: Path) -> PreparedEpisode:
        assert output_dir.name == "episode"
        return PreparedEpisode(
            provider_name=self.provider_name,
            episode_id="fake-episode-1",
            case_id=case.case_id,
            blueprint_name=case.blueprint_name,
            simulator="fake-simulator",
            context=PublicEpisodeContext(
                case_id=case.case_id,
                instruction="Put the cup in the tray.",
                roles={
                    "object": PublicEpisodeRole(role_id="object", entity_id="cup-1", name="cup"),
                    "target": PublicEpisodeRole(role_id="target", entity_id="tray-1", name="tray"),
                },
            ),
            global_args=("--transport", "zenoh"),
            required_modules=(*case.required_modules, "PrivateEvaluator"),
        )

    def start(self, episode: PreparedEpisode) -> None:
        assert episode.episode_id == "fake-episode-1"
        self.active = True

    def reset(self, episode: PreparedEpisode) -> EpisodeResetResult:
        assert self.active
        return EpisodeResetResult(
            provider_name=self.provider_name,
            episode_id=episode.episode_id,
            case_id=episode.case_id,
            initial_conditions_passed=True,
        )

    def evaluate(self, episode: PreparedEpisode) -> EpisodeEvaluationResult:
        assert self.active
        return EpisodeEvaluationResult(
            provider_name=self.provider_name,
            episode_id=episode.episode_id,
            case_id=episode.case_id,
            passed=True,
            summary="object is in target",
            metrics={"sim_time": 1.5},
        )

    def stop(self) -> None:
        self.active = False


class _EntryPoint:
    name = "fake"

    def load(self) -> Any:
        return _FakeProvider


def test_generic_episode_flow_uses_only_dimos_contracts(tmp_path: Path) -> None:
    provider = _FakeProvider()
    case = _case()

    episode = prepare_episode(provider, case, tmp_path / "episode")
    provider.start(episode)
    reset = reset_episode(provider, episode)
    result = evaluate_episode(provider, episode)
    provider.stop()

    assert episode.required_modules == ("PublicActions", "PrivateEvaluator")
    assert reset.initial_conditions_passed is True
    assert result.passed is True
    assert result.metrics == {"sim_time": 1.5}
    assert provider.active is False


def test_evaluation_case_references_provider_selection_without_copying_it() -> None:
    case = _case()

    assert tuple(field.name for field in fields(EvaluationCase)) == (
        "episode_request",
        "blueprint_name",
        "required_modules",
        "required_roles",
    )
    assert case.case_id == case.episode_request.case_id


def test_load_episode_provider_uses_installed_factory(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def entry_points(*, group: str, name: str | None = None) -> list[_EntryPoint]:
        assert group == episodes.ENTRY_POINT_GROUP
        return [_EntryPoint()] if name in (None, "fake") else []

    monkeypatch.setattr(
        "dimos.simulation.episodes.importlib_metadata.entry_points",
        entry_points,
    )

    provider = episodes.load_episode_provider("fake")

    assert isinstance(provider, _FakeProvider)
    assert provider.supported_family_ids == ("object-in-receptacle",)


def test_episode_flow_rejects_provider_identity_changes(tmp_path: Path) -> None:
    class WrongProvider(_FakeProvider):
        def prepare(self, case: EvaluationCase, output_dir: Path) -> PreparedEpisode:
            prepared = super().prepare(case, output_dir)
            return PreparedEpisode(
                provider_name="other",
                episode_id=prepared.episode_id,
                case_id=prepared.case_id,
                blueprint_name=prepared.blueprint_name,
                simulator=prepared.simulator,
                context=prepared.context,
            )

    with pytest.raises(ValueError, match="prepared an episode for"):
        prepare_episode(WrongProvider(), _case(), tmp_path / "episode")


def test_episode_contract_rejects_invalid_values() -> None:
    with pytest.raises(ValueError, match="episode request case_id must not be empty"):
        EvaluationCase(
            episode_request=_Request(""),
            blueprint_name="arm-sim",
        )

    with pytest.raises(ValueError, match="successful reset"):
        EpisodeResetResult(
            provider_name="fake",
            episode_id="episode",
            case_id="case",
            initial_conditions_passed=True,
            failed_conditions=("object fell",),
        )
