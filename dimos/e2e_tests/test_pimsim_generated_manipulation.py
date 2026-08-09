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

import time
from typing import Protocol, cast

import pytest

from dimos.agents.skill_result import SkillResult
from dimos.e2e_tests.episode import EpisodeRun
from dimos.simulation.episodes import EvaluationCase, load_episode_provider

pytestmark = [pytest.mark.self_hosted_large, pytest.mark.mujoco]


class _PickAndPlaceActions(Protocol):
    def look(self) -> SkillResult: ...

    def pick(self, target: str) -> SkillResult: ...

    def drop_on(self, target: str, *, z_offset: float) -> SkillResult: ...


TABLETOP_CASES = {
    "lift-object": EvaluationCase(
        case_id="lift-object/alphabet-soup/scene-290/variation-3",
        family_id="lift-object",
        scene_seed=290,
        variation_seed=3,
        robot_model="xarm7",
        blueprint_name="xarm-perception-sim",
        role_constraints={"object": "alphabet-soup"},
        required_modules=("PickAndPlaceModule",),
    ),
    "lift-object-second-reset": EvaluationCase(
        case_id="lift-object/alphabet-soup/scene-290/variation-65",
        family_id="lift-object",
        scene_seed=290,
        variation_seed=65,
        robot_model="xarm7",
        blueprint_name="xarm-perception-sim",
        role_constraints={"object": "alphabet-soup"},
        required_modules=("PickAndPlaceModule",),
    ),
    "object-in-receptacle": EvaluationCase(
        case_id="object-in-receptacle/alphabet-soup/wooden-tray/scene-296/variation-3",
        family_id="object-in-receptacle",
        scene_seed=296,
        variation_seed=3,
        robot_model="xarm7",
        blueprint_name="xarm-perception-sim",
        role_constraints={"object": "alphabet-soup", "target": "wooden-tray"},
        required_modules=("PickAndPlaceModule",),
    ),
    "object-on-support": EvaluationCase(
        case_id="object-on-support/tomato-sauce/plate/scene-296/variation-3",
        family_id="object-on-support",
        scene_seed=296,
        variation_seed=3,
        robot_model="xarm7",
        blueprint_name="xarm-perception-sim",
        role_constraints={"object": "tomato-sauce", "target": "plate"},
        required_modules=("PickAndPlaceModule",),
    ),
    "collect-objects-in-receptacle": EvaluationCase(
        case_id=("collect-objects-in-receptacle/soup-and-cheese/wooden-tray/scene-296/variation-3"),
        family_id="collect-objects-in-receptacle",
        scene_seed=296,
        variation_seed=3,
        robot_model="xarm7",
        blueprint_name="xarm-perception-sim",
        role_constraints={
            "first_object": "alphabet-soup",
            "second_object": "cream-cheese",
            "target": "wooden-tray",
        },
        required_modules=("PickAndPlaceModule",),
    ),
    "rearrange-objects": EvaluationCase(
        case_id="rearrange-objects/soup-in-tray/sauce-on-plate/scene-296/variation-3",
        family_id="rearrange-objects",
        scene_seed=296,
        variation_seed=3,
        robot_model="xarm7",
        blueprint_name="xarm-perception-sim",
        role_constraints={
            "first_object": "alphabet-soup",
            "second_object": "tomato-sauce",
            "containment_target": "wooden-tray",
            "support_target": "plate",
        },
        required_modules=("PickAndPlaceModule",),
    ),
}

PLACE_CASES = (
    pytest.param(TABLETOP_CASES["object-in-receptacle"], 0.10, id="object-in-receptacle"),
    pytest.param(TABLETOP_CASES["object-on-support"], 0.08, id="object-on-support"),
)


def test_supported_task_families_have_maintained_e2e_cases() -> None:
    provider = load_episode_provider("pimsim")
    maintained_family_ids = {case.family_id for case in TABLETOP_CASES.values()}

    assert maintained_family_ids == set(provider.supported_family_ids)


@pytest.mark.parametrize(
    "evaluation_episode",
    [
        pytest.param(TABLETOP_CASES["lift-object"], id="scene-290"),
        pytest.param(TABLETOP_CASES["lift-object-second-reset"], id="variation-65"),
    ],
    indirect=True,
)
def test_lift_object(evaluation_episode: EpisodeRun) -> None:
    actions = _actions(evaluation_episode)
    observation = _wait_for_role(evaluation_episode, actions, "object")
    assert observation.success, observation

    result = actions.pick(evaluation_episode.role("object"))
    evaluation = evaluation_episode.wait_for_goal()

    assert result.success, f"{result}; private evaluation: {evaluation}"
    assert evaluation.passed is True


@pytest.mark.parametrize(
    ("evaluation_episode", "z_offset"),
    PLACE_CASES,
    indirect=("evaluation_episode",),
)
def test_place_object(
    evaluation_episode: EpisodeRun,
    z_offset: float,
) -> None:
    actions = _actions(evaluation_episode)
    observation = _wait_for_role(evaluation_episode, actions, "object")
    assert observation.success, observation
    assert evaluation_episode.role("target") in observation.message, observation

    pick_result = actions.pick(evaluation_episode.role("object"))
    assert pick_result.success, pick_result
    place_result = actions.drop_on(
        evaluation_episode.role("target"),
        z_offset=z_offset,
    )
    evaluation = evaluation_episode.wait_for_goal()

    assert place_result.success, f"{place_result}; private evaluation: {evaluation}"
    assert evaluation.passed is True


@pytest.mark.parametrize(
    "evaluation_episode",
    [pytest.param(TABLETOP_CASES["collect-objects-in-receptacle"], id="collect-objects")],
    indirect=True,
)
def test_collect_objects(evaluation_episode: EpisodeRun) -> None:
    actions = _actions(evaluation_episode)
    for role_id in ("first_object", "second_object"):
        observation = _wait_for_role(evaluation_episode, actions, role_id)
        assert observation.success, observation
        pick_result = actions.pick(evaluation_episode.role(role_id))
        assert pick_result.success, pick_result
        place_result = actions.drop_on(
            evaluation_episode.role("target"),
            z_offset=0.10,
        )
        assert place_result.success, place_result

    evaluation = evaluation_episode.wait_for_goal()
    assert evaluation.passed is True, evaluation


@pytest.mark.parametrize(
    "evaluation_episode",
    [pytest.param(TABLETOP_CASES["rearrange-objects"], id="rearrange-objects")],
    indirect=True,
)
def test_rearrange_objects(evaluation_episode: EpisodeRun) -> None:
    actions = _actions(evaluation_episode)
    observation = _wait_for_role(evaluation_episode, actions, "first_object")
    assert observation.success, observation
    first_pick = actions.pick(evaluation_episode.role("first_object"))
    assert first_pick.success, first_pick
    first_place = actions.drop_on(
        evaluation_episode.role("containment_target"),
        z_offset=0.10,
    )
    assert first_place.success, first_place

    observation = _wait_for_role(evaluation_episode, actions, "second_object")
    assert observation.success, observation
    second_pick = actions.pick(evaluation_episode.role("second_object"))
    assert second_pick.success, second_pick
    second_place = actions.drop_on(
        evaluation_episode.role("support_target"),
        z_offset=0.08,
    )
    evaluation = evaluation_episode.wait_for_goal()

    assert second_place.success, f"{second_place}; private evaluation: {evaluation}"
    assert evaluation.passed is True


def _wait_for_role(
    episode: EpisodeRun,
    actions: _PickAndPlaceActions,
    role_id: str,
    timeout: float = 20.0,
) -> SkillResult:
    name = episode.role(role_id)
    deadline = time.monotonic() + timeout
    last_observation = None
    while time.monotonic() < deadline:
        last_observation = actions.look()
        if name in last_observation.message:
            return last_observation
        time.sleep(0.25)
    pytest.fail(f"role {role_id!r} ({name!r}) was not observed: {last_observation}")


def _actions(episode: EpisodeRun) -> _PickAndPlaceActions:
    return cast("_PickAndPlaceActions", episode.app.get_module("PickAndPlaceModule"))
