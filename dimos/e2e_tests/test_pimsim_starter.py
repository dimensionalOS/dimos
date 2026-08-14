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

from typing import Any, Protocol, cast

from pimsim.episodes import prepare_episode as prepare_pimsim_episode
import pytest

from dimos.agents.skill_result import SkillResult
from dimos.e2e_tests.episode import EpisodeRun
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.simulation.episodes import EvaluationCase
from pimsim import EpisodeRequest, TaskRequest, WorldRequest, installed_episode_catalog

pytestmark = [pytest.mark.self_hosted_large, pytest.mark.mujoco]


class _EpisodeControl(Protocol):
    def inspect_episode(self) -> dict[str, Any]: ...

    def evaluate_goal(self) -> dict[str, Any]: ...


class _Navigation(Protocol):
    def set_goal(self, goal: PoseStamped) -> bool: ...

    def cancel_goal(self) -> bool: ...


class _PickAndPlace(Protocol):
    def look(self) -> SkillResult: ...


STARTER_NAVIGATION_REQUEST = EpisodeRequest(
    case_id="native-logistics-floor-navigate-to-region-seed-0",
    robot="unitree_g1",
    robot_revision="g1-29dof-v1",
    world=WorldRequest(
        family_id="native-logistics-floor",
        family_revision="1-starter-seed-3",
    ),
    task=TaskRequest(family_id="navigate-to-region", seed=0),
)

STARTER_LIFT_REQUEST = EpisodeRequest(
    case_id="native-tabletop-lift-object-seed-3",
    robot="xarm7",
    robot_revision="1",
    world=WorldRequest(family_id="native-tabletop", family_revision="1"),
    task=TaskRequest(
        family_id="lift-object",
        seed=3,
        roles={"object": "alphabet-soup"},
        bindings={"surface": "surface"},
    ),
)

STARTER_NAVIGATION_CASE = EvaluationCase(
    episode_request=STARTER_NAVIGATION_REQUEST,
    blueprint_name="unitree-g1-groot-wbc",
    required_modules=("ReplanningAStarPlanner",),
    required_roles=("destination",),
)

STARTER_LIFT_CASE = EvaluationCase(
    episode_request=STARTER_LIFT_REQUEST,
    blueprint_name="xarm-perception-sim",
    required_modules=("PickAndPlaceModule",),
    required_roles=("object", "surface"),
)


@pytest.mark.parametrize(
    "evaluation_episode",
    [pytest.param(STARTER_NAVIGATION_CASE, id="g1-navigation")],
    indirect=True,
)
def test_starter_navigation_uses_shared_scene_and_task_request(
    evaluation_episode: EpisodeRun,
) -> None:
    _assert_shared_identity(evaluation_episode, STARTER_NAVIGATION_REQUEST)
    planner = cast(
        "_Navigation",
        evaluation_episode.app.get_module("ReplanningAStarPlanner"),
    )

    try:
        accepted = planner.set_goal(PoseStamped(frame_id="map", position=(4.6, 0.0, 0.0)))
    finally:
        planner.cancel_goal()

    assert accepted is True


@pytest.mark.parametrize(
    "evaluation_episode",
    [pytest.param(STARTER_LIFT_CASE, id="xarm-lift")],
    indirect=True,
)
def test_starter_manipulation_uses_shared_scene_and_task_request(
    evaluation_episode: EpisodeRun,
) -> None:
    _assert_shared_identity(evaluation_episode, STARTER_LIFT_REQUEST)
    actions = cast(
        "_PickAndPlace",
        evaluation_episode.app.get_module("PickAndPlaceModule"),
    )

    observation = actions.look()

    assert observation.success, observation
    assert evaluation_episode.role("object") in observation.message


def _assert_shared_identity(run: EpisodeRun, request: EpisodeRequest) -> None:
    control = cast("_EpisodeControl", run.app.get_module("PimSimEpisodeControl"))
    inspected = control.inspect_episode()
    prepared = prepare_pimsim_episode(request)
    catalog = installed_episode_catalog()
    scene_id = request.world.family_id or ""
    scene = catalog.scene(scene_id)
    canonical_request = catalog.scene_task_request(
        scene_id,
        request.robot,
        request.task.family_id,
        seed=request.task.seed,
        roles=request.task.roles,
        bindings=request.task.bindings,
        parameters=request.task.parameters,
    )
    scenario = inspected["scenario"]
    episode = inspected["episode"]

    assert request == canonical_request
    assert episode["case_id"] == request.case_id
    assert episode["request_digest"] == request.digest
    assert episode["episode_digest"] == prepared.digest
    assert episode["exact_world_digest"] == scene.world_digest
    assert scenario["seed"] == request.task.seed
    assert scenario["scenario_id"] == request.case_id
    assert scenario["metadata"]["family_id"] == request.task.family_id
    assert scenario["instruction"] == prepared.scenario.instruction
    assert scenario["goal_predicate"] == prepared.scenario.to_json_dict()["goal_predicate"]
