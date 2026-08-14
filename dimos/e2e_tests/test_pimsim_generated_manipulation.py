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

import json
import math
import time
from typing import Protocol, cast

import pytest

from dimos.agents.skill_result import SkillResult
from dimos.e2e_tests.episode import EpisodeRun
from dimos.simulation.episodes import EvaluationCase, load_episode_provider
from pimsim import EpisodeRequest, TaskRequest, WorldRequest

pytestmark = [pytest.mark.self_hosted_large, pytest.mark.mujoco]


class _PickAndPlaceActions(Protocol):
    def look(self) -> SkillResult: ...

    def pick(self, target: str) -> SkillResult: ...

    def drop_on(self, target: str, *, z_offset: float) -> SkillResult: ...

    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> SkillResult: ...

    def open_gripper(self) -> SkillResult: ...

    def close_gripper(self) -> SkillResult: ...

    def get_gripper(self) -> float | None: ...

    def get_current_joints(self) -> list[float] | None: ...

    def is_collision_free(self, joints: list[float]) -> bool: ...


def _tabletop_case(
    case_id: str,
    family_id: str,
    world_seed: int,
    task_seed: int,
    roles: dict[str, str],
) -> EvaluationCase:
    return EvaluationCase(
        episode_request=EpisodeRequest(
            case_id=case_id,
            robot="xarm7",
            world=WorldRequest(family_id="native-tabletop", seed=world_seed),
            task=TaskRequest(family_id=family_id, seed=task_seed, roles=roles),
        ),
        blueprint_name="xarm-perception-sim",
        required_modules=("PickAndPlaceModule",),
        required_roles=tuple(roles),
    )


TABLETOP_CASES = {
    "lift-object": _tabletop_case(
        "lift-object/alphabet-soup/scene-290/variation-3",
        "lift-object",
        290,
        3,
        {"object": "alphabet-soup"},
    ),
    "lift-object-second-reset": _tabletop_case(
        "lift-object/alphabet-soup/scene-290/variation-65",
        "lift-object",
        290,
        65,
        {"object": "alphabet-soup"},
    ),
    "object-in-receptacle": _tabletop_case(
        "object-in-receptacle/alphabet-soup/wooden-tray/scene-296/variation-3",
        "object-in-receptacle",
        296,
        3,
        {"object": "alphabet-soup", "target": "wooden-tray"},
    ),
    "object-on-support": _tabletop_case(
        "object-on-support/tomato-sauce/plate/scene-296/variation-3",
        "object-on-support",
        296,
        3,
        {"object": "tomato-sauce", "target": "plate"},
    ),
    "collect-objects-in-receptacle": _tabletop_case(
        "collect-objects-in-receptacle/soup-and-cheese/wooden-tray/scene-296/variation-3",
        "collect-objects-in-receptacle",
        296,
        3,
        {
            "first_object": "alphabet-soup",
            "second_object": "cream-cheese",
            "target": "wooden-tray",
        },
    ),
    "rearrange-objects": _tabletop_case(
        "rearrange-objects/soup-in-tray/sauce-on-plate/scene-296/variation-3",
        "rearrange-objects",
        296,
        3,
        {
            "first_object": "alphabet-soup",
            "second_object": "tomato-sauce",
            "containment_target": "wooden-tray",
            "support_target": "plate",
        },
    ),
}

ROBOCASA_OPEN_DRAWER_CASE = EvaluationCase(
    episode_request=EpisodeRequest(
        case_id="layout-1-style-1-open-drawer-seed-3",
        robot="xarm7",
        world=WorldRequest(
            family_id="robocasa-layout-1-style-1-open-drawer-seed-3",
        ),
        task=TaskRequest(
            family_id="robocasa-open-drawer-canary",
            seed=3,
            roles={"drawer": "drawer", "drawer_object": "pizza_cutter"},
        ),
    ),
    blueprint_name="xarm-perception-sim",
    required_modules=("PickAndPlaceModule",),
    required_roles=("drawer", "drawer_object"),
)

PLACE_CASES = (
    pytest.param(TABLETOP_CASES["object-in-receptacle"], 0.10, id="object-in-receptacle"),
    pytest.param(TABLETOP_CASES["object-on-support"], 0.08, id="object-on-support"),
)


def test_supported_task_families_have_maintained_e2e_cases() -> None:
    provider = load_episode_provider("pimsim")
    maintained_family_ids = {
        cast("EpisodeRequest", case.episode_request).task.family_id
        for case in TABLETOP_CASES.values()
    }

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


@pytest.mark.parametrize(
    "evaluation_episode",
    [pytest.param(ROBOCASA_OPEN_DRAWER_CASE, id="open-drawer")],
    indirect=True,
)
def test_native_robocasa_drawer_reports_partial_public_execution(
    evaluation_episode: EpisodeRun,
) -> None:
    actions = _actions(evaluation_episode)
    joints = actions.get_current_joints()
    assert joints is not None
    assert actions.is_collision_free(joints), "robot starts in collision"

    open_result = actions.open_gripper()
    assert open_result.success, open_result
    for x, y, z in (
        (0.50, -0.85, 0.950),
        (0.50, -0.75, 0.785),
        (0.50, -0.607, 0.785),
    ):
        result = actions.move_to_pose(x, y, z, -math.pi / 2.0, 0.0, 0.0)
        assert result.success, result

    open_position = actions.get_gripper()
    assert open_position is not None
    close_result = actions.close_gripper()
    assert close_result.success, close_result
    _wait_for_gripper_motion_to_settle(actions, initial_position=open_position)

    for y in (-0.66, -0.70, -0.74, -0.78, -0.82, -0.86, -0.90, -0.94):
        result = actions.move_to_pose(0.50, y, 0.785, -math.pi / 2.0, 0.0, 0.0)
        assert result.success, result

    evaluation = evaluation_episode.evaluate_goal()
    goal = json.loads(evaluation.summary)

    assert evaluation.passed is False
    assert goal["kind"] == "open"
    assert goal["passed"] is False
    assert 0.40 < goal["measurements"]["open_progress"] < 0.95


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


def _wait_for_gripper_motion_to_settle(
    actions: _PickAndPlaceActions,
    *,
    initial_position: float,
    timeout: float = 3.0,
) -> float:
    deadline = time.monotonic() + timeout
    previous = initial_position
    moved = False
    stable_samples = 0
    while time.monotonic() < deadline:
        current = actions.get_gripper()
        if current is None:
            pytest.fail("gripper state became unavailable")
        moved = moved or abs(current - initial_position) > 0.01
        stable_samples = stable_samples + 1 if abs(current - previous) < 1e-3 else 0
        if moved and stable_samples >= 3:
            return current
        previous = current
        time.sleep(0.05)
    pytest.fail("gripper did not move and settle before the timeout")
