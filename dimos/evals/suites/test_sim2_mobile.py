# Copyright 2026 Dimensional Inc.
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

from functools import partial

import mujoco
import pytest

from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.agents.mobile_scripted import MobileScripted
from dimos.evals.environments.sim2 import Intervention
from dimos.evals.runner import EvalRunner
from dimos.evals.suites.sim2_mobile import SCENE, SUITE, grade_mobile, mobile_case
from dimos.evals.types import Outcome
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.sim2.world.scene import describe_scene
from dimos.sim2.world.scene_types import EntityState, RegionState, SceneState, SceneUpdate


def state(t, *, x=1.7, y=0, height=0.8, generation=0, contacts=()):
    return SceneState(
        world_id="world",
        scene_id="room",
        episode=0,
        generation=generation,
        tick=round(t * 200),
        sim_time=t,
        ts=1000 + t,
        entities={
            "target": EntityState(
                pose=Pose(1.7, 0, 0.35),
                velocity=(0, 0, 0),
                angular_velocity=(0, 0, 0),
                bounds_min=(1.5, -0.2, 0),
                bounds_max=(1.9, 0.2, 0.7),
            )
        },
        robots={"g1connection": Pose(x, y, height)},
        joints={},
        contacts=contacts,
        regions={"goal": RegionState(pose=Pose(1.7, 0, 0.8), size=(0.7, 0.7, 1.6))},
    )


def outcome(tmp_path, states, edits=()):
    path = tmp_path / "truth.db"
    with SqliteStore(path=str(path)) as store:
        for sample in states:
            store.stream("scene_state", SceneState).append(sample)
        for edit in edits:
            store.stream("interventions", Intervention).append(edit)
        store.stream("completed", SceneState).append(states[-1])
    return Outcome(
        trajectory=TrajectoryBuilder("", name="fixture").build("answer"), artifacts={"truth": path}
    )


def test_six_cases_share_existing_runner_contract_and_robot_free_scene():
    assert [case.id for case in SUITE] == [
        f"sim2_{robot}_{task}" for robot in ("g1", "go2") for task in ("reach", "replan", "search")
    ]
    assert all(
        isinstance(case.grade, partial) and case.grade.func is grade_mobile for case in SUITE
    )
    assert all(case.environment.config.attach is False for case in SUITE)
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    scene = describe_scene(SCENE)
    assert model.nq == 14  # two free props; robots are composed by sim2
    assert scene.entities["target"].movable and scene.entities["obstacle"].movable
    assert scene.spawns["default"].position.to_tuple() == (-2, 0, 0)


@pytest.mark.parametrize(
    "samples,score",
    [
        ([state(i / 10) for i in range(15)], 1),
        ([state(i / 10) for i in range(5)], 0),
        ([state(i / 10, x=1.0) for i in range(15)], 0),
        ([state(i / 10, height=0.2) for i in range(15)], 0),
        ([state(i / 10, contacts=(("g1connection/pelvis", "obstacle"),)) for i in range(15)], 0),
    ],
)
def test_reach_requires_dwell_upright_and_no_sampled_collision(tmp_path, samples, score):
    assert grade_mobile(outcome(tmp_path, samples), robot_id="g1connection", task="reach") == score


@pytest.mark.parametrize("task,reason", [("replan", "block route"), ("search", "relocate target")])
def test_intervention_cases_score_only_after_the_recorded_edit(tmp_path, task, reason):
    before, after = state(0), state(0, generation=1)
    edit = Intervention(
        reason=reason, update=SceneUpdate(), before=before, after=after, ts=after.ts
    )
    states = [before, after, *[state(i / 10, generation=1) for i in range(1, 15)]]
    assert grade_mobile(outcome(tmp_path, states, [edit]), robot_id="g1connection", task=task) == 1


def test_replan_cannot_pass_without_an_obstacle_intervention(tmp_path):
    assert (
        grade_mobile(
            outcome(tmp_path, [state(i / 10) for i in range(15)]),
            robot_id="g1connection",
            task="replan",
        )
        == 0
    )


def test_intervention_does_not_erase_a_previous_collision(tmp_path):
    before = state(0, contacts=(("g1connection/pelvis", "obstacle"),))
    after = state(0, generation=1)
    edit = Intervention(
        reason="block route", update=SceneUpdate(), before=before, after=after, ts=after.ts
    )
    states = [before, after, *[state(i / 10, generation=1) for i in range(1, 15)]]
    assert (
        grade_mobile(outcome(tmp_path, states, [edit]), robot_id="g1connection", task="replan") == 0
    )


def test_route_intervention_fires_once_after_real_robot_progress(mocker):
    env = mobile_case("g1", "replan").environment
    edit = mocker.patch.object(env, "intervene")
    env.on_state(state(0, x=-2))
    edit.assert_not_called()
    env.on_state(state(0.1, x=-1.5))
    env.on_state(state(0.2, x=-1.3))
    edit.assert_called_once_with(
        SceneUpdate(poses={"obstacle": Pose(0, 0, 0.65)}), reason="block route"
    )


def test_relocation_waits_for_the_first_approach(mocker):
    env = mobile_case("g1", "search").environment
    edit = mocker.patch.object(env, "intervene")
    env.on_state(state(0, x=-2))
    env.on_state(state(0.1, x=0.8))
    env.on_state(state(0.5, x=0.8))
    edit.assert_not_called()
    env.on_state(state(0.7, x=0.8))
    edit.assert_called_once()


@pytest.mark.self_hosted
@pytest.mark.native_e2e
@pytest.mark.timeout(240)
@pytest.mark.parametrize("robot", ["g1", "go2"])
@pytest.mark.parametrize("task", ["reach", "replan", "search"])
def test_mobile_case_through_real_runner(tmp_path, robot, task):
    runner = EvalRunner(out_dir=tmp_path, strict=True)
    result = runner.run([mobile_case(robot, task)], MobileScripted())[0]
    assert result.error == "", result.error
    assert result.ended_by == "answer"
    assert result.score == 1.0
