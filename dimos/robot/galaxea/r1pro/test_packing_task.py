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

"""Physical five-object regressions; no synthetic object attachments."""

import mujoco
import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.grasping_transport import PlanarTransport
from dimos.robot.galaxea.r1pro.packing_sim import PACKING_BODIES, prepare_packing_scene
from dimos.robot.galaxea.r1pro.packing_state import (
    PackingMonitor,
    open_gripper_at_home,
    plan_bottle_goal,
    score_packing,
)
from dimos.robot.galaxea.r1pro.packing_task import PackingTask
from dimos.robot.galaxea.r1pro.tray_motion import TrayMotion
from dimos.robot.galaxea.r1pro.tray_sim import configure_tray_holding
from dimos.robot.galaxea.r1pro.tray_task import tray_state

pytestmark = [pytest.mark.mujoco, pytest.mark.self_hosted]


@pytest.fixture(scope="module")
def scene(tmp_path_factory):
    return prepare_packing_scene(tmp_path_factory.mktemp("packing") / "scene.xml")


@pytest.fixture
def task(scene):
    with PackingTask(scene, images=False) as environment:
        yield environment


def test_goal_selection_is_read_only_and_reset_clears_episode_evidence(task):
    task.reset_packing(8200, 0.003)
    before = task.data.qpos.copy(), task.data.qvel.copy(), task.data.ctrl.copy()
    assert plan_bottle_goal(task.data, 0) is not None
    for actual, expected in zip(
        (task.data.qpos, task.data.qvel, task.data.ctrl), before, strict=True
    ):
        np.testing.assert_array_equal(actual, expected)
    assert task.report()["packed"] == 0


@pytest.mark.parametrize("seed", [8200, 8217])
def test_five_bottles_are_lifted_released_and_remain_in_the_tray(task, seed):
    task.reset_packing(seed, 0.003)
    monitor = PackingMonitor(task.model, task.data)
    for index in task.pick_order(seed):
        assert task.select_bottle(index)
        for _, action in task.teacher_actions():
            task.step(action)
            monitor.observe()
        task.remember_result()
        assert task.pick_complete()
    assert task.report()["success"]
    assert monitor.report()["packed"] == 5
    assert all(row["released"] and row["settled"] for row in monitor.report()["bottles"])
    task.reset_packing(seed, 0.003)
    assert not task.report()["success"]
    assert task.report()["packed"] == 0


def test_contained_bottle_lying_down_is_not_a_neat_packing_success(task):
    task.reset_packing(8200, 0.003)
    tray = task.data.body("task_bin").xpos.copy()
    task.data.joint("task_bottle_free").qpos[:] = (
        *tuple(tray + np.array([0, 0, 0.040])),
        np.sqrt(0.5),
        0,
        np.sqrt(0.5),
        0,
    )
    task.data.joint("task_bottle_free").qvel[:] = 0
    mujoco.mj_forward(task.model, task.data)
    result = score_packing(task.data, 0, peak_lift=0.12, bilateral_grasp=True, touching_pads=set())
    assert result.inside_bin and result.released and result.settled
    assert not result.upright
    assert not result.success


def test_a_fallen_bottle_blocks_slots_that_an_upright_bottle_would_clear(task):
    task.reset_packing(8200, 0.003)
    tray = task.data.body("task_bin").xpos.copy()
    joint = task.data.joint("task_bottle_free_2")
    joint.qpos[:] = (*tuple(tray + np.array([0, 0, 0.085])), 1, 0, 0, 0)
    mujoco.mj_forward(task.model, task.data)
    assert plan_bottle_goal(task.data, 0) is not None
    joint.qpos[:] = (*tuple(tray + np.array([0, 0, 0.040])), np.sqrt(0.5), 0, np.sqrt(0.5), 0)
    mujoco.mj_forward(task.model, task.data)
    before = task.data.qpos.copy(), task.data.ctrl.copy()
    assert not task.select_bottle(0)
    np.testing.assert_array_equal(task.data.qpos, before[0])
    np.testing.assert_array_equal(task.data.ctrl, before[1])


def test_placed_bottle_stays_released_when_gripper_closes_elsewhere(task):
    task.reset_packing(8200, 0.003)
    tray = task.data.body("task_bin").xpos.copy()
    task.data.joint("task_bottle_free").qpos[:] = (
        *tuple(tray + np.array([0, 0, 0.085])),
        1,
        0,
        0,
        0,
    )
    task.data.joint("task_bottle_free").qvel[:] = 0
    task.data.joint("r1pro/right_gripper").qpos[:] = 0
    mujoco.mj_forward(task.model, task.data)
    result = score_packing(task.data, 0, peak_lift=0.12, bilateral_grasp=True, touching_pads=set())
    assert result.success and result.released
    # Per-object release does not permit another pick with a closed gripper.
    assert not open_gripper_at_home(task.data)


def test_five_bottle_tray_is_supported_then_physically_carried_with_all_cargo(task):
    task.reset_packing(8200, 0.003)
    for index in task.pick_order():
        assert task.select_bottle(index)
        for _, action in task.teacher_actions():
            task.step(action)
        task.remember_result()
    assert task.report()["success"]
    initial = tray_state(task.model, task.data, cargo_bodies=PACKING_BODIES)
    assert initial["support_geoms"]
    assert all("bottle" not in name for name in initial["support_geoms"])

    configure_tray_holding(task.model)
    motion = TrayMotion(task.model, task.data, cargo_bodies=PACKING_BODIES)
    for _, action in motion.actions(motion.pickup(task.data)):
        task.step(action)
    lifted = tray_state(task.model, task.data, cargo_bodies=PACKING_BODIES)
    assert lifted["bimanual_grasp"]
    assert not lifted["support_geoms"]
    assert lifted["position"][2] > initial["position"][2] + 0.08

    before = task.data.qpos.copy()
    planner = PlanarTransport(task.model, task.data, cargo_bodies=PACKING_BODIES)
    path = planner.plan((-0.4, 0.0))
    np.testing.assert_array_equal(task.data.qpos, before)
    for name in ("task_bin", *PACKING_BODIES):
        np.testing.assert_allclose(
            planner.probe.body(name).xpos[:2] - task.data.body(name).xpos[:2],
            [-0.4, 0.0],
            atol=1e-6,
        )
    hold = task.data.ctrl[task.aids].copy()
    for target in planner.targets(path, 20):
        task.step(hold, base_target=target)
        assert tray_state(task.model, task.data, cargo_bodies=PACKING_BODIES)["bimanual_grasp"]
        assert all(row["inside_bin"] and row["upright"] for row in task.report()["bottles"])
        assert not planner.collisions(task.data)
