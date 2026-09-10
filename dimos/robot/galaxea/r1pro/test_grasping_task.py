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

"""Physical contact and reset checks using the pinned R1Pro model."""

import mujoco
import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.grasping_sim import MANIPULATION_JOINTS, prepare_grasping_scene
from dimos.robot.galaxea.r1pro.grasping_task import GraspingTask
from dimos.robot.galaxea.r1pro.grasping_transport import PlanarTransport
from dimos.simulation.engines.robot_sim_binding import RobotSimSpec, resolve_robot_sim_binding
from dimos.simulation.utils.xml_parser import build_joint_mappings

pytestmark = [pytest.mark.mujoco, pytest.mark.self_hosted]


@pytest.fixture(scope="module")
def task_scene(tmp_path_factory):
    return prepare_grasping_scene(tmp_path_factory.mktemp("r1pro-grasp") / "scene.xml")


@pytest.fixture
def task(task_scene):
    with GraspingTask(task_scene, images=False) as environment:
        yield environment


def test_action_mapping_excludes_free_object_and_passive_finger_followers(task_scene):
    model = mujoco.MjModel.from_xml_path(str(task_scene))
    binding = resolve_robot_sim_binding(
        model,
        RobotSimSpec(
            robot_id="r1pro",
            hardware_joints=MANIPULATION_JOINTS,
            model_joint_names=MANIPULATION_JOINTS,
            model_actuator_names=MANIPULATION_JOINTS,
        ),
        build_joint_mappings(task_scene, model),
    )
    assert len(binding.joint_qpos_adrs) == 20
    assert model.joint("task_bottle_free").qposadr[0] not in binding.joint_qpos_adrs
    assert model.joint("right_gripper_follower").qposadr[0] not in binding.joint_qpos_adrs
    assert all(kind == mujoco.mjtEq.mjEQ_JOINT for kind in model.eq_type)


@pytest.mark.parametrize("seed", [61, 67, 83])
def test_teacher_lifts_with_both_fingers_and_leaves_bottle_released_in_bin(task, seed):
    task.reset(seed)
    for _, action in task.teacher_actions():
        task.step(action)
    result = task.result()
    assert result.success
    assert result.bilateral_grasp and result.peak_lift_m > 0.08
    assert result.inside_bin and result.released and result.settled


def test_reset_restores_robot_and_object_without_retaining_success(task):
    task.reset(17)
    initial = task.data.qpos.copy()
    for _, action in task.teacher_actions():
        task.step(action)
    task.reset(17)
    np.testing.assert_allclose(task.data.qpos, initial, atol=1e-9)
    assert not task.result().success
    assert not task.result().bilateral_grasp
    assert task.result().peak_lift_m == 0


def test_an_untouched_bottle_cannot_pass_task_success(task):
    for _ in range(40):
        task.step(task.home)
    result = task.result()
    assert not result.success
    assert not result.inside_bin
    assert not result.bilateral_grasp


def test_mobile_tray_carries_a_contact_grasped_bottle_and_planning_preserves_state(tmp_path):
    scene = prepare_grasping_scene(tmp_path / "mobile.xml", mobile=True)
    with GraspingTask(scene, images=False) as task:
        for _, action in task.teacher_actions():
            task.step(action)
        assert task.result().success
        initial = task.data.qpos.copy()
        initial_bottle = task.data.body("task_bottle").xpos.copy()
        planner = PlanarTransport(task.model, task.data)
        path = planner.plan((-0.8, 0.0))
        np.testing.assert_array_equal(task.data.qpos, initial)
        with pytest.raises(RuntimeError, match="obstructed"):
            planner.plan((0.75, 0.17))
        hold = task.data.qpos[task.qids].copy()
        for target in planner.targets(path, 20):
            task.step(hold, base_target=target)
            assert task.result().inside_bin
            assert planner.collisions(task.data) == []
        for _ in range(20):
            task.step(hold)
        assert task.result().success
        np.testing.assert_allclose(task.data.qpos[planner.qids[:2]], [-0.8, 0], atol=0.005)
        np.testing.assert_allclose(
            task.data.body("task_bottle").xpos - initial_bottle,
            [-0.8, 0, 0],
            atol=0.01,
        )
