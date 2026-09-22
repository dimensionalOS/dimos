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

"""Fixed startup posture is separate from cargo-adapted carry preparation."""

from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.classical_planning import ClassicalGraspPlanner
from dimos.robot.galaxea.r1pro.grasping_sim import VIRTUAL_BASE_JOINTS
from dimos.robot.galaxea.r1pro.home_kinematics import HomeKinematics
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.object_primitive_state import PrimitiveSceneState
from dimos.robot.galaxea.r1pro.posture_ik import NOMINAL_POSTURE


@pytest.fixture
def init_scene(mocker):
    bodies = ['<body name="base_link"/>']
    names = (*R1PRO_PICK_PLACE_JOINTS, *VIRTUAL_BASE_JOINTS)
    for name in (*names, "left_gripper_follower", "right_gripper_follower"):
        axis = "0 1 0" if name == "r1pro/left_arm_joint1" else "0 0 1"
        site = ""
        for side in ("left", "right"):
            if name == f"r1pro/{side}_arm_joint1":
                site = f'<site name="{side}_tcp"/>'
        bodies.append(
            f'<body><joint name="{name}" axis="{axis}"/>'
            '<inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>'
            f"{site}</body>"
        )
    for side in ("left", "right"):
        bodies.append(
            f'<body name="cargo_{side}"><freejoint name="cargo_{side}_joint"/>'
            '<inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>'
        )
    actuators = "".join(f'<position name="{name}" joint="{name}"/>' for name in names)
    model = mujoco.MjModel.from_xml_string(
        f"<mujoco><worldbody>{''.join(bodies)}</worldbody><actuator>{actuators}</actuator></mujoco>"
    )
    data = mujoco.MjData(model)
    home = np.r_[NOMINAL_POSTURE, 0.05, 0.05]
    for name, value in zip(R1PRO_PICK_PLACE_JOINTS, home, strict=True):
        data.joint(name).qpos[0] = value
        data.actuator(name).ctrl[0] = value
    for side in ("left", "right"):
        data.joint(f"{side}_gripper_follower").qpos[0] = 0.05
    mujoco.mj_forward(model, data)
    scene = mocker.Mock(spec=PrimitiveSceneState, model=model, data=data)
    scene.arms = {side: SimpleNamespace(home=home) for side in ("left", "right")}
    scene.layout = SimpleNamespace(
        objects=[
            SimpleNamespace(name=f"cargo_{side}", joint=f"cargo_{side}_joint")
            for side in ("left", "right")
        ]
    )
    scene.inventory.return_value = [
        dict(object=f"cargo_{side}", held_by=None, grasped=False, contacting_arms=[])
        for side in ("left", "right")
    ]
    scene.transport_planner.return_value = mocker.Mock(
        robot_bodies=set(), cargo_ids=set(), tray_id=-1
    )
    kin = mocker.Mock(
        spec=HomeKinematics,
        world=mocker.Mock(),
        natural_posture=True,
        reference_position=np.zeros(3),
        reference_rotation=np.eye(3),
    )
    kin.world.check_config_collision_free.return_value = True
    return scene, kin, home


def test_init_already_home_returns_hold_without_opening_loaded_hands(init_scene, mocker):
    scene, kin, _ = init_scene
    for row, side in zip(scene.inventory.return_value, ("left", "right"), strict=True):
        row.update(held_by=side, grasped=True, contacting_arms=[side])
        scene.data.actuator(f"r1pro/{side}_gripper").ctrl[0] = 0
    planner = ClassicalGraspPlanner(scene, kinematics=kin)
    search = mocker.patch.object(planner, "posture_path")
    expected = [float(scene.data.actuator(n).ctrl[0]) for n in R1PRO_PICK_PLACE_JOINTS]
    before = scene.data.qpos.copy()

    assert planner.init_posture() == [expected]
    np.testing.assert_array_equal(scene.data.qpos, before)
    search.assert_not_called()


def test_init_targets_recorded_joints_with_load_bias_and_preserves_scene(init_scene, mocker):
    scene, kin, home = init_scene
    scene.data.joint("r1pro/right_arm_joint1").qpos[0] = 0.3
    scene.data.actuator("r1pro/right_arm_joint1").ctrl[0] = 0.302
    scene.data.actuator("r1pro/left_gripper").ctrl[0] = 0
    scene.inventory.return_value[0].update(held_by="left", grasped=True, contacting_arms=["left"])
    mujoco.mj_forward(scene.model, scene.data)
    planner = ClassicalGraspPlanner(scene, kinematics=kin)
    measured = planner.initial.qpos[planner.qids].copy()
    middle = (measured + home) / 2
    search = mocker.patch.object(
        planner, "posture_path", return_value=[measured.tolist(), middle.tolist(), home.tolist()]
    )
    qpos_before, ctrl_before = scene.data.qpos.copy(), scene.data.ctrl.copy()

    points = np.asarray(planner.init_posture())

    search.assert_called_once_with(0, "left", home.tolist(), allow_target_contact=False)
    np.testing.assert_allclose(points[-1, :18], home[:18] + np.eye(18)[11] * 0.002)
    np.testing.assert_array_equal(points[:, 18:], [[0, 0.05]] * 3)
    np.testing.assert_array_equal(scene.data.qpos, qpos_before)
    np.testing.assert_array_equal(scene.data.ctrl, ctrl_before)


def test_init_rejects_fixed_endpoint_that_would_tip_cup_before_search(init_scene, mocker):
    scene, kin, _ = init_scene
    scene.data.joint("r1pro/left_arm_joint1").qpos[0] = 0.5
    scene.data.actuator("r1pro/left_arm_joint1").ctrl[0] = 0.5
    scene.data.actuator("r1pro/left_gripper").ctrl[0] = 0
    scene.inventory.return_value[0].update(held_by="left", grasped=True, contacting_arms=["left"])
    mujoco.mj_forward(scene.model, scene.data)
    planner = ClassicalGraspPlanner(scene, kinematics=kin)
    search = mocker.patch.object(planner, "posture_path")
    before = scene.data.qpos.copy()

    with pytest.raises(RuntimeError, match="Exact init position would tip held cargo_left to 28.6"):
        planner.init_posture()

    search.assert_not_called()
    np.testing.assert_array_equal(scene.data.qpos, before)


@pytest.mark.parametrize("grasped", [False, True])
def test_init_rejects_supported_or_ambiguous_gripper_contact(init_scene, grasped, mocker):
    scene, kin, _ = init_scene
    scene.inventory.return_value[0].update(grasped=grasped, contacting_arms=["left"])
    planner = ClassicalGraspPlanner(scene, kinematics=kin)
    search = mocker.patch.object(planner, "posture_path")

    with pytest.raises(RuntimeError, match="unverified or supported grasp"):
        planner.init_posture()

    search.assert_not_called()


def test_init_waits_for_robot_to_settle_before_planning(init_scene, mocker):
    scene, kin, _ = init_scene
    scene.data.actuator("r1pro/right_arm_joint1").ctrl[0] = 0.04
    planner = ClassicalGraspPlanner(scene, kinematics=kin)
    search = mocker.patch.object(planner, "posture_path")

    with pytest.raises(RuntimeError, match="must settle"):
        planner.init_posture()

    search.assert_not_called()


def test_init_rejects_obstructed_fixed_endpoint_without_changing_goal(init_scene, mocker):
    scene, kin, _ = init_scene
    kin.world.check_config_collision_free.side_effect = [True, False]
    planner = ClassicalGraspPlanner(scene, kinematics=kin)
    search = mocker.patch.object(planner, "posture_path")

    with pytest.raises(RuntimeError, match="fixed init position is obstructed"):
        planner.init_posture()

    search.assert_not_called()
