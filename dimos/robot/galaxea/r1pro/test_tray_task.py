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

"""Physical support, contact-only carrying, and release of a loaded tray."""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.grasping_task import GraspingTask
from dimos.robot.galaxea.r1pro.grasping_transport import PlanarTransport
from dimos.robot.galaxea.r1pro.tray_motion import TrayMotion
from dimos.robot.galaxea.r1pro.tray_sim import configure_tray_holding, prepare_tray_delivery_scene
from dimos.robot.galaxea.r1pro.tray_task import laptop_destination, tray_state

pytestmark = [pytest.mark.mujoco, pytest.mark.self_hosted]


def test_both_hands_physically_carry_tray_and_opening_them_lets_it_fall(tmp_path):
    scene = prepare_tray_delivery_scene(tmp_path / "tray.xml")
    with GraspingTask(scene, images=False) as task:
        initial = tray_state(task.model, task.data)
        assert initial["released"] and initial["support_geoms"]
        for _, action in task.teacher_actions():
            task.step(action)
        assert task.result().success
        configure_tray_holding(task.model)
        motion = TrayMotion(task.model, task.data)
        for _, action in motion.actions(motion.pickup(task.data)):
            task.step(action)
        lifted = tray_state(task.model, task.data)
        assert lifted["bimanual_grasp"] and not lifted["support_geoms"]
        assert lifted["position"][2] > initial["position"][2] + 0.08
        original_qpos = task.data.qpos.copy()
        planner = PlanarTransport(task.model, task.data)
        path = planner.plan((-0.6, 0.0))
        np.testing.assert_array_equal(task.data.qpos, original_qpos)
        hold = task.data.ctrl[task.aids].copy()
        for target in planner.targets(path, 20):
            task.step(hold, base_target=target)
            assert tray_state(task.model, task.data)["bimanual_grasp"]
            assert task.result().inside_bin
            assert planner.collisions(task.data) == []
        carried = tray_state(task.model, task.data)
        np.testing.assert_allclose(
            np.array(carried["position"])[:2] - lifted["position"][:2], [-0.6, 0.0], atol=0.015
        )
        hold[-2:] = 0.05
        for _ in range(40):
            task.step(hold)
        released = tray_state(task.model, task.data)
        assert released["released"]
        assert released["position"][2] < carried["position"][2] - 0.3


def test_delivery_target_tracks_actual_tabletop_beside_laptop():
    model = mujoco.MjModel.from_xml_string("""
    <mujoco><worldbody>
      <body name="desk"><geom name="top" type="box" pos="-1.5 -3.1 .759" size=".7 .325 .009"/></body>
      <body name="entity:laptop" pos="-1.5 -3.07 .79"><geom type="box" size=".15 .17 .01"/></body>
    </worldbody></mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    goal = laptop_destination(model, data)
    assert goal.support_geom == "top"
    np.testing.assert_allclose(goal.tray_position, [-2.0, -2.955, 0.768])
    np.testing.assert_allclose(goal.base_position, [-2.0, -2.475, -np.pi / 2])


def test_arm_plan_rejects_a_blocked_approach_before_execution(tmp_path):
    scene = prepare_tray_delivery_scene(tmp_path / "blocked.xml")
    tree = ET.parse(scene)
    world = tree.getroot().find("worldbody")
    ET.SubElement(
        world, "geom", name="blocking_box", type="box", pos="0.36 0.17 0.93", size="0.08 0.08 0.08"
    )
    tree.write(scene, encoding="unicode")
    with GraspingTask(scene, images=False) as task:
        initial = task.data.qpos.copy()
        with pytest.raises(RuntimeError, match="arm path is obstructed"):
            TrayMotion(task.model, task.data).pickup(task.data)
        np.testing.assert_array_equal(task.data.qpos, initial)
