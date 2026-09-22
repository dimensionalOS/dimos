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

"""Carry preparation never enables a stale path or discards recovery state on failure."""

import threading

import mujoco
import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.classical_sim import R1ProClassicalSim
from dimos.robot.galaxea.r1pro.grasping_sim import VIRTUAL_BASE_JOINTS
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.object_primitive_state import PrimitiveSceneState


@pytest.fixture
def carry_sim(mocker, tmp_path):
    names = (*R1PRO_PICK_PLACE_JOINTS, *VIRTUAL_BASE_JOINTS)
    bodies = "".join(
        f'<body><joint name="{name}"/><geom type="sphere" size="0.01"/></body>' for name in names
    )
    model = mujoco.MjModel.from_xml_string(f"<mujoco><worldbody>{bodies}</worldbody></mujoco>")
    sim = R1ProClassicalSim(output=tmp_path, scene_package=None)
    sim._engine = mocker.Mock(model=model, data=mujoco.MjData(model), _lock=threading.RLock())
    snapshot = mocker.Mock(spec=PrimitiveSceneState, model=model, data=mujoco.MjData(model))
    snapshot.inventory.return_value = [dict(object="object_1", held_by="left")]
    live = mocker.Mock(spec=PrimitiveSceneState)
    mocker.patch.object(sim, "_snapshot", return_value=snapshot)
    mocker.patch.object(sim, "_state", return_value=live)
    planner = mocker.patch(
        "dimos.robot.galaxea.r1pro.classical_sim.ClassicalGraspPlanner"
    ).return_value
    planner.carry_posture.return_value = [[0.0] * 20, [0.1] * 18 + [0.0, 0.0]]
    planner.init_posture.return_value = [[0.0] * 20, [0.02] * 18 + [0.0, 0.0]]
    snapshot.arms = {"right": mocker.Mock(home=np.array([0.01] * 18 + [0.05, 0.05]))}
    mocker.patch.object(sim, "tray_state", return_value=dict(held=False, finger_contacts=[]))
    sim._active = ("pick", "left")
    sim._initial = [dict(object="object_1", held_by=None)]
    yield sim, snapshot, live, planner
    sim._engine = None
    sim.stop()


def test_checked_carry_transitions_from_pick_to_all_object_guard(carry_sim):
    sim, snapshot, live, planner = carry_sim

    result = sim.classical_carry_posture()

    assert result == [[0.0] * 20, [0.1] * 18 + [0.0, 0.0]]
    live.validate.assert_called_once_with(snapshot.inventory.return_value, arm="right", selected=-1)
    assert sim._transport_initial == [dict(object="object_1", held_by="left")]
    assert sim._active is None
    assert sim._initial is None


@pytest.mark.parametrize("joint_index", [0, 4, 11, 18, 20, 21, 22])
def test_moved_robot_rejects_carry_without_discarding_pick_recovery(carry_sim, joint_index):
    sim, snapshot, live, planner = carry_sim
    initial = sim._initial
    sim._engine.data.qpos[joint_index] = 0.01

    with pytest.raises(RuntimeError, match="Robot moved during carry preparation"):
        sim.classical_carry_posture()

    assert sim._transport_initial is None
    assert sim._active == ("pick", "left")
    assert sim._initial is initial


def test_small_settling_motion_does_not_invalidate_carry(carry_sim):
    sim, snapshot, live, planner = carry_sim
    sim._engine.data.qpos[0] = 0.002
    sim._engine.data.qpos[20:] = [0.002, 0.002, 0.002]

    assert sim.classical_carry_posture() == [[0.0] * 20, [0.1] * 18 + [0.0, 0.0]]
    assert sim._transport_initial == snapshot.inventory.return_value


def test_cargo_slip_during_planning_preserves_pick_recovery(carry_sim):
    sim, snapshot, live, planner = carry_sim
    initial = sim._initial
    live.validate.side_effect = RuntimeError("Lost or disturbed the other hand's object_1")

    with pytest.raises(RuntimeError, match="Lost or disturbed"):
        sim.classical_carry_posture()

    assert sim._transport_initial is None
    assert sim._active == ("pick", "left")
    assert sim._initial is initial


def test_planning_failure_preserves_pick_recovery(carry_sim):
    sim, snapshot, live, planner = carry_sim
    initial = sim._initial
    planner.carry_posture.side_effect = RuntimeError("No clear compact carrying posture")

    with pytest.raises(RuntimeError, match="No clear compact"):
        sim.classical_carry_posture()

    assert sim._transport_initial is None
    assert sim._active == ("pick", "left")
    assert sim._initial is initial
    live.validate.assert_not_called()


def test_scene_fault_during_planning_rejects_carry(carry_sim):
    sim, snapshot, live, planner = carry_sim

    def fault_during_plan():
        sim._error = "Robot collided with environment"
        return [[0.0] * 20]

    planner.carry_posture.side_effect = fault_during_plan

    with pytest.raises(RuntimeError, match="Robot collided"):
        sim.classical_carry_posture()

    assert sim._transport_initial is None
    assert sim._active == ("pick", "left")
    live.validate.assert_not_called()


def test_init_returns_recorded_measured_target_not_actuator_or_gripper_home(carry_sim):
    sim, snapshot, live, planner = carry_sim

    result = sim.classical_init_posture()

    assert result == dict(
        waypoints=[[0.0] * 20, [0.02] * 18 + [0.0, 0.0]], target_joints=[0.01] * 18
    )
    planner.init_posture.assert_called_once_with()
    planner.carry_posture.assert_not_called()
    live.validate.assert_called_once_with(snapshot.inventory.return_value, arm="right", selected=-1)
    assert sim._transport_initial == snapshot.inventory.return_value
    assert sim._active is None
    assert sim._initial is None


@pytest.mark.parametrize("joint_index", [0, 4, 11, 18, 20, 21, 22])
def test_moved_robot_rejects_init_without_discarding_recovery(carry_sim, joint_index):
    sim, snapshot, live, planner = carry_sim
    initial = sim._initial
    sim._engine.data.qpos[joint_index] = 0.01

    with pytest.raises(RuntimeError, match="Robot moved during init preparation"):
        sim.classical_init_posture()

    assert sim._transport_initial is None
    assert sim._active == ("pick", "left")
    assert sim._initial is initial


def test_unsafe_init_is_not_replaced_with_carry_or_reset(carry_sim, mocker):
    sim, snapshot, live, planner = carry_sim
    initial = sim._initial
    reset = mocker.patch.object(sim, "reset")
    planner.init_posture.side_effect = RuntimeError("Exact init position would tip held cup")

    with pytest.raises(RuntimeError, match="Exact init position would tip"):
        sim.classical_init_posture()

    planner.carry_posture.assert_not_called()
    reset.assert_not_called()
    live.validate.assert_not_called()
    assert sim._transport_initial is None
    assert sim._active == ("pick", "left")
    assert sim._initial is initial


@pytest.mark.parametrize(
    "tray", [dict(held=True, finger_contacts=[]), dict(held=False, finger_contacts=["left"])]
)
def test_init_rejects_held_or_contacting_tray_before_object_only_planning(carry_sim, tray):
    sim, snapshot, live, planner = carry_sim
    sim.tray_state.return_value = tray

    with pytest.raises(RuntimeError, match="Put down the tray"):
        sim.classical_init_posture()

    planner.init_posture.assert_not_called()
    assert sim._transport_initial is None
