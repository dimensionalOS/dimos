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

"""Asset-backed MuJoCo integration tests; excluded from default fast tests."""

import json

import mujoco
import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.learning import R1PRO_SIM_ACT_JOINTS
from dimos.robot.galaxea.r1pro.sim_act import prepare_r1pro_act_scene
from dimos.simulation.engines.robot_sim_binding import RobotSimSpec, resolve_robot_sim_binding
from dimos.simulation.utils.xml_parser import build_joint_mappings

pytestmark = [pytest.mark.mujoco, pytest.mark.self_hosted]


@pytest.fixture(scope="module")
def robot_model(tmp_path_factory):
    path = prepare_r1pro_act_scene(tmp_path_factory.mktemp("r1pro") / "scene.xml")
    return mujoco.MjModel.from_xml_path(str(path))


def test_act_contract_drives_upper_body_and_keeps_grippers_fixed(robot_model):
    assert (
        tuple(robot_model.actuator(i).name for i in range(robot_model.nu)) == R1PRO_SIM_ACT_JOINTS
    )
    assert robot_model.nq == len(R1PRO_SIM_ACT_JOINTS)
    for side in ("left", "right"):
        for finger in (1, 2):
            assert robot_model.body(f"{side}_gripper_finger_link{finger}").jntnum[0] == 0
    assert robot_model.camera("overview").id >= 0
    assert np.count_nonzero(robot_model.geom_contype) > 0


def test_home_is_stable_and_wrist_targets_move_the_physics(robot_model):
    data = mujoco.MjData(robot_model)
    mujoco.mj_step(robot_model, data, nstep=1000)
    np.testing.assert_allclose(data.qpos, 0, atol=1e-6)
    for name, target in (("r1pro/left_arm_joint7", 0.05), ("r1pro/right_arm_joint7", -0.05)):
        data.ctrl[robot_model.actuator(name).id] = target
    mujoco.mj_step(robot_model, data, nstep=1000)
    assert data.joint("r1pro/left_arm_joint7").qpos[0] == pytest.approx(0.05, abs=1e-4)
    assert data.joint("r1pro/right_arm_joint7").qpos[0] == pytest.approx(-0.05, abs=1e-4)
    assert np.max(np.abs(data.qvel)) < 1e-4


def test_scene_joints_and_dynamic_entities_do_not_enter_robot_action_order(tmp_path):
    (tmp_path / "room.xml").write_text(
        '<mujoco><worldbody><body name="scene_body" pos="3 0 1">'
        '<freejoint name="scene_free"/><geom type="sphere" size=".1" mass="1"/>'
        "</body></worldbody></mujoco>"
    )
    (tmp_path / "scene.meta.json").write_text(
        json.dumps(
            {
                "source_path": "room.xml",
                "alignment": {},
                "artifact_frames": {"mujoco": "dimos_world"},
                "artifacts": {"mujoco_scene": "room.xml"},
                "entities": [
                    {
                        "id": "mug",
                        "spawn": "initial",
                        "initial_pose": {"x": 4, "z": 1},
                        "descriptor": {
                            "entity_id": "mug",
                            "kind": "dynamic",
                            "mass": 0.1,
                            "shape_hint": "box",
                            "extents": [0.1, 0.1, 0.1],
                        },
                    }
                ],
            }
        )
    )
    path = prepare_r1pro_act_scene(tmp_path / "composed.xml", scene_package=tmp_path)
    model = mujoco.MjModel.from_xml_path(str(path))
    spec = RobotSimSpec(
        robot_id="r1pro",
        hardware_joints=R1PRO_SIM_ACT_JOINTS,
        model_joint_names=R1PRO_SIM_ACT_JOINTS,
        model_actuator_names=R1PRO_SIM_ACT_JOINTS,
    )
    binding = resolve_robot_sim_binding(model, spec, build_joint_mappings(path, model))
    assert binding.joint_qpos_adrs == tuple(range(7, 25))
    assert model.joint("entity:mug:free").type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert model.nq == len(R1PRO_SIM_ACT_JOINTS) + 14
