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

"""xArm MuJoCo scenes: arm weight cancelled within actuator limits, shapes rest on the table."""

from pathlib import Path

import mujoco
import numpy as np
import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.xarm.blueprints.basic import xarm_connection
from dimos.robot.manipulators.xarm.config import XArmType
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.simulation.utils.xml_parser import build_joint_mappings
from dimos.utils.data import LfsPath

pytestmark = pytest.mark.self_hosted

SCENES = [
    "xarm6/scene.xml",
    "xarm7/scene.xml",
    "xarm_grasp_sim/scene.xml",
    "robosuite/lift/scene.xml",
]


@pytest.mark.parametrize("scene", SCENES)
def test_arm_gravity_compensation_is_force_limited(scene: str) -> None:
    model = mujoco.MjModel.from_xml_path(str(LfsPath(scene)))
    arm = [model.joint(f"joint{i}").id for i in range(1, 8) if f"joint{i}" in _joints(model)]

    assert len(arm) >= 6
    assert model.body_gravcomp[model.body("link1").id] == 1.0
    for joint in arm:
        a = int(np.flatnonzero(model.actuator_trnid[:, 0] == joint)[0])
        assert model.jnt_actgravcomp[joint]
        assert model.jnt_actfrclimited[joint]
        np.testing.assert_allclose(model.jnt_actfrcrange[joint], model.actuator_forcerange[a])


@pytest.mark.parametrize("scene", ["xarm6/scene.xml", "xarm7/scene.xml"])
def test_shapes_stay_on_the_table(scene: str) -> None:
    model = mujoco.MjModel.from_xml_path(str(LfsPath(scene)))
    data = mujoco.MjData(model)
    balls = ("purple_ball", "teal_ball")
    for ball in balls:
        dof = model.jnt_dofadr[model.body(ball).jntadr[0]]
        data.qvel[dof] = 1.0  # roll fast toward the front rim
    for _ in range(int(3.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)

    shapes = [
        model.body(b).name
        for b in range(model.nbody)
        if model.body_jntnum[b]
        and model.jnt_type[model.body_jntadr[b]] == mujoco.mjtJoint.mjJNT_FREE
    ]
    assert len(shapes) == 7, f"expected the 7 loose shapes, found {shapes}"
    for name in shapes:
        assert data.body(name).xpos[2] > 0.12, f"{name} fell off the table"
        assert np.linalg.norm(data.body(name).cvel) < 0.01, f"{name} is still moving"


@pytest.mark.parametrize("arm", ["xarm6", "xarm7"])
def test_default_sim_start_pose_clears_the_table(arm: XArmType) -> None:
    _, sim_bp = xarm_connection(arm, simulated=True)
    sim = next(
        atom for atom in autoconnect(sim_bp).solidify().blueprints if atom.module is MujocoSimModule
    )
    model = mujoco.MjModel.from_xml_path(sim.kwargs["address"])
    data = mujoco.MjData(model)
    reset = sim.kwargs["reset_joint_positions"]
    mappings = build_joint_mappings(Path(sim.kwargs["address"]), model)
    for mapping, angle in zip(mappings[: len(reset)], reset, strict=True):
        assert mapping.qpos_adr is not None
        data.qpos[mapping.qpos_adr] = angle
    mujoco.mj_forward(model, data)

    table = model.geom("table_top").id
    arm_root = model.body_rootid[model.body("link_base").id]
    touching_bodies = {
        model.body(model.geom_bodyid[geom]).name
        for contact in data.contact
        if table in contact.geom
        for geom in contact.geom
        if model.body_rootid[model.geom_bodyid[geom]] == arm_root
    }
    assert not touching_bodies


def _joints(model: mujoco.MjModel) -> set[str]:
    return {model.joint(j).name for j in range(model.njnt)}
