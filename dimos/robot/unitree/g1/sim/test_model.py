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

"""The G1 plant: pinned bytes, the layout the engine assumes, knobs that bind.

These FAIL rather than skip when the meshes are missing: the plant is a
delta on those bytes, and the unmarked viewer test already needs them.
"""

import hashlib
from pathlib import Path

import mujoco
import numpy as np
import pytest

from dimos.robot.unitree.g1.sim import model as g1_model
from dimos.robot.unitree.g1.sim.plant import LEG_DOFS, TORQUE_LIMITS
from dimos.robot.unitree.g1.sim.ranges import CONTACT_DEFAULTS, ENGINE_DEFAULTS, KNOBS
from dimos.utils.data import LfsPath


def _tree_sha256(root: Path) -> str:
    lines = []
    for f in sorted(p for p in root.rglob("*") if p.is_file()):
        lines.append(
            f"{hashlib.sha256(f.read_bytes()).hexdigest()}  {f.relative_to(root).as_posix()}"
        )
    return hashlib.sha256(("\n".join(lines) + "\n").encode()).hexdigest()


@pytest.fixture(scope="module")
def model() -> mujoco.MjModel:
    return g1_model.load()[0]


def test_the_base_model_is_pinned():
    assert hashlib.sha256(g1_model.ROBOT_MJCF.read_bytes()).hexdigest() == g1_model.MJCF_SHA256
    assert _tree_sha256(Path(str(LfsPath("g1_urdf/meshes")))) == g1_model.MESH_TREE_SHA256


def test_the_layout_the_engine_assumes(model):
    """One free joint, then one actuated hinge per DOF; the 12 leg DOFs first."""
    assert model.nu == 29 and model.nq == 7 + model.nu and model.nv == 6 + model.nu
    assert model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE and model.jnt_qposadr[1] == 7
    legs = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, 1 + i) for i in range(12)]
    assert all(("hip" in n or "knee" in n or "ankle" in n) for n in legs), legs
    assert LEG_DOFS == slice(6, 18)
    assert np.array_equal(TORQUE_LIMITS, model.jnt_actfrcrange[1:, 1])


def test_the_feet_are_eight_named_spheres_at_priority_one(model):
    ids = g1_model.foot_geom_ids(model)
    assert len(ids) == 8
    assert all(model.geom_type[ids] == mujoco.mjtGeom.mjGEOM_SPHERE)
    assert np.allclose(model.geom_size[ids, 0], g1_model.FOOT_RADIUS)
    assert all(model.geom_priority[ids] == 1)
    assert all(model.geom_contype[ids] != 0)


def test_naming_the_feet_changes_no_physics():
    """Priority 1 on a foot whose contact values equal the floor's is a no-op.

    Step the raw attach (unnamed, priority 0) and the plant's load() through
    the same PD hold from the default pose: bit-identical, or the "stock"
    preset is not the untouched model.
    """
    from dimos.robot.unitree.g1.sim.plant import KD, KP

    def run(compiled: mujoco.MjModel) -> np.ndarray:
        data = mujoco.MjData(compiled)
        hold = data.qpos[7:].copy()
        for _ in range(200):
            data.ctrl[:] = KP * (hold - data.qpos[7:]) - KD * data.qvel[6:]
            mujoco.mj_step(compiled, data)
        return data.qpos.copy()

    raw = mujoco.MjSpec.from_file(str(LfsPath("mujoco_sim/scene_empty.xml")))
    robot = mujoco.MjSpec.from_file(str(g1_model.ROBOT_MJCF))
    robot.meshdir = str(LfsPath("g1_urdf/meshes"))
    raw.option.timestep = robot.option.timestep
    raw.attach(robot, frame=raw.worldbody.add_frame(), prefix="")
    assert np.array_equal(run(raw.compile()), run(g1_model.load()[0]))


def test_engine_defaults_are_what_the_scene_compiles_to(model):
    ids = g1_model.foot_geom_ids(model)
    assert model.opt.iterations == ENGINE_DEFAULTS["solver_iterations"]
    assert model.opt.ls_iterations == ENGINE_DEFAULTS["solver_ls_iterations"]
    assert model.opt.cone == ENGINE_DEFAULTS["solver_cone"]
    assert np.allclose(model.geom_solref[ids, 0], CONTACT_DEFAULTS["foot_solref_time"])
    assert np.allclose(model.geom_solref[ids, 1], CONTACT_DEFAULTS["foot_solref_damp"])
    assert np.allclose(model.geom_solimp[ids, 0], CONTACT_DEFAULTS["foot_solimp_dmin"])
    assert np.allclose(model.geom_solimp[ids, 2], CONTACT_DEFAULTS["foot_solimp_width"])


def test_every_knob_lands_where_it_says():
    m = g1_model.load()[0]
    trunk = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, g1_model.BASE_BODY)
    mass0, inertia0, ipos0 = (
        m.body_mass[trunk],
        m.body_inertia[trunk].copy(),
        m.body_ipos[trunk].copy(),
    )
    leg0 = m.body_mass[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_knee_link")]
    values = {k: (KNOBS[k].lo + KNOBS[k].hi) / 2 for k in KNOBS if k != "actuator_tau"}
    values["solver_cone"] = 1
    g1_model.apply_physics(m, values)
    ids = g1_model.foot_geom_ids(m)
    assert np.allclose(m.dof_armature[LEG_DOFS], values["armature"])
    assert np.allclose(m.dof_damping[LEG_DOFS], values["damping"])
    assert np.allclose(m.dof_frictionloss[LEG_DOFS], values["frictionloss"])
    assert np.allclose(m.dof_armature[18:], 0.01), "waist and arms keep the MJCF default"
    assert np.isclose(m.body_mass[trunk], mass0 * values["trunk_mass_scale"])
    assert np.allclose(m.body_inertia[trunk], inertia0 * values["trunk_inertia_scale"])
    assert np.isclose(m.body_ipos[trunk][0], ipos0[0] + values["trunk_com_x"])
    knee = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left_knee_link")
    assert np.isclose(m.body_mass[knee], leg0 * values["leg_mass_scale"])
    assert np.allclose(m.geom_friction[ids, 0], values["foot_friction"])
    assert np.allclose(m.geom_friction[ids, 1], values["foot_friction_torsional"])
    assert np.allclose(m.geom_solref[ids, 0], values["foot_solref_time"])
    assert np.allclose(m.geom_solref[ids, 1], values["foot_solref_damp"])
    assert np.allclose(m.geom_solimp[ids, 0], values["foot_solimp_dmin"])
    assert np.allclose(m.geom_solimp[ids, 2], values["foot_solimp_width"])
    assert m.opt.cone == 1
    with pytest.raises(ValueError, match="unknown physics override"):
        g1_model.apply_physics(m, {"gravity": 9.81})


def test_the_rig_adds_no_joints():
    plain = g1_model.load()[0]
    rigged = g1_model.load(pinned=True, ghost=True)[0]
    assert (rigged.nq, rigged.nv, rigged.nu) == (plain.nq, plain.nv, plain.nu)
    assert rigged.nmocap == 2 and rigged.neq == 1
