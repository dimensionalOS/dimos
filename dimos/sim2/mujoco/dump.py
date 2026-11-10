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

"""Serialize the world MuJoCo is about to be handed, so two builds of it can be diffed.

A world that stands up and a world that collapses differ somewhere, and "somewhere" is the
compiled model: its actuators, its joints, the pose it starts in, the solver that steps it.
Reading that off a running sim reads it through everything else the sim is doing, so this writes
it out at the one moment it is fully determined and nothing has stepped yet.

Three files, because each answers a different question:

  * `world.xml`     the spec in MuJoCo's own words: exact, and exactly as unreadable as the
                    world is large. For "is this prim even here".
  * `model.txt`     MuJoCo's own dump of the compiled model: every array, every resolved value.
  * `summary.txt`   the physics, keyed by name and sorted: actuator gains and limits, joint
                    ranges, damping and armature, the starting pose, the solver options, the
                    contact settings.

`summary.txt` is the one to diff. The other two are ordered by MuJoCo's internal ids, so one
inserted body shifts every line below it and the diff reports only that everything moved.
"""

from pathlib import Path
from typing import Any

import mujoco
import numpy as np

_JOINT_KINDS = {
    int(mujoco.mjtJoint.mjJNT_FREE): "free",
    int(mujoco.mjtJoint.mjJNT_BALL): "ball",  # type: ignore[attr-defined]
    int(mujoco.mjtJoint.mjJNT_SLIDE): "slide",
    int(mujoco.mjtJoint.mjJNT_HINGE): "hinge",
}
_QPOS_WIDTH = {"free": 7, "ball": 4, "slide": 1, "hinge": 1}
_DOF_WIDTH = {"free": 6, "ball": 3, "slide": 1, "hinge": 1}


def dump_world(spec: Any, model: mujoco.MjModel, into: Path) -> Path:
    """Write the spec, the compiled model and a per-name physics summary into `into`."""
    into.mkdir(parents=True, exist_ok=True)
    if spec is not None:
        (into / "world.xml").write_text(spec.to_xml())
    mujoco.mj_printModel(model, str(into / "model.txt"))  # type: ignore[attr-defined]
    (into / "summary.txt").write_text("\n".join(_summary(model)) + "\n")
    return into


def _summary(model: mujoco.MjModel) -> list[str]:
    return [
        *_counts(model),
        "",
        *_options(model),
        "",
        *_actuators(model),
        "",
        *_joints(model),
        "",
        *_start_pose(model),
        "",
        *_bodies(model),
        "",
        *_keys(model),
        "",
        *_contacts(model),
        "",
        *_sensors(model),
        "",
        *_equalities(model),
    ]


def _counts(model: mujoco.MjModel) -> list[str]:
    fields = (
        "nq nv nu na nbody njnt ngeom nsite ncam nlight nmesh nkey npair nexclude neq "
        "ntendon nsensor nmocap"
    ).split()
    return ["[counts]", *(f"{field} = {getattr(model, field, '?')}" for field in fields)]


def _options(model: mujoco.MjModel) -> list[str]:
    opt = model.opt
    fields = (
        "timestep iterations ls_iterations tolerance ls_tolerance impratio "
        "integrator solver cone jacobian noslip_iterations"
    ).split()
    lines = [f"{field} = {_value(getattr(opt, field))}" for field in fields]
    lines += [
        f"gravity = {_value(opt.gravity)}",
        f"wind = {_value(opt.wind)}",
        f"disableflags = {int(opt.disableflags)}",
        f"enableflags = {int(opt.enableflags)}",
    ]
    return ["[options]", *lines]


def _actuators(model: mujoco.MjModel) -> list[str]:
    lines = []
    for index in range(model.nu):
        name = _name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index) or f"actuator[{index}]"
        lines.append(
            f"{name} -> {_trntarget(model, index)}"
            f"  gainprm={_value(model.actuator_gainprm[index][:3])}"
            f"  biasprm={_value(model.actuator_biasprm[index][:3])}"
            f"  gaintype={int(model.actuator_gaintype[index])}"
            f"  biastype={int(model.actuator_biastype[index])}"
            f"  dyntype={int(model.actuator_dyntype[index])}"
            f"  ctrllimited={int(model.actuator_ctrllimited[index])}"
            f"  ctrlrange={_value(model.actuator_ctrlrange[index])}"
            f"  forcelimited={int(model.actuator_forcelimited[index])}"
            f"  forcerange={_value(model.actuator_forcerange[index])}"
            f"  gear={_value(model.actuator_gear[index][:1])}"
        )
    return ["[actuators]", *sorted(lines)]


def _joints(model: mujoco.MjModel) -> list[str]:
    lines = []
    for index in range(model.njnt):
        name = _name(model, mujoco.mjtObj.mjOBJ_JOINT, index) or f"joint[{index}]"
        kind = _JOINT_KINDS.get(int(model.jnt_type[index]), str(model.jnt_type[index]))
        first = int(model.jnt_dofadr[index])
        dofs = range(first, first + _DOF_WIDTH.get(kind, 1))
        lines.append(
            f"{name}  kind={kind}"
            f"  body={_name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.jnt_bodyid[index]))}"
            f"  limited={int(model.jnt_limited[index])}"
            f"  range={_value(model.jnt_range[index])}"
            f"  stiffness={_value(model.jnt_stiffness[index])}"
            f"  axis={_value(model.jnt_axis[index])}"
            f"  pos={_value(model.jnt_pos[index])}"
            f"  armature={_value([model.dof_armature[d] for d in dofs])}"
            f"  damping={_value([model.dof_damping[d] for d in dofs])}"
            f"  frictionloss={_value([model.dof_frictionloss[d] for d in dofs])}"
        )
    return ["[joints]", *sorted(lines)]


def _start_pose(model: mujoco.MjModel) -> list[str]:
    """qpos0 per joint name: the pose the world is in before anything commands it."""
    lines = []
    for index in range(model.njnt):
        name = _name(model, mujoco.mjtObj.mjOBJ_JOINT, index) or f"joint[{index}]"
        kind = _JOINT_KINDS.get(int(model.jnt_type[index]), "hinge")
        start = int(model.jnt_qposadr[index])
        lines.append(f"{name} = {_value(model.qpos0[start : start + _QPOS_WIDTH.get(kind, 1)])}")
    return ["[qpos0]", *sorted(lines)]


def _bodies(model: mujoco.MjModel) -> list[str]:
    """Mass and the pose each body sits at in its parent: where the robot is, and how heavy."""
    lines = []
    for index in range(model.nbody):
        name = _name(model, mujoco.mjtObj.mjOBJ_BODY, index) or f"body[{index}]"
        lines.append(
            f"{name}  parent={_name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.body_parentid[index]))}"
            f"  mass={_value(model.body_mass[index])}"
            f"  pos={_value(model.body_pos[index])}"
            f"  quat={_value(model.body_quat[index])}"
            f"  inertia={_value(model.body_inertia[index])}"
            f"  mocap={int(model.body_mocapid[index] >= 0)}"
            f"  njnt={int(model.body_jntnum[index])}"
            f"  ngeom={int(model.body_geomnum[index])}"
        )
    return ["[bodies]", *sorted(lines)]


def _keys(model: mujoco.MjModel) -> list[str]:
    lines = []
    for index in range(model.nkey):
        name = _name(model, mujoco.mjtObj.mjOBJ_KEY, index) or f"key[{index}]"
        lines += [
            f"{name}  time={_value(model.key_time[index])}",
            f"{name}.qpos = {_value(model.key_qpos[index])}",
            f"{name}.ctrl = {_value(model.key_ctrl[index])}",
        ]
    return ["[keyframes]", *lines]


def _contacts(model: mujoco.MjModel) -> list[str]:
    """Per-geom contact settings, and the explicit pairs and exclusions on top of them."""
    geoms = []
    for index in range(model.ngeom):
        name = _name(model, mujoco.mjtObj.mjOBJ_GEOM, index) or f"geom[{index}]"
        geoms.append(
            f"{name}  contype={int(model.geom_contype[index])}"
            f"  conaffinity={int(model.geom_conaffinity[index])}"
            f"  condim={int(model.geom_condim[index])}"
            f"  priority={int(model.geom_priority[index])}"
            f"  group={int(model.geom_group[index])}"
            f"  friction={_value(model.geom_friction[index])}"
            f"  solref={_value(model.geom_solref[index])}"
            f"  solimp={_value(model.geom_solimp[index][:3])}"
            f"  margin={_value(model.geom_margin[index])}"
            f"  gap={_value(model.geom_gap[index])}"
        )
    pairs = []
    for index in range(model.npair):
        pairs.append(
            f"{_name(model, mujoco.mjtObj.mjOBJ_GEOM, int(model.pair_geom1[index]))}"
            f" <-> {_name(model, mujoco.mjtObj.mjOBJ_GEOM, int(model.pair_geom2[index]))}"
            f"  condim={int(model.pair_dim[index])}"
            f"  friction={_value(model.pair_friction[index])}"
            f"  margin={_value(model.pair_margin[index])}"
        )
    excludes = []
    for index in range(model.nexclude):
        signature = int(model.exclude_signature[index])
        excludes.append(
            f"{_name(model, mujoco.mjtObj.mjOBJ_BODY, signature >> 16)}"
            f" <-> {_name(model, mujoco.mjtObj.mjOBJ_BODY, signature & 0xFFFF)}"
        )
    return [
        "[geom contacts]",
        *sorted(geoms),
        "[contact pairs]",
        *sorted(pairs),
        "[contact exclusions]",
        *sorted(excludes),
    ]


def _sensors(model: mujoco.MjModel) -> list[str]:
    lines = []
    for index in range(model.nsensor):
        name = _name(model, mujoco.mjtObj.mjOBJ_SENSOR, index) or f"sensor[{index}]"  # type: ignore[attr-defined]
        lines.append(
            f"{name}  type={int(model.sensor_type[index])}"
            f"  objtype={int(model.sensor_objtype[index])}"
            f"  obj={_name(model, int(model.sensor_objtype[index]), int(model.sensor_objid[index]))}"
            f"  dim={int(model.sensor_dim[index])}"
        )
    return ["[sensors]", *sorted(lines)]


def _equalities(model: mujoco.MjModel) -> list[str]:
    lines = []
    for index in range(model.neq):
        name = _name(model, mujoco.mjtObj.mjOBJ_EQUALITY, index) or f"equality[{index}]"  # type: ignore[attr-defined]
        lines.append(
            f"{name}  type={int(model.eq_type[index])}"
            f"  active={int(model.eq_active0[index])}"
            f"  obj1={int(model.eq_obj1id[index])}"
            f"  obj2={int(model.eq_obj2id[index])}"
        )
    return ["[equalities]", *sorted(lines)]


def _trntarget(model: mujoco.MjModel, index: int) -> str:
    kinds = {
        int(mujoco.mjtTrn.mjTRN_JOINT): mujoco.mjtObj.mjOBJ_JOINT,
        int(mujoco.mjtTrn.mjTRN_JOINTINPARENT): mujoco.mjtObj.mjOBJ_JOINT,  # type: ignore[attr-defined]
        int(mujoco.mjtTrn.mjTRN_TENDON): mujoco.mjtObj.mjOBJ_TENDON,
        int(mujoco.mjtTrn.mjTRN_SITE): mujoco.mjtObj.mjOBJ_SITE,  # type: ignore[attr-defined]
        int(mujoco.mjtTrn.mjTRN_BODY): mujoco.mjtObj.mjOBJ_BODY,  # type: ignore[attr-defined]
    }
    kind = int(model.actuator_trntype[index])
    if kind not in kinds:
        return f"trntype[{kind}]"
    return _name(model, kinds[kind], int(model.actuator_trnid[index][0])) or "?"


def _name(model: mujoco.MjModel, kind: Any, index: int) -> str:
    if index < 0:
        return ""
    return mujoco.mj_id2name(model, int(kind), index) or ""


def _value(value: Any) -> str:
    """Six decimals, so a float that merely round-trips differently is not a difference."""
    array = np.atleast_1d(np.asarray(value, dtype=np.float64))
    if array.size == 1:
        return f"{array[0]:.6f}"
    return "[" + " ".join(f"{element:.6f}" for element in array) + "]"
