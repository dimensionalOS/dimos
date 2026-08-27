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

"""The G1 plant as a compiled model: the pinned MJCF, and the knobs applied.

The module satisfies :class:`dimos.simulation.sysid.engines.model.Plant`.
The base model is the in-repo ``g1_29dof.xml`` plus the blueprint's empty
scene; both the XML and the mesh tree are pinned by hash (``test_model``)
because every fitted knob is a delta on those bytes.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from dimos.robot.unitree.g1.sim.plant import LEG_DOFS, TORQUE_LIMITS
from dimos.robot.unitree.g1.sim.ranges import KNOBS, PHYSICS_KEYS
from dimos.simulation.sysid.engines.model import add_rig
from dimos.utils.data import LfsPath

__all__ = ["KNOBS", "PHYSICS_KEYS", "TORQUE_LIMITS"]  # the Plant contract's tables

ROBOT_MJCF = Path(__file__).resolve().parents[1] / "assets" / "g1_29dof.xml"
# Pinned by test_model.py; bumping either hash re-means every preset.
MJCF_SHA256 = "90234b20b86c5a962d74aa71dc78760a7278597d3367af8db7a04831c7717546"
MESH_TREE_SHA256 = "10423a7f8793a42fc7f6c377a9e2e1b6145e97433e86c8af20e7327512c000ba"

# The GR00T MJCF's `imu` site sits in torso_link (-0.03959, -0.00224, 0.13792).
IMU_SITE = "imu"

# The trunk: where the mass (9.6 kg vs the 3.8 kg pelvis), the IMU, the lidar
# and any unweighed payload are, so what the trunk knobs would absorb; welded
# to the rope when suspended. The pelvis stays the free-joint root.
BASE_BODY = "torso_link"

# A torso-sized box standing on the pelvis origin, for the viewer's ghost.
GHOST_BOX = ((0.13, 0.11, 0.30), (0.0, 0.0, 0.18))

# Each ankle_roll_link carries four unnamed 5 mm spheres; load() names them.
FOOT_GEOMS = tuple(f"{side}_foot_{i}" for side in ("left", "right") for i in range(4))
FOOT_RADIUS = 0.005
_ANKLE_BODIES = ("left_ankle_roll_link", "right_ankle_roll_link")

LEG_BODIES = tuple(
    f"{side}_{part}_link"
    for side in ("left", "right")
    for part in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
)


def spec(mjcf: Path = ROBOT_MJCF) -> mujoco.MjSpec:
    """The blueprint's empty scene with the robot attached, feet named."""
    scene = mujoco.MjSpec.from_file(str(LfsPath("mujoco_sim/scene_empty.xml")))
    robot = mujoco.MjSpec.from_file(str(mjcf))
    robot.meshdir = str(LfsPath("g1_urdf/meshes"))
    scene.option.timestep = robot.option.timestep
    scene.attach(robot, frame=scene.worldbody.add_frame(), prefix="")
    # Name the foot spheres and raise them to contact priority 1: MuJoCo then
    # takes the FOOT's friction/solref/solimp verbatim for every foot-floor
    # pair instead of mixing them with the floor's. At stock the two are
    # equal, so this changes nothing (held by test_model); it is what makes a
    # foot override bind at all.
    for body_name, side in zip(_ANKLE_BODIES, ("left", "right"), strict=True):
        spheres = [g for g in scene.body(body_name).geoms if g.type == mujoco.mjtGeom.mjGEOM_SPHERE]
        if len(spheres) != 4:
            raise ValueError(f"{body_name}: expected 4 foot spheres, found {len(spheres)}")
        for i, g in enumerate(spheres):
            g.name = f"{side}_foot_{i}"
            g.priority = 1
    return scene


def load(
    *, pinned: bool = False, ghost: bool = False, mjcf: Path = ROBOT_MJCF
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """The compiled scene; ``pinned`` adds the rope, ``ghost`` the viewer's box."""
    s = spec(mjcf)
    add_rig(s, pinned=pinned, ghost=ghost, base_body=BASE_BODY, ghost_box=GHOST_BOX)
    model = s.compile()
    return model, mujoco.MjData(model)


def apply_physics(model: mujoco.MjModel, overrides: dict[str, float]) -> None:
    """Patch plant knob values onto a compiled model, in place.

    Keys are :data:`~dimos.robot.unitree.g1.sim.ranges.PHYSICS_KEYS`; anything
    else raises. An ABSENT key is never written, so the stock value stands.
    Joint knobs apply to the 12 leg joints only; waist and arms keep the MJCF
    default.
    """
    unknown = set(overrides) - PHYSICS_KEYS
    if unknown:
        raise ValueError(f"unknown physics override(s): {sorted(unknown)}")
    if "solver_iterations" in overrides:
        model.opt.iterations = round(overrides["solver_iterations"])
    if "solver_ls_iterations" in overrides:
        model.opt.ls_iterations = round(overrides["solver_ls_iterations"])
    if "solver_cone" in overrides:
        model.opt.cone = round(overrides["solver_cone"])
    if "armature" in overrides:
        model.dof_armature[LEG_DOFS] = overrides["armature"]
    if "damping" in overrides:
        model.dof_damping[LEG_DOFS] = overrides["damping"]
    if "frictionloss" in overrides:
        model.dof_frictionloss[LEG_DOFS] = overrides["frictionloss"]
    trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)
    if "trunk_mass_scale" in overrides:
        model.body_mass[trunk] *= overrides["trunk_mass_scale"]
    if "trunk_inertia_scale" in overrides:
        model.body_inertia[trunk] *= overrides["trunk_inertia_scale"]
    if "trunk_com_x" in overrides:
        model.body_ipos[trunk][0] += overrides["trunk_com_x"]
    if "leg_mass_scale" in overrides:
        for name in LEG_BODIES:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            model.body_mass[bid] *= overrides["leg_mass_scale"]
            model.body_inertia[bid] *= overrides["leg_mass_scale"]
    # geom_friction is (tangential, torsional, rolling); solref (timeconst,
    # dampratio); solimp (dmin, dmax, width, mid, power), of which only dmin
    # and width are writable, as on the Go2.
    for gid in foot_geom_ids(model):
        if "foot_friction" in overrides:
            model.geom_friction[gid, 0] = overrides["foot_friction"]
        if "foot_friction_torsional" in overrides:
            model.geom_friction[gid, 1] = overrides["foot_friction_torsional"]
        if "foot_solref_time" in overrides:
            model.geom_solref[gid, 0] = overrides["foot_solref_time"]
        if "foot_solref_damp" in overrides:
            model.geom_solref[gid, 1] = overrides["foot_solref_damp"]
        if "foot_solimp_dmin" in overrides:
            model.geom_solimp[gid, 0] = overrides["foot_solimp_dmin"]
        if "foot_solimp_width" in overrides:
            model.geom_solimp[gid, 2] = overrides["foot_solimp_width"]


def foot_geom_ids(model: mujoco.MjModel) -> np.ndarray:
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in FOOT_GEOMS]
    if min(ids) < 0:
        raise KeyError("foot spheres are unnamed: this model was not compiled by g1.sim.model.load")
    return np.array(ids, dtype=int)
