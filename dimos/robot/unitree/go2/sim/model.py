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

"""The MuJoCo Go2: menagerie scene loading, motor index maps, physics overrides."""

from __future__ import annotations

import os
from pathlib import Path

import mujoco
import numpy as np

from dimos.robot.unitree.go2.sim.ranges import PHYSICS_KEYS

# Where the accelerometer actually is: the menagerie model's `imu` site,
# (-0.02557, 0, 0.04232) — 49 mm from the body origin. The virtual IMU must be
# read THERE, not at the trunk frame; see backend.MujocoBackend.rollout.
IMU_SITE = "imu"

FOOT_GEOMS = ("FL", "FR", "RL", "RR")
FOOT_RADIUS = 0.022

# Leg dofs in qvel/dof indexing: 6 free-joint dofs, then the twelve joints.
LEG_DOFS = slice(6, 18)


def scene_path(menagerie: Path | None = None) -> Path:
    """Path to the flat-ground go2 scene (menagerie ``unitree_go2/scene.xml``).

    Assets are not vendored: point ``MUJOCO_MENAGERIE`` at a mujoco_menagerie
    checkout, or install ``mujoco_playground``, which manages one.
    """
    root = menagerie or _menagerie_root()
    scene = root / "unitree_go2" / "scene.xml"
    if not scene.is_file():
        raise FileNotFoundError(
            f"go2 scene not found at {scene}. Point MUJOCO_MENAGERIE at a "
            "mujoco_menagerie checkout."
        )
    return scene


def _menagerie_root() -> Path:
    env = os.environ.get("MUJOCO_MENAGERIE")
    if env:
        return Path(env)
    # mujoco_playground maintains a checkout of its own, but importing it drags
    # in mjx -> mujoco_warp, which fails on an mjtEnableBit the installed mujoco
    # does not have. That surfaces as AttributeError from a third-party module,
    # not ImportError, so catch broadly: this is a PATH LOOKUP, and no failure
    # inside an optional dependency should be able to take the package down.
    try:
        from mujoco_playground._src import mjx_env

        return Path(str(mjx_env.MENAGERIE_PATH))
    except Exception:
        pass
    raise FileNotFoundError(
        "no menagerie checkout: set MUJOCO_MENAGERIE to a mujoco_menagerie "
        "clone, or install mujoco_playground"
    )


def load(menagerie: Path | None = None) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(scene_path(menagerie)))
    return model, mujoco.MjData(model)


def apply_physics(model: mujoco.MjModel, overrides: dict[str, float]) -> None:
    """Patch plant knob values onto a compiled model, in place.

    Keys are :data:`~dimos.robot.unitree.go2.sim.ranges.PHYSICS_KEYS`; anything
    else raises. An ABSENT key is never written, so the menagerie default
    stands and a preset that omits a knob reproduces older scores bit-for-bit.

    The foot geom has contact priority 1 and the floor 0, so MuJoCo takes the
    FOOT's friction/solref/solimp verbatim for the pair and ignores the
    floor's: a "softer floor" is expressed by softening the foot.
    """
    unknown = set(overrides) - PHYSICS_KEYS
    if unknown:
        raise ValueError(f"unknown physics override(s): {sorted(unknown)}")
    if "armature" in overrides:
        model.dof_armature[LEG_DOFS] = overrides["armature"]
    if "damping" in overrides:
        model.dof_damping[LEG_DOFS] = overrides["damping"]
    if "frictionloss" in overrides:
        model.dof_frictionloss[LEG_DOFS] = overrides["frictionloss"]
    trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    if "trunk_mass_scale" in overrides:
        model.body_mass[trunk] *= overrides["trunk_mass_scale"]
    if "trunk_inertia_scale" in overrides:
        model.body_inertia[trunk] *= overrides["trunk_inertia_scale"]
    if "trunk_com_x" in overrides:
        model.body_ipos[trunk][0] += overrides["trunk_com_x"]
    if "leg_mass_scale" in overrides:
        for prefix in FOOT_GEOMS:
            for part in ("thigh", "calf"):
                bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}_{part}")
                model.body_mass[bid] *= overrides["leg_mass_scale"]
                model.body_inertia[bid] *= overrides["leg_mass_scale"]
    # geom_friction columns are (tangential, torsional, rolling); solref is
    # (timeconst, dampratio); solimp is (dmin, dmax, width, mid, power). Only
    # dmin and width are writable: dmax, mid and power shape the impedance
    # sigmoid without changing what it starts and ends at, and nothing in the
    # data could tell them apart from width.
    for name in FOOT_GEOMS:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
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
    return np.array(
        [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in FOOT_GEOMS], dtype=int
    )
