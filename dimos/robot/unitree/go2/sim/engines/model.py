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

"""The Go2 plant as a compiled model: the vendored scene, and the knobs applied.

Below the seam but above any one engine: :mod:`mujoco` compiles the MJCF here
and every MuJoCo-family backend (CPU :mod:`~dimos.robot.unitree.go2.sim.engines.mujoco`,
batched :mod:`~dimos.robot.unitree.go2.sim.engines.mjx`) consumes the same
:class:`mujoco.MjModel`. Keeping :func:`load` and :func:`apply_physics` in one
place is what makes "the same plant under two engines" a structural fact
rather than a claim.
"""

from __future__ import annotations

import os
from pathlib import Path

import mujoco
import numpy as np

from dimos.robot.unitree.go2.sim.plant import TORQUE_LIMITS
from dimos.robot.unitree.go2.sim.ranges import KNOBS, PHYSICS_KEYS
from dimos.simulation.sysid.engines.model import add_rig

__all__ = ["KNOBS", "PHYSICS_KEYS", "TORQUE_LIMITS"]  # the Plant contract's tables

# The base model this package's fitted knobs are DELTAS on: the menagerie Go2
# at this exact commit, vendored byte-identical under `data/go2_menagerie`
# (via `get_data`, with menagerie's aggregate LICENSE alongside — the
# unitree_go2 section is BSD-3-Clause, Unitree Robotics; PROVENANCE.md in
# the archive has the full story). Pinning matters because every fitted
# number in a preset means "this value, applied to THESE bytes": an upstream
# retune of the go2's inertia would silently re-meaning every preset.
# test_vendor.py holds the tree hash against the vendored files.
MENAGERIE_REPO = "https://github.com/google-deepmind/mujoco_menagerie"
MENAGERIE_COMMIT = "4c358ef9d9d7f32ca58b40b490884a0c1726a440"  # 2026-06-04
# sha256 of the sorted per-file manifest of the vendored tree; recipe in
# data/go2_menagerie/PROVENANCE.md.
MENAGERIE_TREE_SHA256 = "4ef7ea987b7636c53b7c93910eacdf6e5c0cb763e9f3a7b241674b2c2f1d4a9b"

# Where the accelerometer actually is: the menagerie model's `imu` site,
# (-0.02557, 0, 0.04232) — 49 mm from the body origin. The virtual IMU must be
# read THERE, not at the trunk frame; see MujocoBackend.rollout below.
IMU_SITE = "imu"

FOOT_GEOMS = ("FL", "FR", "RL", "RR")
FOOT_RADIUS = 0.022

# The trunk: welded to the rope when suspended, the IMU's fallback frame.
BASE_BODY = "base"

# The ghost box: the URDF base collision box, centred on the base origin.
GHOST_BOX = ((0.1881, 0.04675, 0.057), (0.0, 0.0, 0.0))

# Leg dofs in qvel/dof indexing: 6 free-joint dofs, then the twelve joints.
LEG_DOFS = slice(6, 18)


def scene_path(menagerie: Path | None = None) -> Path:
    """Path to the flat-ground go2 scene (menagerie ``unitree_go2/scene.xml``).

    Resolution: the vendored snapshot at :data:`MENAGERIE_COMMIT`
    (``get_data("go2_menagerie")``), unless ``MUJOCO_MENAGERIE`` points at a
    developer checkout — an explicit override, and off the pinned bytes.
    """
    root = menagerie or _menagerie_root()
    scene = root / "unitree_go2" / "scene.xml"
    if not scene.is_file():
        raise FileNotFoundError(
            f"go2 scene not found at {scene}: broken MUJOCO_MENAGERIE "
            "override, or a stale/partial data/go2_menagerie extraction"
        )
    return scene


def _menagerie_root() -> Path:
    # The explicit developer override wins; the default is the vendored
    # snapshot, whose absence is a real failure (get_data raises), never a
    # skip. The old mujoco_playground fallback is gone: its import chain
    # reads an mjtEnableBit the pinned mujoco does not define, so it never
    # resolved here — and its checkout would be unpinned anyway.
    env = os.environ.get("MUJOCO_MENAGERIE")
    if env:
        return Path(env)
    from dimos.utils.data import get_data

    return Path(get_data("go2_menagerie"))


def load(
    menagerie: Path | None = None, *, pinned: bool = False, ghost: bool = False
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """The compiled scene; ``pinned`` adds the rope (mocap anchor + weld),
    ``ghost`` a translucent mocap box a viewer drives with the recorded pose.

    The plain path compiles the menagerie XML untouched — bit-identical to
    every rollout ever scored on it. The PINNED path adds a mocap body and an
    ``mjEQ_WELD`` holding the trunk to it; the mocap body carries no joints
    and no geoms, so nq/nv and every contact pair are unchanged. The GHOST
    body is mocap with a contype/conaffinity 0 geom — visual only, it can
    touch nothing — so attaching it never moves the physics being watched.
    """
    if not pinned and not ghost:
        model = mujoco.MjModel.from_xml_path(str(scene_path(menagerie)))
        return model, mujoco.MjData(model)
    spec = mujoco.MjSpec.from_file(str(scene_path(menagerie)))
    add_rig(spec, pinned=pinned, ghost=ghost, base_body=BASE_BODY, ghost_box=GHOST_BOX)
    model = spec.compile()
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
    # Solver settings are model OPTIONS, applied here for the same reason the
    # geom knobs are: a preset must fully determine the plant, and both
    # MuJoCo-family engines compile this one model (MJX reads opt.iterations,
    # opt.ls_iterations and opt.cone from it verbatim). Integer-valued and
    # rounded, so a preset JSON's 1.0 and an int 1 mean the same solver.
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
