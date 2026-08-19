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

from dimos.robot.unitree.go2.sim.ranges import PHYSICS_KEYS

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

# Leg dofs in qvel/dof indexing: 6 free-joint dofs, then the twelve joints.
LEG_DOFS = slice(6, 18)

# The rope, as MuJoCo can express it: a mocap body welded to the trunk, so the
# trunk is held DURING mj_step and gravity loads the legs. (A post-step snap
# lets the whole robot free-fall within the step and the legs end up in a
# weightless plant — see BaseCondition.PINNED.) Stiff on purpose: with these
# values the hold error is sub-micrometre / sub-microradian over seconds, so
# the weld is a rigid grip on the measured pose, not a spring to identify.
ANCHOR_BODY = "trunk_anchor"
WELD_SOLREF = (0.004, 1.0)  # timeconst = 2 * the scene's 2 ms step, critical damping
WELD_SOLIMP = (0.999, 0.9999, 0.001, 0.5, 2.0)

# The recorded pose drawn beside the sim under --view. Visual only: its geom
# collides with nothing, so the watched physics is the scored physics.
GHOST_BODY = "ghost"


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
    if pinned:
        spec.worldbody.add_body(name=ANCHOR_BODY, mocap=True)
        weld = spec.add_equality()
        weld.type = mujoco.mjtEq.mjEQ_WELD  # type: ignore[attr-defined]  # absent from the bundled stubs
        weld.objtype = mujoco.mjtObj.mjOBJ_BODY
        weld.name1 = ANCHOR_BODY
        weld.name2 = "base"
        weld.data[:] = 0.0
        weld.data[6] = 1.0  # relpose quat = identity: base coincides with the anchor
        weld.data[10] = 1.0  # torquescale: the rope reacts torque, not just force
        weld.solref = WELD_SOLREF
        weld.solimp = WELD_SOLIMP
    if ghost:
        body = spec.worldbody.add_body(name=GHOST_BODY, mocap=True)
        geom = body.add_geom()
        geom.type = mujoco.mjtGeom.mjGEOM_BOX
        geom.size = [0.1881, 0.04675, 0.057]  # the URDF base collision box
        geom.rgba = [0.2, 1.0, 0.2, 0.35]
        geom.contype = 0
        geom.conaffinity = 0
    model = spec.compile()
    return model, mujoco.MjData(model)


def mocap_index(model: mujoco.MjModel, name: str) -> int:
    """The mocap slot of a named mocap body — never assume it is 0."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        raise KeyError(f"no body named {name!r} in this model")
    return int(model.body_mocapid[bid])


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
        model.opt.iterations = int(round(overrides["solver_iterations"]))
    if "solver_ls_iterations" in overrides:
        model.opt.ls_iterations = int(round(overrides["solver_ls_iterations"]))
    if "solver_cone" in overrides:
        model.opt.cone = int(round(overrides["solver_cone"]))
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
