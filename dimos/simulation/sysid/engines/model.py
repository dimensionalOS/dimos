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

"""What a MuJoCo plant module provides, and the rig every plant can wear.

Below the seam but above any one engine. A robot's plant is a MODULE
satisfying :class:`Plant` (its pinned base model, its knob table, how the
knobs land on the compiled model); the engine reads every table off it and
never names a body, geom or joint count itself. :func:`add_rig` is the one
piece of model surgery shared by all plants: the rope for a suspended
rollout and the ghost box a viewer drives with the recorded pose.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Protocol

import mujoco
import numpy as np

from dimos.simulation.sysid.presets import Knob

# The rope, as MuJoCo can express it: a mocap body welded to the base, so the
# base is held DURING mj_step and gravity loads the legs. (A post-step snap
# lets the whole robot free-fall within the step and the legs end up in a
# weightless plant; see BaseCondition.PINNED.) Stiff on purpose: with these
# values the hold error is sub-micrometre / sub-microradian over seconds, so
# the weld is a rigid grip on the measured pose, not a spring to identify.
# MuJoCo clamps timeconst to 2*dt, so a plant stepping slower than 2 ms gets
# a correspondingly softer weld.
ANCHOR_BODY = "trunk_anchor"
WELD_SOLREF = (0.004, 1.0)  # timeconst = 2 * a 2 ms step, critical damping
WELD_SOLIMP = (0.999, 0.9999, 0.001, 0.5, 2.0)

# The recorded pose drawn beside the sim under --view. Visual only: its geom
# collides with nothing, so the watched physics is the scored physics.
GHOST_BODY = "ghost"
GHOST_RGBA = (0.2, 1.0, 0.2, 0.35)


class Plant(Protocol):
    """A robot's MuJoCo plant, as a module.

    ``load`` compiles the pinned base model (plus the rig); ``apply_physics``
    lands knob values on it in place, writing only the keys present so an
    absent key keeps the base model's value; ``foot_geom_ids`` names the
    geoms a snap drops onto the floor. The engine assumes one free joint
    followed by one actuated hinge per DOF and nothing else.
    """

    KNOBS: Mapping[str, Knob]
    PHYSICS_KEYS: frozenset[str]
    TORQUE_LIMITS: np.ndarray
    FOOT_RADIUS: float  # foot geom centre to sole, m
    IMU_SITE: str
    BASE_BODY: str  # welded when suspended; the IMU fallback frame

    def load(
        self, *, pinned: bool = False, ghost: bool = False
    ) -> tuple[mujoco.MjModel, mujoco.MjData]: ...

    def apply_physics(self, model: mujoco.MjModel, overrides: dict[str, float]) -> None: ...

    def foot_geom_ids(self, model: mujoco.MjModel) -> np.ndarray: ...


def add_rig(
    spec: mujoco.MjSpec,
    *,
    pinned: bool,
    ghost: bool,
    base_body: str,
    ghost_box: tuple[tuple[float, float, float], tuple[float, float, float]],
) -> None:
    """Add the rope and/or the ghost to a spec before it compiles.

    The anchor is a mocap body with an ``mjEQ_WELD`` to ``base_body``; it
    carries no joints and no geoms, so nq/nv and every contact pair are
    unchanged. The ghost is a mocap body with a contype/conaffinity 0 box
    (``ghost_box`` = half-sizes, offset from the base origin), so attaching
    it never moves the physics being watched.
    """
    if pinned:
        spec.worldbody.add_body(name=ANCHOR_BODY, mocap=True)
        weld = spec.add_equality()
        weld.type = mujoco.mjtEq.mjEQ_WELD  # type: ignore[attr-defined]  # absent from the bundled stubs
        weld.objtype = mujoco.mjtObj.mjOBJ_BODY
        weld.name1 = ANCHOR_BODY
        weld.name2 = base_body
        weld.data[:] = 0.0
        weld.data[6] = 1.0  # relpose quat = identity: base coincides with the anchor
        weld.data[10] = 1.0  # torquescale: the rope reacts torque, not just force
        weld.solref = WELD_SOLREF
        weld.solimp = WELD_SOLIMP
    if ghost:
        size, pos = ghost_box
        body = spec.worldbody.add_body(name=GHOST_BODY, mocap=True)
        geom = body.add_geom()
        geom.type = mujoco.mjtGeom.mjGEOM_BOX
        geom.size = list(size)
        geom.pos = list(pos)
        geom.rgba = list(GHOST_RGBA)
        geom.contype = 0
        geom.conaffinity = 0


def mocap_index(model: mujoco.MjModel, name: str) -> int:
    """The mocap slot of a named mocap body; never assume it is 0."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        raise KeyError(f"no body named {name!r} in this model")
    return int(model.body_mocapid[bid])
