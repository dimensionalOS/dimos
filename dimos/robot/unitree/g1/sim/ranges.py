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

"""Knob set and presets for the G1 plant.

Every range says where it came from. Nothing on the G1 has been weighed or
measured on a bench yet: the only prior data point is the fork
``aaryan/g1-groot-characterization``, which fitted damping 5.64e-4,
armature 0.01384 and frictionloss 3.25 on the 12 leg joints by a different
method (candidate residual ratio with a closed-loop gate) on the same
recording. A reference, not a prior; the ranges admit it.
"""

from __future__ import annotations

from dimos.simulation.sysid.presets import (
    Knob as Knob,
    Preset as Preset,
    load_preset as _load_preset,
)

KNOBS: dict[str, Knob] = {
    "armature": Knob(
        0.002, 0.05, log=True, unit="kg*m^2", why="stock 0.01; fork 0.0138; Go2 spread up to 0.049"
    ),
    "damping": Knob(
        1e-4,
        1.0,
        log=True,
        unit="N*m*s/rad",
        why="stock 0.001 and fork 5.6e-4 both say tiny; Go2 0.04-0.5; only a hanging "
        "recording resolves it, so the range is wide and the default plan pins it",
    ),
    "frictionloss": Knob(
        0.05,
        8.0,
        log=True,
        unit="N*m",
        why="stock 0.1; fork 3.25 (32x stock, outside the Go2 0.1-2.0); sealed "
        "high-ratio planetary drives make several N.m plausible",
    ),
    "actuator_tau": Knob(
        0.0005, 0.02, log=True, unit="s", why="the Go2 range; nothing G1-specific measured yet"
    ),
    "trunk_mass_scale": Knob(0.9, 1.3, why="placeholder until the robot is weighed; pinned"),
    "trunk_inertia_scale": Knob(0.8, 1.3, why="placeholder until the robot is weighed; pinned"),
    "trunk_com_x": Knob(
        -0.03, 0.03, unit="m", why="placeholder until the robot is weighed; pinned"
    ),
    "leg_mass_scale": Knob(
        0.8, 1.3, why="CAD masses from Unitree's URDF, narrower than the Go2's menagerie doubt"
    ),
    "foot_friction": Knob(0.5, 1.2, why="a floor property; stock 1.0; the floor is undeclared"),
    "foot_friction_torsional": Knob(
        0.001,
        0.02,
        log=True,
        unit="m",
        why="stock 0.005; the patch formula is meaningless for a four-sphere foot "
        "(torsion is carried by the sphere spread); pinned",
    ),
    "foot_solref_time": Knob(
        0.002,
        0.03,
        log=True,
        unit="s",
        why="stock 0.02; below ~0.01 is stiff for the 5 ms step",
    ),
    "foot_solref_damp": Knob(0.5, 2.0, why="stock 1.0; the Go2 range"),
    "foot_solimp_dmin": Knob(
        0.5, 0.99, why="stock here is MuJoCo's 0.9, not menagerie's 0.015; the range brackets it"
    ),
    "foot_solimp_width": Knob(0.001, 0.08, log=True, unit="m", why="stock 0.001; the Go2 range"),
    # Contact SOLVER settings: model options, never searched. A solver is
    # chosen and the contact is identified UNDER it. The scene compiles to
    # 100/50 and a PYRAMIDAL cone (the Go2 scene is elliptic).
    "solver_iterations": Knob(1, 200, log=True, why="Newton iteration cap; scene default 100"),
    "solver_ls_iterations": Knob(1, 100, log=True, why="line-search cap; scene default 50"),
    "solver_cone": Knob(
        0, 1, why="categorical, mjtCone: 0 = pyramidal (scene default), 1 = elliptic"
    ),
}

# The values the compiled scene carries for every key a preset MAY carry:
# an absent key is never written, so every preset that omits one reproduces
# the stock physics for it bit-for-bit. Applying all of them is a no-op
# (held by test_model), which is what lets `stock` be a complete start point
# for identify and fit.
JOINT_DEFAULTS: dict[str, float] = {
    # g1_29dof.xml joint default class.
    "armature": 0.01,
    "damping": 0.001,
    "frictionloss": 0.1,
}
MASS_DEFAULTS: dict[str, float] = {
    "trunk_mass_scale": 1.0,
    "trunk_inertia_scale": 1.0,
    "trunk_com_x": 0.0,
    "leg_mass_scale": 1.0,
}
CONTACT_DEFAULTS: dict[str, float] = {
    # MuJoCo geom defaults: friction="1 0.005 0.0001", solref="0.02 1",
    # solimp="0.9 0.95 0.001 0.5 2".
    "foot_friction": 1.0,
    "foot_friction_torsional": 0.005,
    "foot_solref_time": 0.02,
    "foot_solref_damp": 1.0,
    "foot_solimp_dmin": 0.9,
    "foot_solimp_width": 0.001,
}
SOLVER_DEFAULTS: dict[str, float] = {
    "solver_iterations": 100.0,
    "solver_ls_iterations": 50.0,
    "solver_cone": 0.0,  # mjCONE_PYRAMIDAL
}
SOLVER_KEYS = frozenset(SOLVER_DEFAULTS)
ENGINE_DEFAULTS: dict[str, float] = {
    **JOINT_DEFAULTS,
    **MASS_DEFAULTS,
    **CONTACT_DEFAULTS,
    **SOLVER_DEFAULTS,
}

# Model-override keys apply_physics accepts: every knob except the actuator lag.
PHYSICS_KEYS = frozenset(KNOBS) - {"actuator_tau"}

# Bare g1_29dof.xml: the experimental control every claim is comparative against.
STOCK = Preset(name="stock", builtin=True)

BUILTIN_PRESETS: dict[str, Preset] = {p.name: p for p in (STOCK,)}
DEFAULT_PRESET = "stock"


def load_preset(name: str | None = None) -> Preset:
    """A built-in G1 preset by name, or a JSON file a fit wrote."""
    p = _load_preset(DEFAULT_PRESET if name is None else name, BUILTIN_PRESETS)
    if p.envelope is not None:
        raise ValueError(f"{p.name!r}: the G1 has no measured torque envelope named {p.envelope!r}")
    return p
